#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include "mock_time.h"

// Mock implementations for non-Arduino environments

// From state.h
struct AccelStepper {
	static const int FULL4WIRE = 4;
	long             target = 0;
	long             position = 0;

	AccelStepper(int = 0, int = 0, int = 0, int = 0, int = 0, bool = true) {}

	void setAcceleration(int) {}

	void setMaxSpeed(double) {}

	void moveTo(long s) { target = s; }

	long distanceToGo() { return abs(target - position); }

	void run() {
		if (position != target) {
			position += std::signbit(target - position) * std::max(1, abs(target / 5));
		}
	}

	long currentPosition() { return position; }
};

using SemaphoreHandle_t = int;

// From state.cpp
#define portMAX_DELAY (uint32_t)0xffffffffUL
#define pdTRUE 1
#define OUTPUT 0x01
#define HIGH 0x1
#define LOW 0x0
#define SERIAL_8N1 0

static inline void pinMode(int, int) {}

static inline void digitalWrite(int, int) {}

inline static TestClock mock_clock;

template <typename T>
T roundedDifference(const T& left, const T& right, const T& margin) {
	auto roundL = left - left % margin;
	auto roundR = right - right % margin;
	return roundL - roundR;
}

// New Mocks

// Mock for HardwareSerial.h
class HardwareSerial {
public:
	HardwareSerial(int) {}

	void begin(int, int = 0, int = 0, int = 0) {}

	size_t available() { return 0; }

	size_t readBytes(char*, size_t) { return 0; }

	size_t write(const uint8_t* buffer, size_t size) {
		std::cout.write(reinterpret_cast<const char*>(buffer), size);
		return size;
	}

	void print(auto message) { std::cout << message; }

	void println(auto message) { std::cout << message << std::endl; }

	operator bool() const { return true; }
};

static HardwareSerial Serial(0);

// Mock for LD2450.h
class LD2450 {
	// Constants for mock behavior
	static constexpr double MAX_RADIUS = 10000.0;
	static constexpr double TARGET_SPEED = 1000.0; // 1 m/s

	struct MockTarget {
		int       id;
		int16_t   x, y;           // Final, rounded output
		double    true_x, true_y; // High-precision internal position
		int16_t   dest_x, dest_y;
		TimePoint pause_until;
		TimePoint last_updated;
	};

	std::mt19937                           rng;
	std::uniform_real_distribution<double> angle_dist;
	std::uniform_real_distribution<double> radius_dist;
	std::uniform_int_distribution<int>     pause_dist;

public:
	std::vector<MockTarget> mock_targets;

	struct RadarTarget {
		bool     valid;
		int16_t  x;
		int16_t  y;
		uint16_t id;
	};

	LD2450(): LD2450(std::random_device{}()) {} // Default to a random seed

	explicit LD2450(unsigned int seed):
		rng(seed), angle_dist(0, 2 * M_PI), radius_dist(0, MAX_RADIUS), pause_dist(2, 7) {
		mock_targets.emplace_back(MockTarget{0, 0, 1000, 0.0, 1000.0, 0, 1000, Clock::now(), Clock::now()});
		mock_targets.emplace_back(MockTarget{1, -1000, 2000, -1000.0, 2000.0, -1000, 2000, Clock::now(), Clock::now()});
		mock_targets.emplace_back(MockTarget{2, 1000, 2000, 1000.0, 2000.0, 5000, 5000, Clock::now(), Clock::now()});
	}

	void begin(HardwareSerial&, bool) {}

	int read() { return 3; } // Return 1 to 3 targets

	RadarTarget getTarget(int id) {
		update_target(mock_targets[id]);

		RadarTarget target;
		target.valid = true;
		target.x = mock_targets[id].x;
		target.y = mock_targets[id].y;
		target.id = id;
		return target;
	}

	bool waitForSensorMessage(bool) { return false; }

private:
	void random_point_in_circle(int16_t& x, int16_t& y) {
		double r = radius_dist(rng);
		double angle = angle_dist(rng);
		x = static_cast<int16_t>(r * sin(angle));
		y = static_cast<int16_t>(r * cos(angle));
	}

	void update_target(MockTarget& target) {
		auto now = Clock::now();
		auto time_delta = std::chrono::duration_cast<std::chrono::microseconds>(now - target.last_updated).count();
		target.last_updated = now;
		double delta_t = time_delta / 1000000.0; // Convert to seconds

		if (now < target.pause_until) {
			// Target is paused, do nothing.
		} else {
			double dist_to_dest = std::hypot(target.dest_x - target.true_x, target.dest_y - target.true_y);
			double move_dist = TARGET_SPEED * delta_t;

			// If we are very close, or would overshoot, just arrive at the destination.
			if (dist_to_dest < move_dist) {
				target.true_x = target.dest_x;
				target.true_y = target.dest_y;
			} else {
				// Move towards the destination
				target.true_x += (target.dest_x - target.true_x) / dist_to_dest * move_dist;
				target.true_y += (target.dest_y - target.true_y) / dist_to_dest * move_dist;
			}
		}

		// Check for arrival and update behavior
		double final_dist_to_dest = std::hypot(target.dest_x - target.true_x, target.dest_y - target.true_y);
		if (final_dist_to_dest < 1.0) { // Arrived
			switch (target.id) {
			case 0: // Random Walk
			case 2: // Continuous Patrol
				random_point_in_circle(target.dest_x, target.dest_y);
				break;
			case 1: // Walk and Pause
				target.pause_until = now + std::chrono::seconds(pause_dist(rng));
				random_point_in_circle(target.dest_x, target.dest_y);
				break;
			}
		}

		// Constrain all targets to the circle and update final output
		double r = std::hypot(target.true_x, target.true_y);
		if (r > MAX_RADIUS) {
			// This is a simple boundary constraint. A more realistic one might "bounce".
			target.true_x = target.true_x / r * MAX_RADIUS;
			target.true_y = target.true_y / r * MAX_RADIUS;
			random_point_in_circle(target.dest_x, target.dest_y);
		}

		// Update the final, rounded output values
		target.x = static_cast<int16_t>(round(target.true_x));
		target.y = static_cast<int16_t>(round(target.true_y));
	}
};

// Mock for Arduino.h
using TaskHandle_t = void*;
using std::min;

static inline void randomSeed(int) {}

static inline int analogRead(int) {
	return 0;
}

static inline void delay(int) {}

static inline void vTaskDelay(int d) {
	std::this_thread::sleep_for(std::chrono::milliseconds(50 * d));
}

static std::vector<std::thread> threads;

static inline void xTaskCreatePinnedToCore(auto task, const char*, int, void* parameters, int, TaskHandle_t*, int) {
	threads.emplace_back(task, parameters);
}

#define portTICK_PERIOD_MS 1
