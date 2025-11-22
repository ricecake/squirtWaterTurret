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
	static constexpr double CONE_ANGLE = M_PI / 3.0;
	static constexpr double WALK_PAUSE_SPEED = 100.0;
	static constexpr double PATROL_SPEED = 200.0;
	static constexpr double BOUNDARY_REDIRECT_SPEED = 150.0;

	struct MockTarget {
		int       id;
		double    x, y;
		double    dest_x, dest_y;
		TimePoint pause_until;
		TimePoint last_updated;
	};

	std::vector<MockTarget> mock_targets;
	std::mt19937                  rng;
	std::uniform_real_distribution<double> angle_dist;
	std::uniform_real_distribution<double> radius_dist;
	std::uniform_int_distribution<int>     walk_dist;
	std::uniform_int_distribution<int>     pause_dist;

public:
	bool mock_enabled = true;

	struct RadarTarget {
		bool valid;
		int  x;
		int  y;
		int  id;
	};

	LD2450() :
		rng(12345), // Fixed seed for deterministic tests
		angle_dist(-CONE_ANGLE, CONE_ANGLE),
		radius_dist(0, MAX_RADIUS),
		walk_dist(-100, 100),
		pause_dist(2, 7) {
		mock_targets.emplace_back(MockTarget{0, 0, 1000, 0, 1000, Clock::now(), Clock::now()});
		mock_targets.emplace_back(MockTarget{1, -1000, 2000, -1000, 2000, Clock::now(), Clock::now()});
		mock_targets.emplace_back(MockTarget{2, 1000, 2000, -5000, 5000, Clock::now(), Clock::now()});
	}

	void begin(HardwareSerial&, bool) {}

	int read() { return 3; } // Return 1 to 3 targets

	RadarTarget getTarget(int id) {
		if (mock_enabled) {
			update_target(mock_targets[id]);
		}
		RadarTarget target;
		target.valid = true;
		target.x = mock_targets[id].x;
		target.y = mock_targets[id].y;
		target.id = id;
		return target;
	}

	bool waitForSensorMessage(bool) { return false; }

private:
	void random_point_in_cone(double& x, double& y) {
		double r = radius_dist(rng);
		double angle = angle_dist(rng);
		x = r * sin(angle);
		y = r * cos(angle);
	}

	void update_target(MockTarget& target) {
		auto now = Clock::now();
		auto time_delta = std::chrono::duration_cast<std::chrono::microseconds>(now - target.last_updated).count();
		target.last_updated = now;
		double delta_t = time_delta / 1000000.0; // Convert to seconds

		switch (target.id) {
		case 0: {
			// Simple random walk
			target.x += walk_dist(rng) * 2 * delta_t;
			target.y += walk_dist(rng) * 2 * delta_t;
			break;
		}
		case 1: {
			// Walk to a random point, then pause
			if (now < target.pause_until)
				break;
			double dist = std::hypot(target.dest_x - target.x, target.dest_y - target.y);
			if (dist < 100) {
				target.pause_until = now + std::chrono::seconds(pause_dist(rng));
				random_point_in_cone(target.dest_x, target.dest_y);
			} else if (dist > 1e-6) {
				target.x += (target.dest_x - target.x) / dist * WALK_PAUSE_SPEED * delta_t;
				target.y += (target.dest_y - target.y) / dist * WALK_PAUSE_SPEED * delta_t;
			}
			break;
		}
		case 2: {
			// Move continuously to random points in a wide area
			double dist = std::hypot(target.dest_x - target.x, target.dest_y - target.y);
			if (dist < 500) {
				random_point_in_cone(target.dest_x, target.dest_y);
			}
			if (dist > 1e-6) {
				target.x += (target.dest_x - target.x) / dist * PATROL_SPEED * delta_t;
				target.y += (target.dest_y - target.y) / dist * PATROL_SPEED * delta_t;
			}
			break;
		}
		}

		// Constrain all targets to the cone
		double r = std::hypot(target.x, target.y);
		double angle = atan2(target.x, target.y);
		if (r > MAX_RADIUS || abs(angle) > CONE_ANGLE) {
			random_point_in_cone(target.dest_x, target.dest_y);
			// Move towards the new destination immediately
			double dist = std::hypot(target.dest_x - target.x, target.dest_y - target.y);
			if (dist > 1e-6) {
				target.x += (target.dest_x - target.x) / dist * BOUNDARY_REDIRECT_SPEED * delta_t;
				target.y += (target.dest_y - target.y) / dist * BOUNDARY_REDIRECT_SPEED * delta_t;
			}
		}
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
