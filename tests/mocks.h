#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>

#include "mock_time.h"

// Mock implementations for non-Arduino environments

// From state.h
struct AccelStepper {
	static const int FULL4WIRE = 4;

	AccelStepper(int = 0, int = 0, int = 0, int = 0, int = 0, bool = true) {}

	void setAcceleration(int) {}

	void setMaxSpeed(double) {}

	void moveTo(long) {}

	long distanceToGo() { return 0; }

	void run() {}

	long currentPosition() { return 0; }
};

using SemaphoreHandle_t = int;

// From state.cpp
#define portMAX_DELAY (uint32_t)0xffffffffUL
#define pdTRUE 1
#define OUTPUT 0x01
#define HIGH 0x1
#define LOW 0x0
#define SERIAL_8N1 0

static inline SemaphoreHandle_t xSemaphoreCreateMutex() {
	return 0;
}

static inline int xSemaphoreTake(SemaphoreHandle_t, uint32_t) {
	return pdTRUE;
}

static inline void xSemaphoreGive(SemaphoreHandle_t) {}

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

	void print(const char* message) { std::cout << message; }

	void println(const char* message) { std::cout << message << std::endl; }

	operator bool() const { return true; }
};

static HardwareSerial Serial(0);

// Mock for LD2450.h
class LD2450 {
public:
	struct RadarTarget {
		bool valid;
		int  x;
		int  y;
		int  id;
	};

	void begin(HardwareSerial&, bool) {}

	int read() { return 1 + (rand() % 3); } // Return 1 to 3 targets

	RadarTarget getTarget(int) {
		RadarTarget target;
		target.valid = true;
		target.x = rand() % 2000 - 1000;
		target.y = rand() % 2000;
		target.id = rand() % 10;
		return target;
	}

	bool waitForSensorMessage(bool) { return false; }
};

// Mock for Arduino.h
using TaskHandle_t = void*;
using std::min;

static inline void randomSeed(int) {}

static inline int analogRead(int) {
	return 0;
}

static inline void delay(int) {}

static inline void vTaskDelay(int) {}

static inline void xTaskCreatePinnedToCore(void (*)(void*), const char*, int, void*, int, TaskHandle_t*, int) {}

#define portTICK_PERIOD_MS 1

// Mocks for esp_timer.h and esp32-hal-gpio.h are mostly empty as their functions are already mocked.
