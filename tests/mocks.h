#pragma once

#include <chrono>
#include <cstdint>

// Mock implementations for non-Arduino environments

// From state.h
struct AccelStepper {
	AccelStepper(int = 0, int = 0, int = 0) {}
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

static inline SemaphoreHandle_t xSemaphoreCreateMutex() {
	return 0;
}
static inline int xSemaphoreTake(SemaphoreHandle_t, uint32_t) {
	return pdTRUE;
}
static inline void xSemaphoreGive(SemaphoreHandle_t) {}
static inline void pinMode(int, int) {}
static inline void digitalWrite(int, int) {}

// From command.cpp
static inline int64_t esp_timer_get_time() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}