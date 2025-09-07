#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "vector.hpp"
#include "command.h"
#include "target.h"
#include "fpm_adapter.hpp"

using fixed = fixed_16_16;

class Command;

class SystemState
{
private:
	const int motorInterfaceType = 1;

	// Define pin connections
	// const int stepPinB = 32;
	// const int dirPinB = 33;
	// const int stepPinA = 25;
	// const int dirPinA = 26;
	const int stepPinA = 32;
	const int dirPinA = 33;
	const int stepPinB = 25;
	const int dirPinB = 26;

	const int firePin = 2;

public:
	// Define motor limits
	// const int maxSpeed = maxSpeed; // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
	// const int acceleration = 120;

	// Define other constants
	const int stepFraction = 16; // The microstep fraction

	const int h_max = 500;
	const int v_max = 1000;
	const int h_min = -500;
	const int v_min = -1000;

	const fixed angleToStep{0.1125}; //(360 / 200) / 1 / 16; // circle / steps per circle / gear ratio / step division

private:

public:
	AccelStepper stepperA;
	AccelStepper stepperB;
	MultiStepper steppers;

	SemaphoreHandle_t xMutex;

private:
	bool moveState = true;
	bool fireState = false;
	bool needTrackingUpdate = false;
	uint8_t trackingSpeed = 255;
	uint8_t selectedTarget = 0;

private:
	PositionVector targetAimpoint();
	Target target[4]; // Target zero is for special overrides, without messing with radar targets
	std::priority_queue<Command *, std::vector<Command *>, decltype([](auto left, auto right)
																	{ return left->run_after >= right->run_after; })>
		commandQueue;

public:
	SystemState();
	Target &currentTarget();
	void updateTarget(const uint8_t idx, const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin = 0);
	void setTarget(uint8_t index, uint8_t speed = 0xFF);
	void setFire(bool active);
	void queueSelectTarget(uint8_t index, uint16_t milliseconds);
	void queueFire(uint8_t milliseconds);
	void queueLinger(uint8_t milliseconds);
	void processCommandQueue();
	void actualizeState();
	inline Target &fetchTarget(const uint8_t idx) {
		return target[idx];
	}
	fixed targetTravelDistance();

private:
	void actualizePosition();
	void actualizeFiring();
};
