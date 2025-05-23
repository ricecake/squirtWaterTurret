#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "command.h"
#include "target.h"

class Command;

class SystemState
{
private:
	const int motorInterfaceType = 1;

	// Define pin connections
	const int stepPinB = 32;
	const int dirPinB = 33;
	const int stepPinA = 25;
	const int dirPinA = 26;

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

	const float angleToStep = 0.1125; //(360 / 200) / 1 / 16; // circle / steps per circle / gear ratio / step division

private:
	int altitude = 1320;

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
	Target target[4]; // Target zero is for special overrides, without messing with radar targets
	std::priority_queue<Command*, std::vector<Command*>, decltype([](auto left, auto right)
																	{ return left->run_after >= right->run_after; })>
		commandQueue;

public:
	SystemState();
	Target &currentTarget();
	void updateTarget(Target &, uint16_t indifferenceMargin = 0);
	void setTarget(uint8_t index, uint8_t speed = 0xFF);
	void setFire(bool active);
	void queueSelectTarget(uint8_t index, uint16_t milliseconds);
	void queueFire(uint8_t milliseconds);
	void queueLinger(uint8_t milliseconds);
	void processCommandQueue();
	void actualizeState();
	long targetTravelDistance();

private:
	void actualizePosition();
	void actualizeFiring();
};
