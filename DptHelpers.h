#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>

// inline int64_t milliseconds(int64_t millis);
// inline int64_t seconds(int64_t seconds);

// template<typename T>
// int64_t milliseconds(T millis, int64_t offset = 0);

// template<typename T>
// int64_t seconds(T seconds, int64_t offset = 0);

template <typename T>
int64_t milliseconds(T millis, int64_t offset = 0)
{
	return offset + int64_t(1000 * millis);
}

template <typename T>
int64_t seconds(T seconds, int64_t offset = 0)
{
	return offset + int64_t(1000 * 1000 * seconds);
}

// Define motor interface type
const int motorInterfaceType = 1;
const int maxSpeed = 1000;	   // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
const int acceleration = 3000; // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.

class SystemState;

class Target
{
public:
	uint8_t index;
	int64_t seen;
	int64_t last_action;
	long X_coord = 0;
	long Y_coord = 0;
	long Z_coord = 1000;
	bool valid = false;
	long speed = 0;

private:
	long distance = 0;
	double pitch = 0;
	double yaw = 0;

public:
	Target();
	Target(uint8_t index, long X, long Y, long Z = 1000, long speed = 0, bool valid = true);
	Target(uint8_t index, long X, long Y, long speed, bool valid = true);
	double Pitch();
	double Yaw();
	long Distance();
	int64_t timeSinceLastAction();
	bool actionIdleExceeds(int64_t limit);
	void IncrementAction();
	void Update(Target &updated);
};

class Command
{
public:
	int64_t id = 0;
	int64_t run_after = 0;

	virtual void Execute(SystemState *state) = 0;
	Command(int64_t run_after);
};

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
	std::priority_queue<Command *, std::vector<Command *>, decltype([](auto left, auto right)
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

class TargetSelection : public Command
{
	uint8_t target_id;
	int speed = maxSpeed;

public:
	void Execute(SystemState *state);
	TargetSelection(uint8_t, int, int64_t);

	// This should actually just be speed and target index.
	// The radar will set the targets as it finds them, and then we schedule which one we're interested in.
};

class FireControl : public Command
{
	bool active;

public:
	void Execute(SystemState *state);
	FireControl(bool, int64_t);
};
