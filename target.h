
#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>

struct Velocity {
	double dir = 0;
	long magnitude = 0;
};

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

	Velocity velocity;

	long last_X_coord = 0;
	long last_Y_coord = 0;
	long last_Z_coord = -400;
	Velocity last_velocity;

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
	void Update(long int, long int, long int);
	void Velocity();
	void PredictedPositionAtTime();
};
