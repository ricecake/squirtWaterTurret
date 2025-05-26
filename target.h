
#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "vector.hpp"
#include "fpm/fixed.hpp"

using fixed = fpm::fixed_16_16;
using Velocity = Vector3D<fixed>;

class Target : public Vector3D<fixed>
{
public:
	Target();
	Target(uint8_t index, long X, long Y, long Z = 1000, long speed = 0, bool valid = true);
	Target(uint8_t index, long X, long Y, long speed, bool valid = true);

public:
	void Update(Target &updated);
	void Update(long int, long int, long int);

public:
	double Pitch();
	double Yaw();
	long Distance();
	Velocity velocity();

	int64_t timeSinceLastAction();
	bool actionIdleExceeds(int64_t limit);
	void IncrementAction();
	void PredictedPositionAtTime();

public:
	uint8_t index;
	int64_t seen;
	int64_t last_action;
	bool valid = false;

public:
	long X_coord = 0;
	long Y_coord = 0;
	long Z_coord = -400;
	long speed = 0;

private:
	long _distance = 0;
	double _pitch = 0;
	double _yaw = 0;
	Velocity _velocity;

private:
	long last_X_coord = 0;
	long last_Y_coord = 0;
	long last_Z_coord = 0;
	Velocity last_velocity;
};
