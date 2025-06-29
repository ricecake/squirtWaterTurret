#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "vector.hpp"
#include "fpm_adapter.hpp"

const auto FIXEDPI = fixed_16_16::pi();

using fixed = fixed_16_16;
using VelocityVector = Vector3D<fixed>;
class Target;
class Target : public Vector3D<fixed>
{
public:
	Target();
	Target(Vector3D<fixed> vec);
	Target(uint8_t index, long X, long Y, long Z = 1000, long speed = 0, bool valid = true);
	Target(uint8_t index, long X, long Y, long speed, bool valid = true);

public:
	void Update(Target &updated);
	void Update(long int, long int, long int);

public:
	fixed Pitch();
	fixed Yaw();
	long Distance();
	VelocityVector Velocity();

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
	fixed _pitch {0};
	fixed _yaw {0};
	VelocityVector _velocity;

private:
	long last_X_coord = 0;
	long last_Y_coord = 0;
	long last_Z_coord = 0;
	int64_t last_seen = 0;
	VelocityVector last_velocity;
};
