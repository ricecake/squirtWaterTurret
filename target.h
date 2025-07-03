#pragma once

// #include <functional>
#include <stdint.h>
// #include <queue>
// #include <AccelStepper.h>
// #include <MultiStepper.h>
#include "vector.hpp"
#include "fpm_adapter.hpp"

const auto FIXEDPI = fixed_16_16::pi();

using fixed = fixed_16_16;

// class VelocityVector : public Vector3D<fixed, VelocityVector> {

// };

using VelocityVector = Vector3D<fixed>;

class Target;
class Target : public Vector3D<fixed, Target>
{
public:
	Target();
	Target(Vector3D<fixed> vec);
	Target(fixed X, fixed Y, fixed Z = 1000);
	Target(uint8_t index, fixed X, fixed Y, fixed Z, fixed speed = 0, bool valid = true);
	Target(uint8_t index, fixed X, fixed Y, fixed speed, bool valid = true);

public:
	void Update(Target &updated);
	void Update(fixed, fixed, fixed);

public:
	fixed Pitch();
	fixed Yaw();
	fixed Distance();
	const VelocityVector Velocity() const;
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

// public:
// 	long X_coord = 0;
// 	long Y_coord = 0;
// 	long Z_coord = -400;
	fixed speed = 0;

private:
	fixed _distance = 0;
	fixed _pitch {0};
	fixed _yaw {0};
	VelocityVector _velocity;

private:
	fixed last_X_coord = 0;
	fixed last_Y_coord = 0;
	fixed last_Z_coord = 0;
	int64_t last_seen = 0;
	VelocityVector last_velocity;
};
