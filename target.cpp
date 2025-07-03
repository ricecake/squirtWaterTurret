#include <climits>
#include <stdint.h>
// #include <Arduino.h>
#include "esp_timer.h"
#include "DptHelpers.h"

#include "target.h"
#include <math.h>
#include "fpm_adapter.hpp"
#include "aproximate_math.hpp"

Target::Target() : index(0), Vector3D<fixed, Target>(), valid(false)
{
	seen = esp_timer_get_time();
}

Target::Target(Vector3D<fixed> vec) : Vector3D<fixed, Target>(vec.X_coord, vec.Y_coord, vec.Z_coord)
{
	seen = esp_timer_get_time();
}

Target::Target(fixed X, fixed Y, fixed Z) : Vector3D<fixed, Target>(X, Y, Z)
{
	seen = esp_timer_get_time();
}

Target::Target(uint8_t index, fixed X, fixed Y, fixed speed, bool valid) : index(index), Vector3D<fixed, Target>(X, Y, 1000), speed(speed), valid(valid)
{
	seen = esp_timer_get_time();
}

Target::Target(uint8_t index, fixed X, fixed Y, fixed Z, fixed speed, bool valid) : index(index), Vector3D<fixed, Target>(X, Y, Z), speed(speed), valid(valid)
{
	seen = esp_timer_get_time();
}

void Target::Update(fixed new_X_coord, fixed new_Y_coord, fixed new_Z_coord)
{
	last_seen = this->seen;
	last_X_coord = this->X_coord;
	last_Y_coord = this->Y_coord;
	last_Z_coord = this->Z_coord;
	last_velocity = this->_velocity;

	valid = true;
	seen = esp_timer_get_time();

	if (new_X_coord)
	{
		X_coord = new_X_coord;
	}
	if (new_Y_coord)
	{
		Y_coord = new_Y_coord;
	}
	if (new_Z_coord)
	{
		Z_coord = new_Z_coord;
	}

	if (new_X_coord || new_Y_coord || new_Z_coord)
	{
		_distance = 0;
		_pitch = 0;
		_yaw = 0;
		_velocity = VelocityVector();
	}
}

void Target::Update(Target &updated)
{
	this->Update(updated.X_coord, updated.Y_coord, updated.Z_coord);

	valid = updated.valid;
	seen = updated.seen;

	if (updated.X_coord || updated.Y_coord)
	{
		_distance = updated._distance;
		_pitch = updated._pitch;
		_yaw = updated._yaw;
	}
}

fixed Target::Pitch()
{
	if (!_pitch)
	{
		_pitch = atan(fixed(X_coord) / fixed(Y_coord)) * -180 / FIXEDPI;
	}
	return _pitch;
}
fixed Target::Yaw()
{
	if (!_yaw)
	{
		auto dist = Distance();
		if (!dist) {
			return 0;
		}
		_yaw = atan(Z_coord / Distance()) * 180 / FIXEDPI; // Height of default target - height of turret = angle to aim at (table height is 1320)
	}
	return _yaw;
}
fixed Target::Distance()
{
	if (!_distance)
	{
		if (!Y_coord) {
			return 1;
		}
		_distance = sqrt(pow(X_coord, 2) + pow(Y_coord, 2));
	}
	return _distance;
}

const VelocityVector Target::Velocity() const
{
	auto span = fixed(seen - last_seen);
	if (span == 0)
	{
		return VelocityVector(0, 0, 0);
	}
	Target lastState(last_X_coord, last_Y_coord, last_Z_coord);
	Target vel = (*this - lastState) / span;
	return VelocityVector(vel.X_coord, vel.Y_coord, vel.Z_coord);
}

VelocityVector Target::Velocity()
{
	if (!_velocity)
	{
		auto span = fixed(seen - last_seen);
		if (span == 0)
		{
			return VelocityVector(0, 0, 0);
		}
		Target lastState(last_X_coord, last_Y_coord, last_Z_coord);
		Target vel = (*this - lastState) / span;
		_velocity = VelocityVector(vel.X_coord, vel.Y_coord, vel.Z_coord);
	}
	return _velocity;
}

int64_t Target::timeSinceLastAction()
{
	auto now = esp_timer_get_time();
	return now - last_action;
	// return 0;
}

bool Target::actionIdleExceeds(int64_t limit)
{
	return timeSinceLastAction() >= limit;
}

void Target::IncrementAction()
{
	last_action = esp_timer_get_time();
}
