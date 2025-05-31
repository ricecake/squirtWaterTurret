#include <climits>
#include <stdint.h>
#include <Arduino.h>
#include "esp_timer.h"
#include "DptHelpers.h"

#include "target.h"
#include <math.h>
#include "fpm/fixed.hpp"
#include "aproximate_math.hpp"

Target::Target() : index(0), X_coord(0), Y_coord(0), Z_coord(0), speed(0), valid(false)
{
	seen = esp_timer_get_time();
}

Target::Target(uint8_t index, long X, long Y, long speed, bool valid) : index(index), X_coord(X), Y_coord(Y), speed(speed), valid(valid)
{
	seen = esp_timer_get_time();
}

Target::Target(uint8_t index, long X, long Y, long Z, long speed, bool valid) : index(index), X_coord(X), Y_coord(Y), Z_coord(Z), speed(speed), valid(valid)
{
	seen = esp_timer_get_time();
}

void Target::Update(long new_X_coord, long new_Y_coord, long new_Z_coord) {
	last_X_coord  = this->X_coord;
	last_Y_coord  = this->Y_coord;
	last_Z_coord  = this->Z_coord;
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

	if (new_X_coord || new_Y_coord)
	{
		_distance = 0;
		_pitch = static_cast<fixed>(0);
		_yaw = static_cast<fixed>(0);
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
		_yaw = atan(fixed(Z_coord) / fixed(Distance())) * 180 / FIXEDPI; // Height of default target - height of turret = angle to aim at (table height is 1320)
	}
	return _yaw;
}
long Target::Distance()
{
	if (!_distance)
	{
		_distance = sqrt(pow(X_coord, 2) + pow(Y_coord, 2));
	}
	return _distance;
}

int64_t Target::timeSinceLastAction()
{
	auto now = esp_timer_get_time();
	return now - last_action;
}

bool Target::actionIdleExceeds(int64_t limit)
{
	return timeSinceLastAction() >= limit;
}

void Target::IncrementAction()
{
	last_action = esp_timer_get_time();
}
