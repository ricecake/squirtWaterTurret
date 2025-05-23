#include <climits>
#include <stdint.h>
#include <Arduino.h>
#include "esp_timer.h"
#include "DptHelpers.h"

#include "target.h"
#include <math.h>

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
	last_velocity = this->velocity;

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
		distance = 0;
		pitch = 0;
		yaw = 0;
	}
}

void Target::Update(Target &updated)
{

  this->Update(updated.X_coord, updated.Y_coord, updated.Z_coord);

	valid = updated.valid;
	seen = updated.seen;

	if (updated.X_coord || updated.Y_coord)
	{
		distance = updated.distance;
		pitch = updated.pitch;
		yaw = updated.yaw;
	}
}

double Target::Pitch()
{
	if (!pitch)
	{
		pitch = atan(double(X_coord) / double(Y_coord)) * -180.0 / PI;
	}
	return pitch;
}
double Target::Yaw()
{
	if (!yaw)
	{
		yaw = atan(double(Z_coord) / double(Distance())) * 180.0 / PI; // Height of default target - height of turret = angle to aim at (table height is 1320)
	}
	return yaw;
}
long Target::Distance()
{
	if (!distance)
	{
		distance = sqrt(pow(X_coord, 2) + pow(Y_coord, 2));
	}
	return distance;
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
