#include <climits>
#include <stdint.h>
// #include <Arduino.h>
// #include "esp_timer.h"
#include "DptHelpers.h"

#include "target.h"
#include <math.h>
#include "fpm_adapter.hpp"
#include "aproximate_math.hpp"

DistanceVector::DistanceVector(VelocityVector, ChronoDuration auto interval)
{
}

PositionVector::PositionVector(PositionVector, DistanceVector)
{
}

PositionVector::PositionVector(PositionVector, VelocityVector, ChronoDuration auto interval)
{
}

fixed PositionVector::Pitch()
{
	if (!_pitch)
	{
		_pitch = pitch();
	}
	return _pitch;
}

fixed PositionVector::Yaw()
{
	if (!_yaw)
	{
		_yaw = yaw(); // Height of default target - height of turret = angle to aim at (table height is 1320)
	}
	return _yaw;
}

fixed PositionVector::Distance()
{
	if (!_distance)
	{
		_distance = magnitudeXY();
	}
	return _distance;
}

VelocityVector::VelocityVector(DistanceVector dist, TimeInterval interval)
{
	// TimeInterval duration = interval.as<fixed, std::ratio<1>();
	if (interval.count())
	{
		X_coord = dist.X_coord / interval.count();
		Y_coord = dist.Y_coord / interval.count();
		Z_coord = dist.Z_coord / interval.count();
	}
}

// VelocityVector::VelocityVector(PositionVector P2, PositionVector P1, ChronoDuration auto interval)
// {
// 	auto duration = std::chrono::duration_cast<std::chrono::seconds>(interval).count();
// 	if (duration)
// 	{
// 		X_coord = (P2.X_coord - P1.X_coord) / duration;
// 		Y_coord = (P2.Y_coord - P1.Y_coord) / duration;
// 		Z_coord = (P2.Z_coord - P1.Z_coord) / duration;
// 	}
// }

// DistanceVector VelocityVector::operator*(const ChronoDuration auto &interval)
// {
// 	auto scale = AsSeconds(interval).count();
// 	return DistanceVector(X_coord, Y_coord, Z_coord) * scale;
// }

Target::Target(PositionVector P, VelocityVector V) : position(P), velocity(V)
{
}

Target::Target(uint8_t index, bool valid, PositionVector P, VelocityVector V) : index(index), valid(valid), position(P), velocity(V)
{
}

void Target::Update(PositionVector P)
{
	last_seen = seen;
	seen = Clock::now();
	velocity = VelocityVector();
	last_position = position;
	position = P;
}

PositionVector Target::PredictedPositionAtTime(ChronoDuration auto interval)
{
	return PositionVector();
}

const VelocityVector Target::Velocity() const
{
	// if (velocity)
	// {
		return velocity;
	// }

	// return VelocityVector(position - last_position, TimeInterval(seen - last_seen));
}

const PositionVector Target::Position() const
{
	return position;
}

TimeInterval Target::timeSinceLastAction() const
{
	return TimeInterval(Clock::now() - last_action);
}

TimeInterval Target::timeSinceLastSeen() const
{
	return TimeInterval(Clock::now() - seen);
}

void Target::IncrementAction()
{
	last_action = Clock::now();
}

/****
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
 */