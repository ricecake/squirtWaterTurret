#include <climits>
#include <stdint.h>
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
	if (interval.count())
	{
		X_coord = dist.X_coord / interval.count();
		Y_coord = dist.Y_coord / interval.count();
		Z_coord = dist.Z_coord / interval.count();
	}
}

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
	return velocity;
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
