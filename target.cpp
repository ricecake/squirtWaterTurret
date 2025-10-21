/**
 * @file target.cpp
 * @brief Implements the classes for representing targets and 3D vectors.
 */
#include "target.h"

#include <climits>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"
#include <math.h>
#include <stdint.h>

/**
 * @brief Constructs a DistanceVector from a VelocityVector and a time interval.
 * This is calculated as `distance = velocity * time`.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
DistanceVector::DistanceVector(VelocityVector v, ChronoDuration auto interval) {
	*this = v * interval;
}

/**
 * @brief Constructs a PositionVector by adding a DistanceVector to another PositionVector.
 * This represents a displacement from a starting point.
 * @param p The initial position vector.
 * @param d The distance vector to add.
 */
PositionVector::PositionVector(PositionVector p, DistanceVector d) {
	*this = p + d;
}

/**
 * @brief Constructs a PositionVector by applying a VelocityVector over a time interval to a PositionVector.
 * This projects a future position based on current position and velocity.
 * @param p The initial position vector.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
PositionVector::PositionVector(PositionVector p, VelocityVector v, ChronoDuration auto interval) {
	*this = p + v * interval;
}

/**
 * @brief Calculates the pitch of the vector, caching the result for efficiency.
 * @return The pitch angle in radians.
 */
fixed PositionVector::Pitch() {
	if (!_pitch) {
		_pitch = pitch();
	}
	return _pitch;
}

/**
 * @brief Calculates the yaw of the vector, caching the result for efficiency.
 * @return The yaw angle in radians.
 */
fixed PositionVector::Yaw() {
	if (!_yaw) {
		_yaw = yaw();
	}
	return _yaw;
}

/**
 * @brief Calculates the 2D magnitude (distance in the XY plane) of the vector, caching the result.
 * @return The distance in the XY plane.
 */
fixed PositionVector::Distance() {
	if (!_distance) {
		_distance = magnitudeXY();
	}
	return _distance;
}

/**
 * @brief Constructs a VelocityVector from a DistanceVector and a time interval.
 * This is calculated as `velocity = distance / time`.
 * @param dist The distance vector.
 * @param interval The time interval.
 */
VelocityVector::VelocityVector(DistanceVector dist, TimeInterval interval) {
	if (interval.count()) {
		X_coord = dist.X_coord / interval.count();
		Y_coord = dist.Y_coord / interval.count();
		Z_coord = dist.Z_coord / interval.count();
	}
}

/**
 * @brief Constructs a Target with a given position and velocity.
 * @param P The initial position vector.
 * @param V The initial velocity vector.
 */
Target::Target(PositionVector P, VelocityVector V) : position(P), velocity(V) {}

/**
 * @brief Constructs a Target with a full initial state.
 * @param index The index of the target in its array.
 * @param valid The validity of the target.
 * @param P The initial position vector.
 * @param V The initial velocity vector.
 */
Target::Target(uint8_t index, bool valid, PositionVector P, VelocityVector V) :
	valid(valid), index(index), position(P), velocity(V) {}

/**
 * @brief Updates the target's state with a new position measurement.
 *
 * This method recalculates the target's velocity based on the change in position
 * and time since the last update. It also updates the `seen` timestamp and the
 * `position` history, which are crucial for velocity calculation in subsequent updates.
 *
 * @param P The new position vector of the target.
 */
void Target::Update(PositionVector P) {
	TimePoint    new_seen = Clock::now();
	TimeInterval time_delta = TimeInterval(new_seen - seen);

	if (time_delta.count() > 0) {
		// Calculate velocity from the displacement and time delta.
		velocity = (P - position) / time_delta;
	}
	// If no time has passed, velocity remains unchanged.

	// Update the target's history for the next iteration.
	last_position = position;
	position = P;
	last_seen = seen;
	seen = new_seen;
}

/**
 * @brief Predicts the target's position at a future time, assuming constant velocity.
 * @param interval The time interval for the prediction.
 * @return The predicted position vector.
 */
PositionVector Target::PredictedPositionAtTime(ChronoDuration auto interval) {
	return PositionVector(position, velocity, interval);
}

/**
 * @brief Gets the current velocity of the target.
 * @return The constant velocity vector of the target.
 */
const VelocityVector Target::Velocity() const {
	return velocity;
}

/**
 * @brief Gets the current position of the target.
 * @return The constant position vector of the target.
 */
const PositionVector Target::Position() const {
	return position;
}

/**
 * @brief Calculates the time elapsed since the last action was performed on this target.
 * "Action" typically refers to the system actively engaging with the target, such as firing.
 * @return The time interval since the last action.
 */
TimeInterval Target::timeSinceLastAction() const {
	return TimeInterval(Clock::now() - last_action);
}

/**
 * @brief Calculates the time elapsed since the target was last seen (i.e., its position was updated).
 * @return The time interval since the last sighting.
 */
TimeInterval Target::timeSinceLastSeen() const {
	return TimeInterval(Clock::now() - seen);
}

/**
 * @brief Resets the action timer by updating the last action timestamp to the current time.
 * This should be called whenever the system performs an action on the target.
 */
void Target::IncrementAction() {
	last_action = Clock::now();
}
