/**
 * @file target.cpp
 * @brief Implements the Target and vector classes.
 *
 * This file contains the method definitions for the Target, PositionVector,
 * VelocityVector, and DistanceVector classes.
 */
#include "target.h"

#include <climits>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"
#include <math.h>
#include <stdint.h>

/**
 * @brief Constructs a DistanceVector from a VelocityVector and a time interval.
 *
 * This constructor calculates the displacement by multiplying velocity by time.
 *
 * @param v The velocity vector.
 * @param interval The time interval.
 */
DistanceVector::DistanceVector(VelocityVector v, ChronoDuration auto interval) {
	*this = v * interval;
}

/**
 * @brief Constructs a PositionVector by adding a DistanceVector to another PositionVector.
 *
 * This represents a translation of a point in space.
 *
 * @param p The initial position vector.
 * @param d The distance vector to add.
 */
PositionVector::PositionVector(PositionVector p, DistanceVector d) {
	*this = p + d;
}

/**
 * @brief Constructs a PositionVector by applying a VelocityVector over a time interval to a starting PositionVector.
 *
 * This calculates a future position based on a starting point, constant velocity, and duration.
 *
 * @param p The initial position vector.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
PositionVector::PositionVector(PositionVector p, VelocityVector v, ChronoDuration auto interval) {
	*this = p + v * interval;
}

/**
 * @brief Calculates the pitch of the vector, caching the result for efficiency.
 *
 * If the pitch has not been calculated before, it computes it and stores it.
 * Subsequent calls will return the cached value.
 *
 * @return The pitch in degrees.
 */
fixed PositionVector::Pitch() {
	if (!_pitch) {
		_pitch = pitch();
	}
	return _pitch;
}

/**
 * @brief Calculates the yaw of the vector, caching the result for efficiency.
 *
 * If the yaw has not been calculated before, it computes it and stores it.
 * Subsequent calls will return the cached value.
 *
 * @return The yaw in degrees.
 */
fixed PositionVector::Yaw() {
	if (!_yaw) {
		_yaw = yaw();
	}
	return _yaw;
}

/**
 * @brief Calculates the 2D magnitude (distance in the XY plane) of the vector, caching the result.
 *
 * If the distance has not been calculated before, it computes it and stores it.
 * Subsequent calls will return the cached value.
 *
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
 *
 * This constructor calculates velocity by dividing distance by time.
 * It handles the case where the time interval is zero to avoid division by zero.
 *
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
 * @brief Constructs a Target with an index, validity, position, and velocity.
 * @param index The index of the target (e.g., in an array).
 * @param valid The initial validity of the target data.
 * @param P The initial position vector.
 * @param V The initial velocity vector.
 */
Target::Target(uint8_t index, bool valid, PositionVector P, VelocityVector V) :
	valid(valid), index(index), position(P), velocity(V) {}

/**
 * @brief Updates the target's state with a new position measurement.
 *
 * This method recalculates the target's velocity based on the change in position
 * and the time elapsed since the last update. It also updates the `seen` timestamp
 * and shifts the position history (`position` becomes `last_position`).
 *
 * @param P The new position vector measurement.
 */
void Target::Update(PositionVector P) {
	TimePoint    new_seen = Clock::now();
	TimeInterval time_delta = TimeInterval(new_seen - seen);

	if (time_delta.count() > 0) {
		// Calculate velocity from the displacement and time delta.
		velocity = (P - position) / time_delta;
	}
	// If no time has passed, velocity remains unchanged.

	// Update the target's history.
	last_position = position;
	position = P;
	last_seen = seen;
	seen = new_seen;
}

/**
 * @brief Predicts the target's position at a future time, assuming constant velocity.
 * @param interval The time interval from now for the prediction.
 * @return The predicted `PositionVector`.
 */
PositionVector Target::PredictedPositionAtTime(ChronoDuration auto interval) {
	return PositionVector(position, velocity, interval);
}

/**
 * @brief Gets the current velocity of the target.
 * @return A const reference to the target's velocity vector.
 */
const VelocityVector Target::Velocity() const {
	return velocity;
}

/**
 * @brief Gets the current position of the target.
 * @return A const reference to the target's position vector.
 */
const PositionVector Target::Position() const {
	return position;
}

/**
 * @brief Calculates the time elapsed since the last action was performed on this target.
 * An "action" could be, for example, the system deciding to aim at this target.
 * @return The `TimeInterval` since the last action.
 */
TimeInterval Target::timeSinceLastAction() const {
	return TimeInterval(Clock::now() - last_action);
}

/**
 * @brief Calculates the time elapsed since the target's position was last updated.
 * @return The `TimeInterval` since the last sighting.
 */
TimeInterval Target::timeSinceLastSeen() const {
	return TimeInterval(Clock::now() - seen);
}

/**
 * @brief Updates the `last_action` timestamp to the current time.
 * This is used to mark that the system has interacted with or processed this target.
 */
void Target::IncrementAction() {
	last_action = Clock::now();
}
