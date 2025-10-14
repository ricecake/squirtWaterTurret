#include "target.h"

#include "aproximate_math.hpp"

#include "fpm_adapter.hpp"

#include <climits>
#include <math.h>
#include <stdint.h>

/**
 * @brief Constructs a DistanceVector from a VelocityVector and a time interval.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
DistanceVector::DistanceVector(VelocityVector v, ChronoDuration auto interval) {
	*this = v * interval;
}

/**
 * @brief Constructs a PositionVector by adding a DistanceVector to a PositionVector.
 * @param p The initial position vector.
 * @param d The distance vector to add.
 */
PositionVector::PositionVector(PositionVector p, DistanceVector d) {
	*this = p + d;
}

/**
 * @brief Constructs a PositionVector by applying a VelocityVector over a time interval to a PositionVector.
 * @param p The initial position vector.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
PositionVector::PositionVector(PositionVector p, VelocityVector v, ChronoDuration auto interval) {
	*this = p + v * interval;
}

/**
 * @brief Calculates the pitch of the vector, caching the result.
 * @return The pitch in radians.
 */
fixed PositionVector::Pitch() {
	if (!_pitch) {
		_pitch = pitch();
	}
	return _pitch;
}

/**
 * @brief Calculates the yaw of the vector, caching the result.
 * @return The yaw in radians.
 */
fixed PositionVector::Yaw() {
	if (!_yaw) {
		_yaw = yaw();
	}
	return _yaw;
}

/**
 * @brief Calculates the 2D magnitude (distance) of the vector, caching the result.
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
 * @param P The position vector.
 * @param V The velocity vector.
 */
Target::Target(PositionVector P, VelocityVector V) :
	position(P),
	velocity(V) {
}

/**
 * @brief Constructs a Target with an index, validity, position, and velocity.
 * @param index The index of the target.
 * @param valid The validity of the target.
 * @param P The position vector.
 * @param V The velocity vector.
 */
Target::Target(uint8_t index, bool valid, PositionVector P, VelocityVector V) :
	valid(valid),
	index(index),
	position(P),
	velocity(V) {
}

/**
 * @brief Updates the target's state with a new position measurement.
 *
 * This method recalculates the target's velocity based on the change in position
 * and time since the last update. It also updates the `seen` and `position` history.
 *
 * @param P The new position vector.
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
 * @brief Predicts the target's position at a future time.
 * @param interval The time interval for the prediction.
 * @return The predicted position vector.
 */
PositionVector Target::PredictedPositionAtTime(ChronoDuration auto interval) {
	return PositionVector(position, velocity, interval);
}

/**
 * @brief Gets the velocity of the target.
 * @return The velocity vector.
 */
const VelocityVector Target::Velocity() const {
	return velocity;
}

/**
 * @brief Gets the position of the target.
 * @return The position vector.
 */
const PositionVector Target::Position() const {
	return position;
}

/**
 * @brief Calculates the time elapsed since the last action was performed on this target.
 * @return The time interval since the last action.
 */
TimeInterval Target::timeSinceLastAction() const {
	return TimeInterval(Clock::now() - last_action);
}

/**
 * @brief Calculates the time elapsed since the target was last seen.
 * @return The time interval since the last sighting.
 */
TimeInterval Target::timeSinceLastSeen() const {
	return TimeInterval(Clock::now() - seen);
}

/**
 * @brief Increments the action counter by updating the last action timestamp.
 */
void Target::IncrementAction() {
	last_action = Clock::now();
}
