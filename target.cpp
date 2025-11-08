#include "target.h"

#include <climits>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"
#include <math.h>
#include <stdint.h>

/**
 * @brief Constructs a Target with a given position and velocity.
 * @param P The position vector.
 * @param V The velocity vector.
 */
Target::Target(PositionVector P, VelocityVector V): position(P), velocity(V) {}

/**
 * @brief Constructs a Target with an index, validity, position, and velocity.
 * @param index The index of the target.
 * @param valid The validity of the target.
 * @param P The position vector.
 * @param V The velocity vector.
 */
Target::Target(uint8_t index, bool valid, PositionVector P, VelocityVector V):
	valid(valid), index(index), position(P), velocity(V) {}

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
