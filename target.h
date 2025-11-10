#pragma once

#ifdef ARDUINO
	#include <Arduino.h>
#endif

#include <chrono>
#include <cstdint>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"
#include "spatial.h"
#include "utilities.h"
#include "vector.hpp"

using fixed = fixed_16_16;

class Target;

/**
 * @brief Represents a target in the system.
 *
 * This class stores all relevant information about a target, including its
 * position, velocity, and timing information. It also provides methods for
 * predicting future positions and calculating intercept points.
 */
class Target {
public:
	// -- Constructors --
	Target() = default;
	Target(const Target& other) = default;
	Target(PositionVector P, VelocityVector V = VelocityVector(0, 0, 0));
	Target(
		uint8_t        index,
		bool           valid = false,
		PositionVector P = PositionVector(0, 0, 0),
		VelocityVector V = VelocityVector(0, 0, 0)
	);

	// -- Public Methods --
	void                 Update(PositionVector P);
	fixed                Pitch();
	fixed                Yaw();
	fixed                Distance();
	const VelocityVector Velocity() const;
	const PositionVector Position() const;
	TimeInterval         timeSinceLastAction() const;
	TimeInterval         timeSinceLastSeen() const;
	bool                 actionIdleExceeds(const ChronoDuration auto limit) const;
	bool                 idleExceeds(const ChronoDuration auto limit) const;
	void                 IncrementAction();
	PositionVector       PredictedPositionAtTime(ChronoDuration auto interval);
	const PositionVector interceptPosition() const;

	// -- Public Attributes --
	bool      valid = false;
	uint8_t   index;
	uint8_t   id;
	TimePoint seen;
	TimePoint last_action;

private:
	// -- Private Attributes --
	TimePoint      last_seen;
	PositionVector position;
	PositionVector last_position;
	VelocityVector velocity;
};

// ======================================================================================
// --- Inline-Defined Methods ---
// ======================================================================================

// -- Target --

/**
 * @brief Constructs a Target with a given position and velocity.
 * @param P The position vector.
 * @param V The velocity vector.
 */
inline Target::Target(PositionVector P, VelocityVector V): position(P), velocity(V) {}

/**
 * @brief Constructs a Target with an index, validity, position, and velocity.
 * @param index The index of the target.
 * @param valid The validity of the target.
 * @param P The position vector.
 * @param V The velocity vector.
 */
inline Target::Target(uint8_t index, bool valid, PositionVector P, VelocityVector V):
	valid(valid), index(index), position(P), velocity(V) {}

/**
 * @brief Predicts the target's position at a future time.
 * @param interval The time interval for the prediction.
 * @return The predicted position vector.
 */
inline PositionVector Target::PredictedPositionAtTime(ChronoDuration auto interval) {
	return PositionVector(position, velocity, interval);
}

inline bool Target::actionIdleExceeds(const ChronoDuration auto limit) const {
	return timeSinceLastAction() > limit;
}

inline bool Target::idleExceeds(const ChronoDuration auto limit) const {
	return timeSinceLastSeen() > limit;
}

/**
 * @brief Gets the velocity of the target.
 * @return The velocity vector.
 */
inline const VelocityVector Target::Velocity() const {
	return velocity;
}

/**
 * @brief Gets the position of the target.
 * @return The position vector.
 */
inline const PositionVector Target::Position() const {
	return position;
}

/**
 * @brief Calculates the time elapsed since the last action was performed on this target.
 * @return The time interval since the last action.
 */
inline TimeInterval Target::timeSinceLastAction() const {
	return TimeInterval(Clock::now() - last_action);
}

/**
 * @brief Calculates the time elapsed since the target was last seen.
 * @return The time interval since the last sighting.
 */
inline TimeInterval Target::timeSinceLastSeen() const {
	return TimeInterval(Clock::now() - seen);
}

/**
 * @brief Increments the action counter by updating the last action timestamp.
 */
inline void Target::IncrementAction() {
	last_action = Clock::now();
}
