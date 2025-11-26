#pragma once

#ifdef ARDUINO
	#include <Arduino.h>
#endif

#include <cstdint>
#include <memory>

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
	void Update(PositionVector P);

	// -- Getters --
	const PositionVector Position() const;
	const VelocityVector Velocity() const;
	fixed                Distance();
	fixed                Pitch();
	fixed                Yaw();

	// -- Time & State Queries --
	TimeInterval timeSinceLastAction() const;
	TimeInterval timeSinceLastSeen() const;
	bool         actionable() const;

	// -- Time-based Checks (use operators: clock_now() - last_action > milliseconds(100)) --
	bool idleExceeds(const ChronoDuration auto limit) const { return timeSinceLastSeen() > limit; }

	bool actionIdleExceeds(const ChronoDuration auto limit) const { return timeSinceLastAction() > limit; }

	// -- Setters & Mutations --
	void                 IncrementAction();
	PositionVector       PredictedPositionAtTime(ChronoDuration auto interval);
	const PositionVector InterceptAimpoint();

	// -- Public Attributes --
	bool      valid = false;
	uint8_t   index = 0;
	uint32_t  id = 0;
	TimePoint seen;
	TimePoint last_action;

private:
	std::shared_ptr<const PositionVector> interceptPosition() const;

	std::shared_ptr<const PositionVector> last_aimpoint = nullptr;
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

inline const PositionVector Target::InterceptAimpoint() {
	if (last_aimpoint == nullptr) {
		last_aimpoint = interceptPosition();
	}
	return *last_aimpoint;
}

/**
 * @brief Constructs a Target with a given position and velocity.
 * @param P The position vector.
 * @param V The velocity vector.
 */
inline Target::Target(PositionVector P, VelocityVector V): position(P), velocity(V) {}

/**
 * @brief Constructs a Target with an index, validity, position, and velocity.
 * @param idx The index of the target.
 * @param is_valid The validity of the target.
 * @param P The position vector.
 * @param V The velocity vector.
 */
inline Target::Target(uint8_t idx, bool is_valid, PositionVector P, VelocityVector V):
	valid(is_valid), index(idx), position(P), velocity(V) {}

// -- Getters --

/**
 * @brief Gets the position of the target.
 * @return The position vector.
 */
inline const PositionVector Target::Position() const {
	return position;
}

/**
 * @brief Gets the velocity of the target.
 * @return The velocity vector.
 */
inline const VelocityVector Target::Velocity() const {
	return velocity;
}

inline fixed Target::Distance() {
	return position.Distance();
}

// -- Time & State Queries --

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

inline bool Target::actionable() const {
	return valid && actionIdleExceeds(fireActionInterval);
}

// -- Setters & Mutations --

/**
 * @brief Increments the action counter by updating the last action timestamp.
 */
inline void Target::IncrementAction() {
	last_action = Clock::now();
}

/**
 * @brief Predicts the target's position at a future time.
 * @param interval The time interval for the prediction.
 * @return The predicted position vector.
 */
inline PositionVector Target::PredictedPositionAtTime(ChronoDuration auto interval) {
	return PositionVector(position, velocity, interval);
}