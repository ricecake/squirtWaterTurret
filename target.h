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

inline const PositionVector Target::interceptPosition() const {
	const PositionVector       proj_pos = PositionVector(0, 0, 1.5);
	const PositionVector       target_pos = position;
	const VelocityVector       target_velocity = velocity;
	const fixed_24_8           proj_speed = 20;
	const Vector3D<fixed_24_8> Gv(0, 0, 9.814);
	const fixed_24_8           G = Gv.magnitude();

	const fixed_24_8 P = target_velocity.X_coord;
	const fixed_24_8 Q = target_velocity.Z_coord;
	const fixed_24_8 R = target_velocity.Y_coord;

	const auto       diff = target_pos - proj_pos;
	const fixed_24_8 H = diff.X_coord;
	const fixed_24_8 J = diff.Z_coord;
	const fixed_24_8 K = diff.Y_coord;

	const fixed_24_8 L = fixed_24_8(-0.5) * G;
	const fixed_24_8 S = proj_speed;

	// Quartic Coefficients
	const fixed_24_8 c0 = L * L;
	const fixed_24_8 c1 = -2 * Q * L;
	const fixed_24_8 c2 = -2 * J * L + fixed_24_8(target_velocity.dot(target_velocity)) - pow(S, 2);
	const fixed_24_8 c3 = 2 * (diff.dot(target_velocity));
	const fixed_24_8 c4 = diff.dot(diff);

	const std::function<fixed_24_8(const fixed_24_8)> movingTargetInterceptQuartic =
		[=](const fixed_24_8 t) -> fixed_24_8 {
		return c0 * pow(t, 4) + c1 * pow(t, 3) + c2 * pow(t, 2) + c3 * t + c4;
	};

	const auto [converged, intercept] = Approximate::small_root(movingTargetInterceptQuartic);

	if (intercept == 0) {
		return PositionVector(H, K, J);
	}

	auto pos = diff + target_velocity * intercept;
	pos.Z_coord = fixed_24_8(pos.Z_coord) - L * pow(intercept, 2);

	return PositionVector(
		(H + P * intercept) / intercept,
		(K + R * intercept) / intercept,
		(J + Q * intercept - L * pow(intercept, 2)) / intercept
	);
};
