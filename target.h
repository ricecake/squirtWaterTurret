#pragma once

#ifdef ARDUINO
	#include <Arduino.h>
#endif

#include <chrono>
#include <cstdint>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"
#include "utilities.h"
#include "vector.hpp"

using fixed = fixed_16_16;

class Target;
class PositionVector;
class DistanceVector;
class VelocityVector;

/**
 * @brief Represents a 3D distance vector.
 *
 * This class is used to define a displacement in 3D space.
 */
class DistanceVector: public Vector3D<fixed, DistanceVector> {
public:
	// -- Type Definitions --
	using Vec = Vector3D<fixed, DistanceVector>;

	// -- Constructors --
	DistanceVector() = default;
	DistanceVector(const DistanceVector& other) = default;
	constexpr DistanceVector(fixed x, fixed y, fixed z);
	DistanceVector(VelocityVector v, ChronoDuration auto interval);
};

/**
 * @brief Represents a 3D position vector.
 *
 * This class defines a specific point in 3D space and provides methods
 * to calculate pitch, yaw, and distance.
 */
class PositionVector: public Vector3D<fixed, PositionVector> {
public:
	// -- Type Definitions --
	using Vec = Vector3D<fixed, PositionVector>;

	// -- Constructors --
	PositionVector() = default;
	PositionVector(const PositionVector& other) = default;
	constexpr PositionVector(fixed x, fixed y, fixed z);
	PositionVector(PositionVector, DistanceVector);
	PositionVector(PositionVector p, VelocityVector v, ChronoDuration auto interval);

	// -- Public Methods --
	fixed Pitch();
	fixed Yaw();
	fixed Distance();

private:
	// -- Private Attributes --
	fixed _distance = 0; ///< Cached distance value.
	fixed _pitch = 0;    ///< Cached pitch value.
	fixed _yaw = 0;      ///< Cached yaw value.
};

/**
 * @brief Represents a 3D velocity vector.
 *
 * This class is used to define the rate of change of position.
 */
class VelocityVector: public Vector3D<fixed, VelocityVector> {
public:
	// -- Type Definitions --
	using Vec = Vector3D<fixed, VelocityVector>;

	// -- Constructors --
	VelocityVector() = default;
	VelocityVector(const VelocityVector& other) = default;
	constexpr VelocityVector(fixed x, fixed y, fixed z);
	VelocityVector(DistanceVector, TimeInterval interval);
};

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

// -- DistanceVector --
constexpr DistanceVector::DistanceVector(fixed x, fixed y, fixed z): Vec(x, y, z) {}

/**
 * @brief Constructs a DistanceVector from a VelocityVector and a time interval.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
inline DistanceVector::DistanceVector(VelocityVector v, ChronoDuration auto interval) {
	*this = v * interval;
}

// -- PositionVector --
constexpr PositionVector::PositionVector(fixed x, fixed y, fixed z): Vec(x, y, z) {}

/**
 * @brief Constructs a PositionVector by applying a VelocityVector over a time interval to a PositionVector.
 * @param p The initial position vector.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
inline PositionVector::PositionVector(PositionVector p, VelocityVector v, ChronoDuration auto interval) {
	*this = p + v * interval;
}

// -- VelocityVector --
constexpr VelocityVector::VelocityVector(fixed x, fixed y, fixed z): Vec(x, y, z) {}

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

// ======================================================================================
// --- Operator Overloads ---
// ======================================================================================

constexpr const VelocityVector operator/(const DistanceVector& D, const ChronoDuration auto& interval) {
	auto scale = interval.count();
	return VelocityVector(D.X_coord / scale, D.Y_coord / scale, D.Z_coord / scale);
}

constexpr const DistanceVector operator*(const VelocityVector& V, const ChronoDuration auto& interval) {
	auto scale = static_cast<fixed>(interval.count());
	return DistanceVector(V.X_coord, V.Y_coord, V.Z_coord) * scale;
}

constexpr const PositionVector operator+(const PositionVector& A, const DistanceVector& B) {
	return PositionVector(A.X_coord + B.X_coord, A.Y_coord + B.Y_coord, A.Z_coord + B.Z_coord);
}

constexpr const DistanceVector operator-(const PositionVector& A, const PositionVector& B) {
	return DistanceVector(A.X_coord - B.X_coord, A.Y_coord - B.Y_coord, A.Z_coord - B.Z_coord);
}
