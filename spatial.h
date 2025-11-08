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

class PositionVector;
class DistanceVector;
class VelocityVector;

// Helper to check for approximate equality of fixed-point numbers
inline bool is_close(fixed a, fixed b, fixed tolerance = fixed(0.001)) {
	return fpm::abs(a - b) < tolerance;
}

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
 * @brief Constructs a PositionVector by adding a DistanceVector to a PositionVector.
 * @param p The initial position vector.
 * @param d The distance vector to add.
 */
inline PositionVector::PositionVector(PositionVector p, DistanceVector d) {
	*this = p + d;
}

/**
 * @brief Constructs a PositionVector by applying a VelocityVector over a time interval to a PositionVector.
 * @param p The initial position vector.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
inline PositionVector::PositionVector(PositionVector p, VelocityVector v, ChronoDuration auto interval) {
	*this = p + v * interval;
}

/**
 * @brief Calculates the pitch of the vector, caching the result.
 * @return The pitch in radians.
 */
inline fixed PositionVector::Pitch() {
	if (!_pitch) {
		_pitch = pitch();
	}
	return _pitch;
}

/**
 * @brief Calculates the yaw of the vector, caching the result.
 * @return The yaw in radians.
 */
inline fixed PositionVector::Yaw() {
	if (!_yaw) {
		_yaw = yaw();
	}
	return _yaw;
}

/**
 * @brief Calculates the 2D magnitude (distance) of the vector, caching the result.
 * @return The distance in the XY plane.
 */
inline fixed PositionVector::Distance() {
	if (!_distance) {
		_distance = magnitudeXY();
	}
	return _distance;
}

// -- VelocityVector --
constexpr VelocityVector::VelocityVector(fixed x, fixed y, fixed z): Vec(x, y, z) {}

/**
 * @brief Constructs a VelocityVector from a DistanceVector and a time interval.
 * @param dist The distance vector.
 * @param interval The time interval.
 */
inline VelocityVector::VelocityVector(DistanceVector dist, TimeInterval interval) {
	if (interval.count()) {
		X_coord = dist.X_coord / interval.count();
		Y_coord = dist.Y_coord / interval.count();
		Z_coord = dist.Z_coord / interval.count();
	}
}

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
