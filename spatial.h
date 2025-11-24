#pragma once

#include <chrono>
#include <cstdint>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"
#include "logger.h"
#include "utilities.h"
#include "vector.hpp"

using fixed = fixed_16_16;

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

	static fixed integer_sqrt(uint64_t n) {
		if (n < 2)
			return (uint32_t)n;

		uint64_t root = 0;
		uint64_t bit = 1ULL << 62; // The "test bit"

		// Optimization: Skip leading zeros to find the start range
		while (bit > n) {
			bit >>= 2;
		}

		while (bit != 0) {
			if (n >= root + bit) {
				n -= (root + bit);
				// Equivalent to: root += 2*bit; root >>= 1;
				root = (root >> 1) + bit;
			} else {
				root >>= 1;
			}
			bit >>= 2;
		}
		return fixed::from_raw_value((uint32_t)root);
	}

	fixed angleTo(const VectorCompatible<fixed> auto& other) const {
		if (!(*this && other)) {
			return NumericType(0);
		}
		// 1. Raw Coordinates (Scale: 1Q)
		int64_t aX = X_coord.raw_value();
		int64_t aY = Y_coord.raw_value();
		int64_t aZ = Z_coord.raw_value();
		int64_t bX = other.X_coord.raw_value();
		int64_t bY = other.Y_coord.raw_value();
		int64_t bZ = other.Z_coord.raw_value();

		// 2. Dot Product (Scale: 2Q)
		int64_t dot_raw = aX * bX + aY * bY + aZ * bZ;

		// 3. Cross Product Components (Scale: 2Q)
		int64_t cX_2Q = aY * bZ - aZ * bY;
		int64_t cY_2Q = aZ * bX - aX * bZ;
		int64_t cZ_2Q = aX * bY - aY * bX;

		// 4. SAFE MAGNITUDE CALCULATION
		// We MUST shift down to 1Q before squaring to avoid int64 overflow.
		// (Squaring a 2Q number results in 4Q, which overflows int64 if value > 0.7)
		int64_t cX_1Q = cX_2Q >> fixed::FixedBits;
		int64_t cY_1Q = cY_2Q >> fixed::FixedBits;
		int64_t cZ_1Q = cZ_2Q >> fixed::FixedBits;

		// Sum of Squares (Scale: 2Q)
		// 1Q * 1Q = 2Q. Summing three 2Q numbers fits safely in int64 (up to ~46,000 vector magnitude).
		uint64_t cross_sq_2Q = (uint64_t)(cX_1Q * cX_1Q + cY_1Q * cY_1Q + cZ_1Q * cZ_1Q);

		// 5. Square Root
		// Input is 2Q, so Output is 1Q.
		uint32_t cross_mag_raw = integer_sqrt(cross_sq_2Q); // Why is sqrt(160369865) 0? 542170?

		// 6. Final Angle
		// Ensure dot product is also 1Q
		fixed dot_product_1Q = fixed::from_raw_value(dot_raw >> fixed::FixedBits);
		fixed cross_magnitude_1Q = fixed::from_raw_value(cross_mag_raw);

		return atan2(cross_magnitude_1Q, dot_product_1Q) * rad2DegFactor;
	}

	fixed dot(const VectorCompatible<fixed> auto& other) const {
		if (!(*this && other)) {
			return NumericType(0);
		}

		int64_t aX = X_coord.raw_value();
		int64_t aY = Y_coord.raw_value();
		int64_t aZ = Z_coord.raw_value();
		int64_t bX = other.X_coord.raw_value();
		int64_t bY = other.Y_coord.raw_value();
		int64_t bZ = other.Z_coord.raw_value();
		int64_t dot_product_raw = (int64_t)aX * bX + (int64_t)aY * bY + (int64_t)aZ * bZ;
		return fixed::from_raw_value((dot_product_raw >> fixed::FixedBits));
	}

	PositionVector cross(const VectorCompatible<fixed> auto& other) const {
		int64_t aX = X_coord.raw_value();
		int64_t aY = Y_coord.raw_value();
		int64_t aZ = Z_coord.raw_value();
		int64_t bX = other.X_coord.raw_value();
		int64_t bY = other.Y_coord.raw_value();
		int64_t bZ = other.Z_coord.raw_value();

		int64_t cX_raw = (int64_t)aY * bZ - (int64_t)aZ * bY;
		int64_t cY_raw = (int64_t)aZ * bX - (int64_t)aX * bZ;
		int64_t cZ_raw = (int64_t)aX * bY - (int64_t)aY * bX;

		return PositionVector(
			fixed::from_raw_value(cX_raw),
			fixed::from_raw_value(cY_raw),
			fixed::from_raw_value(cZ_raw)
		);
	}

	fixed magnitude() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aX = X_coord.raw_value();
		int64_t aY = Y_coord.raw_value();
		int64_t aZ = Z_coord.raw_value();

		int64_t dot_product_raw = aX * aX + aY * aY + aZ * aZ;
		return integer_sqrt(dot_product_raw);
	}

	fixed magnitudeXY() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aX = X_coord.raw_value();
		int64_t aY = Y_coord.raw_value();

		int64_t dot_product_raw = aX * aX + aY * aY;
		return integer_sqrt(dot_product_raw);
	}

	fixed magnitudeXZ() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aX = X_coord.raw_value();
		int64_t aZ = Z_coord.raw_value();

		int64_t dot_product_raw = aX * aX + aZ * aZ;
		return integer_sqrt(dot_product_raw);
	}

	fixed magnitudeYZ() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aY = Y_coord.raw_value();
		int64_t aZ = Z_coord.raw_value();

		int64_t dot_product_raw = aY * aY + aZ * aZ;
		return integer_sqrt(dot_product_raw);
	}

	PositionVector normalize() const {
		fixed mag = magnitude();
		if (mag != 0) {
			return PositionVector(X_coord / mag, Y_coord / mag, Z_coord / mag);
		}
		return PositionVector(X_coord, Y_coord, Z_coord);
	}

	NumericType pitch() const {
		auto mag = magnitudeXY();
		if (!(Z_coord || mag)) {
			return 0;
		}
		return atan2(Z_coord, magnitudeXY()) * rad2DegFactor;
	}

	NumericType yaw() const {
		if (!(X_coord || Y_coord)) {
			return 0;
		}
		return atan2(X_coord, Y_coord) * rad2DegFactor;
	}

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
