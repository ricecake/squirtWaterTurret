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
 * @brief A 3D vector class with specialized fixed-point arithmetic.
 *
 * This class provides optimized implementations for magnitude, dot product,
 * cross product, and other vector operations using fixed-point numbers to
 * avoid floating-point arithmetic and improve performance on embedded systems.
 * It uses the Curiously Recurring Template Pattern (CRTP) to allow derived
 * classes to customize behavior while reusing the base implementation.
 *
 * @tparam Derived The derived class type for CRTP.
 */
template <typename Derived>
class FixedVector3D: public Vector3D<fixed, Derived> {
public:
	// -- Type Definitions --
	using Vec = Vector3D<fixed, Derived>;
	using typename Vec::NumericType;
	using Vec::rad2DegFactor;
	using Vec::Vec;
	using Vec::x;
	using Vec::y;
	using Vec::z;

	// -- Public Methods --

	// Fixed-point math functions have inherent sign conversion issues;
	// suppressing these warnings since the conversions are intentional and safe.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"

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

#pragma GCC diagnostic pop

	fixed angleTo(const VectorCompatible<fixed> auto& other) const {
		if (!(*this && other)) {
			return NumericType(0);
		}
		// 1. Raw Coordinates (Scale: 1Q)
		int64_t aX = x.raw_value();
		int64_t aY = y.raw_value();
		int64_t aZ = z.raw_value();
		int64_t bX = other.x.raw_value();
		int64_t bY = other.y.raw_value();
		int64_t bZ = other.z.raw_value();

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
		// The input (cross_sq_2Q) is a Q32 value (squared Q16). The output of the
		// sqrt will be a Q16 value, which we store in a fixed type directly.
		fixed cross_magnitude_1Q = integer_sqrt(cross_sq_2Q);

		// 6. Final Angle
		// Ensure dot product is also Q16
		fixed dot_product_1Q = fixed::from_raw_value(static_cast<fixed::base_type>(dot_raw >> fixed::FixedBits));

		return atan2(cross_magnitude_1Q, dot_product_1Q) * rad2DegFactor;
	}

	fixed dot(const VectorCompatible<fixed> auto& other) const {
		if (!(*this && other)) {
			return NumericType(0);
		}

		int64_t aX = x.raw_value();
		int64_t aY = y.raw_value();
		int64_t aZ = z.raw_value();
		int64_t bX = other.x.raw_value();
		int64_t bY = other.y.raw_value();
		int64_t bZ = other.z.raw_value();
		int64_t dot_product_raw = (int64_t)aX * bX + (int64_t)aY * bY + (int64_t)aZ * bZ;
		return fixed::from_raw_value(static_cast<fixed::base_type>(dot_product_raw >> fixed::FixedBits));
	}

	Derived cross(const VectorCompatible<fixed> auto& other) const {
		int64_t aX = x.raw_value();
		int64_t aY = y.raw_value();
		int64_t aZ = z.raw_value();
		int64_t bX = other.x.raw_value();
		int64_t bY = other.y.raw_value();
		int64_t bZ = other.z.raw_value();

		int64_t cX_raw = (int64_t)aY * bZ - (int64_t)aZ * bY;
		int64_t cY_raw = (int64_t)aZ * bX - (int64_t)aX * bZ;
		int64_t cZ_raw = (int64_t)aX * bY - (int64_t)aY * bX;

		return Derived(
			fixed::from_raw_value(static_cast<fixed::base_type>(cX_raw >> fixed::FixedBits)),
			fixed::from_raw_value(static_cast<fixed::base_type>(cY_raw >> fixed::FixedBits)),
			fixed::from_raw_value(static_cast<fixed::base_type>(cZ_raw >> fixed::FixedBits))
		);
	}

	// Magnitude calculations involve fixed-point sign conversions; suppress warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"

	fixed magnitude() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aX = x.raw_value();
		int64_t aY = y.raw_value();
		int64_t aZ = z.raw_value();

		int64_t dot_product_raw = aX * aX + aY * aY + aZ * aZ;
		return integer_sqrt(dot_product_raw);
	}

	fixed magnitudeXY() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aX = x.raw_value();
		int64_t aY = y.raw_value();

		int64_t dot_product_raw = aX * aX + aY * aY;
		return integer_sqrt(dot_product_raw);
	}

	fixed magnitudeXZ() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aX = x.raw_value();
		int64_t aZ = z.raw_value();

		int64_t dot_product_raw = aX * aX + aZ * aZ;
		return integer_sqrt(dot_product_raw);
	}

	fixed magnitudeYZ() const {
		if (!(*this)) {
			return NumericType(0);
		}
		int64_t aY = y.raw_value();
		int64_t aZ = z.raw_value();

		int64_t dot_product_raw = aY * aY + aZ * aZ;
		return integer_sqrt(dot_product_raw);
	}

#pragma GCC diagnostic pop

	Derived normalize() const {
		fixed mag = magnitude();
		if (mag != 0) {
			return Derived(x / mag, y / mag, z / mag);
		}
		return Derived(x, y, z);
	}

	NumericType pitch() const {
		auto mag = magnitudeXY();
		if (!(z || mag)) {
			return 0;
		}
		return atan2(z, magnitudeXY()) * rad2DegFactor;
	}

	NumericType yaw() const {
		if (!(x || y)) {
			return 0;
		}
		return atan2(x, y) * rad2DegFactor;
	}
};

/**
 * @brief Represents a 3D distance vector.
 *
 * This class is used to define a displacement in 3D space.
 */
class DistanceVector: public FixedVector3D<DistanceVector> {
public:
	// -- Type Definitions --
	using Vec = FixedVector3D<DistanceVector>;

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
class PositionVector: public FixedVector3D<PositionVector> {
public:
	// -- Type Definitions --
	using Vec = FixedVector3D<PositionVector>;

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
class VelocityVector: public FixedVector3D<VelocityVector> {
public:
	// -- Type Definitions --
	using Vec = FixedVector3D<VelocityVector>;

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
constexpr DistanceVector::DistanceVector(fixed dx, fixed dy, fixed dz): Vec(dx, dy, dz) {}

/**
 * @brief Constructs a DistanceVector from a VelocityVector and a time interval.
 * @param v The velocity vector.
 * @param interval The time interval.
 */
inline DistanceVector::DistanceVector(VelocityVector v, ChronoDuration auto interval) {
	*this = v * interval;
}

// -- PositionVector --
constexpr PositionVector::PositionVector(fixed px, fixed py, fixed pz): Vec(px, py, pz) {}

/**
 * @brief Constructs a PositionVector by adding a DistanceVector to a PositionVector.
 * @param p The initial position vector.
 * @param d The distance vector to add.
 */
inline PositionVector::PositionVector(PositionVector p, DistanceVector d): Vec(p.x + d.x, p.y + d.y, p.z + d.z) {}

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
constexpr VelocityVector::VelocityVector(fixed vx, fixed vy, fixed vz): Vec(vx, vy, vz) {}

/**
 * @brief Constructs a VelocityVector from a DistanceVector and a time interval.
 * @param dist The distance vector.
 * @param interval The time interval.
 */
inline VelocityVector::VelocityVector(DistanceVector dist, TimeInterval interval) {
	if (interval.count()) {
		x = dist.x / interval.count();
		y = dist.y / interval.count();
		z = dist.z / interval.count();
	}
}

// ======================================================================================
// --- Operator Overloads ---
// ======================================================================================

constexpr const VelocityVector operator/(const DistanceVector& D, const ChronoDuration auto& interval) {
	auto scale = interval.count();
	return VelocityVector(D.x / scale, D.y / scale, D.z / scale);
}

constexpr const DistanceVector operator*(const VelocityVector& V, const ChronoDuration auto& interval) {
	auto scale = static_cast<fixed>(interval.count());
	return DistanceVector(V.x, V.y, V.z) * scale;
}

constexpr const PositionVector operator+(const PositionVector& A, const DistanceVector& B) {
	return PositionVector(A.x + B.x, A.y + B.y, A.z + B.z);
}

constexpr const DistanceVector operator-(const PositionVector& A, const PositionVector& B) {
	return DistanceVector(A.x - B.x, A.y - B.y, A.z - B.z);
}
