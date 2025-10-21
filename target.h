/**
 * @file target.h
 * @brief Defines classes for representing targets, positions, velocities, and distances in 3D space.
 */
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
 * @brief Represents a 3D distance vector (a displacement).
 *
 * This class inherits from `Vector3D` and is used to define a displacement
 * in 3D space, typically representing the difference between two positions.
 */
class DistanceVector: public Vector3D<fixed, DistanceVector> {
public:
	// -- Type Definitions --
	using Vec = Vector3D<fixed, DistanceVector>;

	// -- Constructors --
	DistanceVector() = default;
	DistanceVector(const DistanceVector& other) = default;
	/**
	 * @brief Constructs a DistanceVector from x, y, and z components.
	 * @param x The x-component of the distance.
	 * @param y The y-component of the distance.
	 * @param z The z-component of the distance.
	 */
	constexpr DistanceVector(fixed x, fixed y, fixed z);
	/**
	 * @brief Constructs a DistanceVector from a velocity and a time interval.
	 * @param v The velocity vector.
	 * @param interval The duration of travel.
	 */
	DistanceVector(VelocityVector v, ChronoDuration auto interval);
};

/**
 * @brief Represents a 3D position vector (a point in space).
 *
 * This class defines a specific point in 3D space and provides methods
 * for calculating related geometric properties like pitch, yaw, and distance
 * from the origin.
 */
class PositionVector: public Vector3D<fixed, PositionVector> {
public:
	// -- Type Definitions --
	using Vec = Vector3D<fixed, PositionVector>;

	// -- Constructors --
	PositionVector() = default;
	PositionVector(const PositionVector& other) = default;
	/**
	 * @brief Constructs a PositionVector from x, y, and z coordinates.
	 * @param x The x-coordinate.
	 * @param y The y-coordinate.
	 * @param z The z-coordinate.
	 */
	constexpr PositionVector(fixed x, fixed y, fixed z);
	/**
	 * @brief Constructs a new PositionVector by displacing an existing one.
	 * @param p The starting position.
	 * @param d The distance to move.
	 */
	PositionVector(PositionVector p, DistanceVector d);
	/**
	 * @brief Constructs a new PositionVector by projecting from a starting point.
	 * @param p The starting position.
	 * @param v The velocity vector.
	 * @param interval The duration of travel.
	 */
	PositionVector(PositionVector p, VelocityVector v, ChronoDuration auto interval);

	// -- Public Methods --
	/**
	 * @brief Calculates the pitch angle to this position from the origin.
	 * @return The pitch angle in radians.
	 */
	fixed Pitch();

	/**
	 * @brief Calculates the yaw angle to this position from the origin.
	 * @return The yaw angle in radians.
	 */
	fixed Yaw();

	/**
	 * @brief Calculates the distance of this position from the origin.
	 * @return The distance.
	 */
	fixed Distance();

private:
	// -- Private Attributes --
	fixed _distance = 0; ///< Cached distance from the origin.
	fixed _pitch = 0;    ///< Cached pitch angle.
	fixed _yaw = 0;      ///< Cached yaw angle.
};

/**
 * @brief Represents a 3D velocity vector.
 *
 * This class is used to define the rate and direction of change of position.
 */
class VelocityVector: public Vector3D<fixed, VelocityVector> {
public:
	// -- Type Definitions --
	using Vec = Vector3D<fixed, VelocityVector>;

	// -- Constructors --
	VelocityVector() = default;
	VelocityVector(const VelocityVector& other) = default;
	/**
	 * @brief Constructs a VelocityVector from x, y, and z components.
	 * @param x The x-component of the velocity.
	 * @param y The y-component of the velocity.
	 * @param z The z-component of the velocity.
	 */
	constexpr VelocityVector(fixed x, fixed y, fixed z);
	/**
	 * @brief Constructs a VelocityVector from a distance traveled over a time interval.
	 * @param d The distance vector.
	 * @param interval The duration of travel.
	 */
	VelocityVector(DistanceVector d, TimeInterval interval);
};

/**
 * @brief Represents a target in the system.
 *
 * This class stores all relevant information about a target, including its
 * current and previous positions, its calculated velocity, and timing information
 * for tracking and prediction. It provides methods for updating its state,
 * predicting future positions, and calculating intercept points for ballistics.
 */
class Target {
public:
	// -- Constructors --
	Target() = default;
	Target(const Target& other) = default;
	/**
	 * @brief Constructs a Target with an initial position and velocity.
	 * @param P The initial position vector.
	 * @param V The initial velocity vector (defaults to zero).
	 */
	Target(PositionVector P, VelocityVector V = VelocityVector(0, 0, 0));
	/**
	 * @brief Constructs a Target with full initial state.
	 * @param index The index of this target in its array.
	 * @param valid Whether the target is initially valid (defaults to false).
	 * @param P The initial position vector (defaults to origin).
	 * @param V The initial velocity vector (defaults to zero).
	 */
	Target(
		uint8_t        index,
		bool           valid = false,
		PositionVector P = PositionVector(0, 0, 0),
		VelocityVector V = VelocityVector(0, 0, 0)
	);

	// -- Public Methods --
	/**
	 * @brief Updates the target's state with a new position measurement.
	 *
	 * This method updates the target's current position, calculates its velocity based on
	 * the change since the last update, and records the time of the update.
	 * @param P The new position vector of the target.
	 */
	void Update(PositionVector P);

	/**
	 * @brief Gets the pitch angle to the target's current position.
	 * @return The pitch angle in radians.
	 */
	fixed Pitch();

	/**
	 * @brief Gets the yaw angle to the target's current position.
	 * @return The yaw angle in radians.
	 */
	fixed Yaw();

	/**
	 * @brief Gets the distance to the target's current position.
	 * @return The distance.
	 */
	fixed Distance();

	/**
	 * @brief Gets the current velocity of the target.
	 * @return The velocity vector.
	 */
	const VelocityVector Velocity() const;

	/**
	 * @brief Gets the current position of the target.
	 * @return The position vector.
	 */
	const PositionVector Position() const;

	/**
	 * @brief Calculates the time elapsed since the last action was taken on this target.
	 * @return A `TimeInterval` representing the elapsed time.
	 */
	TimeInterval timeSinceLastAction() const;

	/**
	 * @brief Calculates the time elapsed since the target was last seen (updated).
	 * @return A `TimeInterval` representing the elapsed time.
	 */
	TimeInterval timeSinceLastSeen() const;

	/**
	 * @brief Checks if the time since the last action exceeds a given limit.
	 * @param limit The duration to check against.
	 * @return `true` if the idle time exceeds the limit, `false` otherwise.
	 */
	bool actionIdleExceeds(const ChronoDuration auto limit) const;

	/**
	 * @brief Checks if the time since the target was last seen exceeds a given limit.
	 * @param limit The duration to check against.
	 * @return `true` if the idle time exceeds the limit, `false` otherwise.
	 */
	bool idleExceeds(const ChronoDuration auto limit) const;

	/**
	 * @brief Resets the `last_action` timer to the current time.
	 */
	void IncrementAction();

	/**
	 * @brief Predicts the target's position at a future time.
	 * @param interval The time interval into the future.
	 * @return The predicted `PositionVector`.
	 */
	PositionVector PredictedPositionAtTime(ChronoDuration auto interval);

	/**
	 * @brief Calculates the intercept position for a projectile to hit the moving target.
	 *
	 * This method solves a quartic equation to find the time of flight and then
	 * calculates the direction the turret must aim, accounting for gravity and
	 * target motion.
	 *
	 * @return The `PositionVector` of the calculated aimpoint.
	 */
	const PositionVector interceptPosition() const;

	// -- Public Attributes --
	bool      valid = false;  ///< Whether the target's data is currently considered valid.
	uint8_t   index;          ///< The index of this target within its storage array.
	uint8_t   id;             ///< A unique identifier for the target.
	TimePoint seen;           ///< The timestamp of the last position update.
	TimePoint last_action;    ///< The timestamp of the last action taken involving this target.

private:
	// -- Private Attributes --
	TimePoint      last_seen;     ///< The timestamp of the second-to-last position update.
	PositionVector position;      ///< The current position of the target.
	PositionVector last_position; ///< The previous position of the target.
	VelocityVector velocity;      ///< The calculated velocity of the target.
};

// ======================================================================================
// --- Inline-Defined Methods ---
// ======================================================================================

// -- DistanceVector --
/**
 * @brief Constructs a DistanceVector from x, y, and z components.
 */
constexpr DistanceVector::DistanceVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}

// -- PositionVector --
/**
 * @brief Constructs a PositionVector from x, y, and z coordinates.
 */
constexpr PositionVector::PositionVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}

// -- VelocityVector --
/**
 * @brief Constructs a VelocityVector from x, y, and z components.
 */
constexpr VelocityVector::VelocityVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}

// -- Target --
/**
 * @brief Checks if the time since the last action exceeds a given limit.
 */
inline bool Target::actionIdleExceeds(const ChronoDuration auto limit) const {
	return timeSinceLastAction() > limit;
}

/**
 * @brief Checks if the time since the target was last seen exceeds a given limit.
 */
inline bool Target::idleExceeds(const ChronoDuration auto limit) const {
	return timeSinceLastSeen() > limit;
}

/**
 * @brief Calculates the intercept position for a projectile to hit the moving target.
 *
 * This function implements a numerical solution to the ballistic targeting problem
 * for a moving target under the influence of gravity. It solves a quartic equation
 * representing the time of flight, based on the work of Forrest Smith. Once the time
 * of flight (`intercept`) is found, it calculates the future position of the target
 * and adjusts for projectile drop to determine the final aimpoint.
 *
 * @return The `PositionVector` representing the required aimpoint. If the solver fails,
 *         it returns the current relative position of the target as a fallback.
 */
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

	// Quartic Coefficients for the time of flight equation: c0*t^4 + c1*t^3 + c2*t^2 + c3*t + c4 = 0
	const fixed_24_8 c0 = L * L;
	const fixed_24_8 c1 = -2 * Q * L;
	const fixed_24_8 c2 = -2 * J * L + fixed_24_8(target_velocity.dot(target_velocity)) - pow(S, 2);
	const fixed_24_8 c3 = 2 * (diff.dot(target_velocity));
	const fixed_24_8 c4 = diff.dot(diff);

	const std::function<const fixed_24_8(const fixed_24_8)> movingTargetInterceptQuartic = [=](const fixed_24_8 t
	                                                                                       ) -> const fixed_24_8 {
		return c0 * pow(t, 4) + c1 * pow(t, 3) + c2 * pow(t, 2) + c3 * t + c4;
	};

	// Find the smallest positive real root of the quartic equation
	const auto [converged, intercept] = Approximate::small_root(movingTargetInterceptQuartic);

	if (intercept == 0) {
		return PositionVector(H, K, J);
	}

	// Calculate the position of the target at the time of intercept
	auto pos = diff + target_velocity * intercept;
	// Adjust the Z coordinate for projectile drop due to gravity
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

/**
 * @brief Calculates velocity from distance and time. (Velocity = Distance / Time)
 */
constexpr const VelocityVector operator/(const DistanceVector& D, const ChronoDuration auto& interval) {
	auto scale = interval.count();
	return VelocityVector(D.X_coord / scale, D.Y_coord / scale, D.Z_coord / scale);
}

/**
 * @brief Calculates distance from velocity and time. (Distance = Velocity * Time)
 */
constexpr const DistanceVector operator*(const VelocityVector& V, const ChronoDuration auto& interval) {
	auto scale = interval.count();
	return DistanceVector(V.X_coord, V.Y_coord, V.Z_coord) * scale;
}

/**
 * @brief Calculates a new position by adding a distance vector to a position vector.
 */
constexpr const PositionVector operator+(const PositionVector& A, const DistanceVector& B) {
	return PositionVector(A.X_coord + B.X_coord, A.Y_coord + B.Y_coord, A.Z_coord + B.Z_coord);
}

/**
 * @brief Calculates the distance vector between two position vectors.
 */
constexpr const DistanceVector operator-(const PositionVector& A, const PositionVector& B) {
	return DistanceVector(A.X_coord - B.X_coord, A.Y_coord - B.Y_coord, A.Z_coord - B.Z_coord);
}