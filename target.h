#pragma once

#include <Arduino.h>

#include <stdint.h>
#include "vector.hpp"
#include "fpm_adapter.hpp"
#include <chrono>
#include "aproximate_math.hpp"
#include "utilities.h"

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
class DistanceVector : public Vector3D<fixed, DistanceVector>
{
	using Vec = Vector3D<fixed, DistanceVector>;

public:
	DistanceVector() = default;
	DistanceVector(const DistanceVector &other) = default;
	constexpr DistanceVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}
	DistanceVector(VelocityVector, ChronoDuration auto interval);
};

/**
 * @brief Represents a 3D position vector.
 *
 * This class defines a specific point in 3D space and provides methods
 * to calculate pitch, yaw, and distance.
 */
class PositionVector : public Vector3D<fixed, PositionVector>
{
	using Vec = Vector3D<fixed, PositionVector>;
	fixed _distance = 0; ///< Cached distance value.
	fixed _pitch = 0;    ///< Cached pitch value.
	fixed _yaw = 0;      ///< Cached yaw value.

public:
	PositionVector() = default;
	PositionVector(const PositionVector &other) = default;
	constexpr PositionVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}
	PositionVector(PositionVector, DistanceVector);
	PositionVector(PositionVector, VelocityVector, ChronoDuration auto interval);

public:
	fixed Pitch();
	fixed Yaw();
	fixed Distance();
};

/**
 * @brief Represents a 3D velocity vector.
 *
 * This class is used to define the rate of change of position.
 */
class VelocityVector : public Vector3D<fixed, VelocityVector>
{
	using Vec = Vector3D<fixed, VelocityVector>;

public:
	VelocityVector() = default;
	VelocityVector(const VelocityVector &other) = default;
	constexpr VelocityVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}
	VelocityVector(DistanceVector, TimeInterval interval);
};

/**
 * @brief Represents a target in the system.
 *
 * This class stores all relevant information about a target, including its
 * position, velocity, and timing information. It also provides methods for
 * predicting future positions and calculating intercept points.
 */
class Target
{
public:
	Target() = default;
	Target(const Target &other) = default;
	Target(PositionVector P, VelocityVector V = VelocityVector(0, 0, 0));
	Target(uint8_t index, bool valid = false, PositionVector P = PositionVector(0, 0, 0), VelocityVector V = VelocityVector(0, 0, 0));

public:
	void Update(PositionVector P);

public:
	fixed Pitch();
	fixed Yaw();
	fixed Distance();

	const VelocityVector Velocity() const;
	const PositionVector Position() const;

	TimeInterval timeSinceLastAction() const;
	TimeInterval timeSinceLastSeen() const;

	/**
	 * @brief Checks if the time since the last action exceeds a limit.
	 * @param limit The duration to check against.
	 * @return True if the idle time exceeds the limit, false otherwise.
	 */
	bool actionIdleExceeds(const ChronoDuration auto limit) const
	{
		return timeSinceLastAction() > limit;
	}

	/**
	 * @brief Checks if the time since the target was last seen exceeds a limit.
	 * @param limit The duration to check against.
	 * @return True if the idle time exceeds the limit, false otherwise.
	 */
	bool idleExceeds(const ChronoDuration auto limit) const
	{
		return timeSinceLastSeen() > limit;
	}

	void IncrementAction();
	PositionVector PredictedPositionAtTime(ChronoDuration auto interval);

	/**
	 * @brief Calculates the intercept position for a moving target.
	 *
	 * This function solves a quartic equation to find the time of intercept,
	 * considering the projectile's speed and the effect of gravity.
	 *
	 * @return The calculated position vector for interception.
	 */
	const PositionVector interceptPosition() const
	{
		const PositionVector proj_pos = PositionVector(0, 0, 1.5);
		const PositionVector target_pos = position;
		const VelocityVector target_velocity = velocity;
		const fixed_24_8 proj_speed = 20;
		const Vector3D<fixed_24_8> Gv(0, 0, 9.814);
		const fixed_24_8 G = Gv.magnitude();

		const fixed_24_8 P = target_velocity.X_coord;
		const fixed_24_8 Q = target_velocity.Z_coord;
		const fixed_24_8 R = target_velocity.Y_coord;

		const auto diff = target_pos - proj_pos;
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

		const std::function<const fixed_24_8(const fixed_24_8)> movingTargetInterceptQuartic = [=](const fixed_24_8 t) -> const fixed_24_8
		{
			return c0 * pow(t, 4) + c1 * pow(t, 3) + c2 * pow(t, 2) + c3 * t + c4;
		};

		const auto [converged, intercept] = Approximate::small_root(movingTargetInterceptQuartic);

		auto pos = diff + target_velocity * intercept;
		pos.Z_coord = fixed_24_8(pos.Z_coord) - L * pow(intercept, 2);

		return PositionVector(
			(H + P * intercept) / intercept,
			(K + R * intercept) / intercept,
			(J + Q * intercept - L * pow(intercept, 2)) / intercept);
	};

public:
	bool valid = false;
	uint8_t index;
	uint8_t id;
	TimePoint seen;
	TimePoint last_action;

private:
	TimePoint last_seen;
	PositionVector position;
	PositionVector last_position;
	VelocityVector velocity;
};

constexpr const VelocityVector operator/(const DistanceVector &D, const ChronoDuration auto &interval)
{
	auto scale = interval.count();
	return VelocityVector(
		D.X_coord / scale,
		D.Y_coord / scale,
		D.Z_coord / scale);
}

constexpr const DistanceVector operator*(const VelocityVector &V, const ChronoDuration auto &interval)
{
	auto scale = interval.count();
	return DistanceVector(V.X_coord, V.Y_coord, V.Z_coord) * scale;
}

constexpr const PositionVector operator+(const PositionVector &A, const DistanceVector &B)
{
	return PositionVector(
		A.X_coord + B.X_coord,
		A.Y_coord + B.Y_coord,
		A.Z_coord + B.Z_coord);
}

constexpr const DistanceVector operator-(const PositionVector &A, const PositionVector &B)
{
	return DistanceVector(
		A.X_coord - B.X_coord,
		A.Y_coord - B.Y_coord,
		A.Z_coord - B.Z_coord);
}
