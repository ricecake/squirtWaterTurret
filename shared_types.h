#pragma once

#include <cstdint>

/**
 * @brief An enum to specify the source of targeting data.
 */
enum class TargetSource : uint8_t {
	STATIC, ///< Target is a fixed, statically defined point.
	RADAR,  ///< Target is provided by the onboard radar sensor.
	CV,     ///< Target is provided by the external computer vision system.
};

/**
 * @brief An enum to specify the turret's targeting strategy.
 */
enum class TurretStrategy : uint8_t {
	LEAST_HIT,
	MOST_HIT,
	CLOSEST,
	FURTHEST,
	LEAST_RECENT,
	MOST_RECENT,
	RANDOM,
	SMALLEST_TRAVEL,
	LONGEST_TRAVEL
};


/**
 * @brief An enum to specify the turret's firing stance.
 */
enum class TurretStance : uint8_t {
	AGGRESSIVE,
	PASSIVE,
	NEUTRAL
};
