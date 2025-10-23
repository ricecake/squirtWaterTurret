/**
 * @file state.h
 * @brief Defines the SystemState class, which manages the overall state of the turret system.
 *
 * This file contains the declaration of the SystemState class, which encapsulates all
 * system components, including motors, targets, configuration parameters, and the
 * command queue. It provides the central logic for controlling the turret's behavior.
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <span>

#ifdef ARDUINO
	#include "freertos/FreeRTOS.h"
	#include "freertos/semphr.h"
	#include <AccelStepper.h>
	#include <Arduino.h>
#else
	#include "tests/mocks.h"
#endif

#include "command.h"
#include "fpm_adapter.hpp"
#include "serializer.hpp"
#include "target.h"
#include "utilities.h"
#include "vector.hpp"

using fixed = fixed_16_16;
class Command;

/**
 * @brief A struct to hold tunable configuration parameters for the system.
 *
 * These values are expected to be set at runtime via a configuration message,
 * allowing for on-the-fly adjustments to the turret's performance.
 */
struct ConfigParameters {
	fixed projectile_speed = projectileSpeed; ///< The initial speed of the projectile in meters/second.
	fixed turret_height = altitude;           ///< The height of the turret from the ground in meters.
};

/**
 * @brief A comparator for sorting commands in the priority queue.
 *
 * This lambda function compares two command pointers based on their `run_after`
 * timestamp, ensuring that commands scheduled to run earlier have higher priority.
 */
const inline auto CommandPointerComparator = [](const auto& left, const auto& right) {
	return left->run_after >= right->run_after;
};

/**
 * @brief Manages the overall state of the system.
 *
 * This class encapsulates all system components, including motors, targets,
 * and the command queue. It provides a central interface for updating and controlling
 * the system's behavior, processing commands, and actualizing the physical state
 * of the hardware.
 */
class SystemState {
public:
	// -- Constructors --
	/**
	 * @brief Constructs a new SystemState object.
	 *
	 * Initializes motors, sets default target source, and prepares the command queue.
	 */
	SystemState();

	// -- Public Methods --
	/**
	 * @brief Gets the currently active target.
	 * @return A reference to the active Target object.
	 */
	Target&           currentTarget();
	/**
	 * @brief Gets the array of targets corresponding to the current target source.
	 * @return A std::span referencing the active target array.
	 */
	std::span<Target> currentTargetArray();
	/**
	 * @brief Gets the size of the current target array.
	 * @return The number of targets in the active array.
	 */
	constexpr size_t  size();

	/**
	 * @brief Updates a target's position and validity by its index in an array.
	 * @param targetArray The array of targets to modify.
	 * @param idx The index of the target to update.
	 * @param valid The new validity state of the target.
	 * @param newPosition The new position vector of the target.
	 * @param indifferenceMargin An angle (in motor steps) threshold; updates are ignored if the change is smaller than
	 * this.
	 */
	void updateTarget(
		auto&           targetArray,
		const uint8_t   idx,
		const bool      valid,
		PositionVector& newPosition,
		const uint16_t  indifferenceMargin = 0
	);
	/**
	 * @brief Updates a target's position and validity by its unique ID.
	 *
	 * If a target with the given ID exists, it is updated. If not, it attempts to
	 * find an invalid target to replace. If all are valid, it replaces the least-recently-seen target.
	 *
	 * @param targetArray The array of targets to modify.
	 * @param id The unique ID of the target to update.
	 * @param valid The new validity state of the target.
	 * @param newPosition The new position vector of the target.
	 * @param indifferenceMargin An angle threshold; updates are ignored if the change is smaller than this.
	 */
	void updateTargetById(
		auto&           targetArray,
		const uint8_t   id,
		const bool      valid,
		PositionVector& newPosition,
		const uint16_t  indifferenceMargin = 0
	);
	/**
	 * @brief Updates the target that is nearest to a given position in 3D space.
	 * @param valid The new validity state of the target.
	 * @param newPosition The new position vector to compare against.
	 * @param indifferenceMargin An angle threshold to prevent minor updates.
	 */
	void    updateNearestTarget(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);
	/**
	 * @brief Updates the target that is nearest to a given position in 2D space (X and Y axes).
	 * @param valid The new validity state of the target.
	 * @param newPosition The new position vector to compare against.
	 * @param indifferenceMargin An angle threshold to prevent minor updates.
	 */
	void    updateNearestTarget2d(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);
	/**
	 * @brief Sets the currently selected target for aiming.
	 * @param index The index of the target to select from the current target array.
	 * @param speed The speed at which to move to the target.
	 */
	void    setTarget(uint8_t index, uint8_t speed = 0xFF);
	/**
	 * @brief Activates or deactivates the firing mechanism.
	 * @param active The desired firing state (true to fire, false to stop).
	 */
	void    setFire(bool active);
	/**
	 * @brief Enables or disables motor movement.
	 * @param active The desired movement state (true to enable, false to disable).
	 */
	void    setMove(bool active);
	/**
	 * @brief Gets the current state of the firing mechanism.
	 * @return True if firing is active, false otherwise.
	 */
	bool    getFireState();
	/**
	 * @brief Gets the current state of motor movement.
	 * @return True if movement is enabled, false otherwise.
	 */
	bool    getMoveState();
	/**
	 * @brief Queues a command to select a target after a delay.
	 * @param index The index of the target to select.
	 * @param milliseconds The delay in milliseconds before the command executes.
	 */
	void    queueSelectTarget(uint8_t index, uint16_t milliseconds);
	/**
	 * @brief Queues a command to fire the weapon after a delay.
	 * @param milliseconds The delay in milliseconds before the command executes.
	 */
	void    queueFire(uint16_t milliseconds);
	/**
	 * @brief Queues a command to pause execution for a duration.
	 * @param milliseconds The duration of the pause in milliseconds.
	 */
	void    queueLinger(uint8_t milliseconds);
	/**
	 * @brief Updates the system's configuration parameters from a message.
	 * @param config A pointer to the configuration message containing the new values.
	 */
	void    updateConfig(cerializer::Config* config);
	/**
	 * @brief Processes commands from the command queue that are due to be run.
	 *
	 * This method should be called repeatedly in the main loop.
	 */
	void    processCommandQueue();
	/**
	 * @brief Updates the physical state of the hardware (motors, firing pin) to match the desired state.
	 *
	 * This method should be called repeatedly in the main loop.
	 */
	void    actualizeState();
	/**
	 * @brief Fetches a target by its index from the current target array.
	 * @param idx The index of the target to retrieve.
	 * @return A reference to the requested Target object.
	 */
	Target& fetchTarget(const uint8_t idx);
	/**
	 * @brief Finds the index of the target nearest to a given 3D point.
	 * @param point The position vector to compare against.
	 * @return The index of the nearest target.
	 */
	uint8_t fetchNearestTargetIdx(const PositionVector& point);
	/**
	 * @brief Finds the index of the target nearest to a given 2D point (X and Y axes).
	 * @param point The position vector to compare against.
	 * @return The index of the nearest target.
	 */
	uint8_t fetchNearestTarget2dIdx(const PositionVector& point);
	/**
	 * @brief Calculates the travel distance for the motors to aim at the current target.
	 * @return The travel distance as a fixed-point number.
	 */
	fixed   targetTravelDistance();

	// -- Public Attributes --
	const int   stepFraction = 16;   ///< Microstep fraction for the stepper motors.
	const int   h_max = 500;         ///< Maximum horizontal position.
	const int   v_max = 1000;        ///< Maximum vertical position.
	const int   h_min = -500;        ///< Minimum horizontal position.
	const int   v_min = -1000;       ///< Minimum vertical position.
	const fixed angleToStep{0.1125}; ///< Conversion factor from angle to motor steps.

	ConfigParameters  config;   ///< Runtime configuration parameters.
	AccelStepper      stepperA; ///< Stepper motor A instance.
	AccelStepper      stepperB; ///< Stepper motor B instance.
	SemaphoreHandle_t xMutex;   ///< Mutex for thread-safe access to shared resources.

	cerializer::TargetSource target_source;  ///< The active source for targeting data (CV, Radar, Static).
	std::array<Target, 32>   cvTarget;       ///< Array to store targets from the computer vision system.
	std::array<Target, 3>    radarTarget;    ///< Array to store targets from the radar system.
	Target                   staticTarget;   ///< A single target for static aiming.

private:
	// -- Private Methods --
	/**
	 * @brief Moves the motors to aim at the calculated aimpoint for the current target.
	 */
	void           actualizePosition();
	/**
	 * @brief Sets the state of the firing pin.
	 */
	void           actualizeFiring();
	/**
	 * @brief Calculates the required aimpoint to hit the current target, accounting for gravity and projectile speed.
	 * @return The calculated position vector of the aimpoint.
	 */
	PositionVector targetAimpoint();

	// -- Private Attributes --
	const int motorInterfaceType = 1; ///< Stepper motor driver interface type.
	const int stepPinA = 32;          ///< Step pin for stepper motor A.
	const int dirPinA = 33;           ///< Direction pin for stepper motor A.
	const int stepPinB = 25;          ///< Step pin for stepper motor B.
	const int dirPinB = 26;           ///< Direction pin for stepper motor B.
	const int firePin = 2;            ///< Pin for the firing mechanism.

	bool    moveState = true;           ///< Flag indicating if movement is enabled.
	bool    fireState = false;          ///< Flag indicating the current firing state.
	bool    needTrackingUpdate = false; ///< Flag indicating if a tracking update is required.
	uint8_t trackingSpeed = 255;        ///< The speed for tracking movements.
	uint8_t selectedTarget = 0;         ///< The index of the currently selected target.

	///< Priority queue for pending commands, sorted by execution time.
	std::priority_queue<Command*, std::vector<Command*>, decltype(CommandPointerComparator)>
		commandQueue;
};

// ======================================================================================
// --- Inline-Defined Methods ---
// ======================================================================================

/**
 * @brief Gets the size of the current target array.
 * @return The number of targets in the active array.
 */
constexpr size_t SystemState::size() {
	return currentTargetArray().size();
}

/**
 * @brief Updates a target's position and validity by its index in an array.
 * @param targetArray The array of targets to modify.
 * @param idx The index of the target to update.
 * @param valid The new validity state of the target.
 * @param newPosition The new position vector of the target.
 * @param indifferenceMargin An angle (in motor steps) threshold; updates are ignored if the change is smaller than
 * this.
 */
inline void SystemState::updateTarget(
	auto&           targetArray,
	const uint8_t   idx,
	const bool      valid,
	PositionVector& newPosition,
	const uint16_t  indifferenceMargin
) {
	bool doUpdate = true;
	if (indifferenceMargin > 0) {
		auto oldTarget = targetArray[idx];
		auto oldTargetPos = oldTarget.Position();
		if (oldTargetPos) {
			auto travelAngle = abs(oldTargetPos.angleTo(newPosition)) / angleToStep;
			doUpdate = (travelAngle) > indifferenceMargin;
		}
	}

	if (doUpdate) {
		// Need something that can indicate that this is a reduced dimension measurement, so we only update fields that
		// are real
		targetArray[idx].Update(newPosition);
		targetArray[idx].valid = valid;
		needTrackingUpdate = true;
	}
}

/**
 * @brief Updates a target's position and validity by its unique ID.
 *
 * If a target with the given ID exists, it is updated. If not, it attempts to
 * find an invalid target to replace. If all are valid, it replaces the least-recently-seen target.
 *
 * @param targetArray The array of targets to modify.
 * @param id The unique ID of the target to update.
 * @param valid The new validity state of the target.
 * @param newPosition The new position vector of the target.
 * @param indifferenceMargin An angle threshold; updates are ignored if the change is smaller than this.
 */
inline void SystemState::updateTargetById(
	auto&           targetArray,
	const uint8_t   id,
	const bool      valid,
	PositionVector& newPosition,
	const uint16_t  indifferenceMargin
) {
	auto pred = [&](const Target& item) { return item.id == id; };
	auto found = std::ranges::find_if(targetArray, pred);

	if (found == targetArray.end()) {
		auto pred = [&](const Target& item) { return item.valid == false; };
		found = std::ranges::find_if(targetArray, pred);
	}

	if (found == targetArray.end()) {
		auto pred = [&](const Target& item) { return item.seen; };
		found = std::ranges::min_element(targetArray, std::ranges::less{}, pred);
	}

	updateTarget(targetArray, found->index, valid, newPosition, indifferenceMargin);
	targetArray[found->index].id = id;
};

/**
 * @brief Fetches a target by its index from the current target array.
 * @param idx The index of the target to retrieve.
 * @return A reference to the requested Target object.
 */
inline Target& SystemState::fetchTarget(const uint8_t idx) {
	return currentTargetArray()[idx];
}

/**
 * @brief Finds the index of the target nearest to a given 3D point.
 * @param point The position vector to compare against.
 * @return The index of the nearest target.
 */
inline uint8_t SystemState::fetchNearestTargetIdx(const PositionVector& point) {
	auto distance = [&](Target& item) {
		auto pos = item.Position();
		return pow(pos.X_coord - point.X_coord, 2) + pow(pos.Y_coord - point.Y_coord, 2) +
			pow(pos.Z_coord - point.Z_coord, 2);
	};
	auto res = std::ranges::min_element(currentTargetArray(), std::ranges::less{}, distance);
	return std::ranges::distance(currentTargetArray().begin(), res);
}

/**
 * @brief Finds the index of the target nearest to a given 2D point (X and Y axes).
 * @param point The position vector to compare against.
 * @return The index of the nearest target.
 */
inline uint8_t SystemState::fetchNearestTarget2dIdx(const PositionVector& point) {
	auto distance = [&](Target& item) {
		auto pos = item.Position();
		return pow(pos.X_coord - point.X_coord, 2) + pow(pos.Y_coord - point.Y_coord, 2);
	};
	auto res = std::ranges::min_element(currentTargetArray(), std::ranges::less{}, distance);
	return std::ranges::distance(currentTargetArray().begin(), res);
}