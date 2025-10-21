/**
 * @file state.h
 * @brief Defines the SystemState class, which manages the overall state of the turret system.
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
 * @brief Holds tunable configuration parameters for the system.
 *
 * These values are expected to be set at runtime via a configuration message,
 * allowing for dynamic adjustment of the turret's performance characteristics.
 */
struct ConfigParameters {
	fixed projectile_speed = projectileSpeed; ///< The initial speed of the projectile in meters/second.
	fixed turret_height = altitude;           ///< The height of the turret from the ground in meters.
};

/**
 * @brief A comparator for ordering Command pointers in a priority queue.
 *
 * This lambda function is used by `std::priority_queue` to sort commands based on their
 * `run_after` timestamp. Commands scheduled to run earlier (with a smaller timestamp)
 * will have a higher priority.
 */
const inline auto CommandPointerComparator = [](const auto& left, const auto& right) {
	return left->run_after >= right->run_after;
};

/**
 * @brief Manages the overall state of the turret system.
 *
 * This class encapsulates all system components, including motors, targets,
 * and the command queue. It provides a centralized interface for updating and
 * controlling the system's behavior, processing commands, and managing targets
 * from various sources (CV, Radar, Static).
 */
class SystemState {
public:
	// -- Constructors --
	/**
	 * @brief Constructs a new SystemState object.
	 *
	 * Initializes the stepper motors, sets up pin modes, creates the mutex for
	 * thread safety, and sets the default target source to STATIC.
	 */
	SystemState();

	// -- Public Methods --
	/**
	 * @brief Gets the currently active target.
	 * @return A reference to the active Target object.
	 */
	Target& currentTarget();

	/**
	 * @brief Gets the array of targets for the current target source.
	 * @return A std::span over the array of Target objects.
	 */
	std::span<Target> currentTargetArray();

	/**
	 * @brief Gets the number of targets for the current target source.
	 * @return The size of the current target array.
	 */
	constexpr size_t size();

	/**
	 * @brief Updates a target at a specific index in a given target array.
	 *
	 * This method updates the position and validity of a target. An indifference margin
	 * can be used to prevent small, insignificant updates.
	 *
	 * @param targetArray The array of targets to modify.
	 * @param idx The index of the target to update.
	 * @param valid The new validity status of the target.
	 * @param newPosition The new position of the target.
	 * @param indifferenceMargin An optional margin (in motor steps) to prevent minor updates.
	 */
	void updateTarget(
		auto&           targetArray,
		const uint8_t   idx,
		const bool      valid,
		PositionVector& newPosition,
		const uint16_t  indifferenceMargin = 0
	);

	/**
	 * @brief Updates a target in a given array by its ID.
	 *
	 * Searches for a target with the given ID. If not found, it tries to use an invalid
	 * target slot or, as a last resort, overwrites the least recently seen target.
	 *
	 * @param targetArray The array of targets to modify.
	 * @param id The ID of the target to update.
	 * @param valid The new validity status of the target.
	 * @param newPosition The new position of the target.
	 * @param indifferenceMargin An optional margin to prevent minor updates.
	 */
	void updateTargetById(
		auto&           targetArray,
		const uint8_t   id,
		const bool      valid,
		PositionVector& newPosition,
		const uint16_t  indifferenceMargin = 0
	);

	/**
	 * @brief Updates the nearest target in the current array to a new position.
	 * @param valid The new validity status.
	 * @param newPosition The new position.
	 * @param indifferenceMargin An optional margin to prevent minor updates.
	 */
	void updateNearestTarget(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);

	/**
	 * @brief Updates the nearest target in the current array based on 2D distance.
	 * @param valid The new validity status.
	 * @param newPosition The new position.
	 * @param indifferenceMargin An optional margin to prevent minor updates.
	 */
	void updateNearestTarget2d(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);

	/**
	 * @brief Sets the currently active target by its index.
	 * @param index The index of the target to make active.
	 * @param speed The tracking speed for the motors.
	 */
	void setTarget(uint8_t index, uint8_t speed = 0xFF);

	/**
	 * @brief Sets the firing state of the turret.
	 * @param active True to fire, false to stop firing.
	 */
	void setFire(bool active);

	/**
	 * @brief Enables or disables motor movement.
	 * @param active True to enable movement, false to disable.
	 */
	void setMove(bool active);

	/**
	 * @brief Gets the current firing state.
	 * @return True if the turret is currently firing.
	 */
	bool getFireState();

	/**
	 * @brief Gets the current movement state.
	 * @return True if motor movement is enabled.
	 */
	bool getMoveState();

	/**
	 * @brief Queues a command to select a target after a delay.
	 * @param index The index of the target to select.
	 * @param milliseconds The delay in milliseconds before the command executes.
	 */
	void queueSelectTarget(uint8_t index, uint16_t milliseconds);

	/**
	 * @brief Queues a command to fire the turret after a delay.
	 * @param milliseconds The delay in milliseconds before the command executes.
	 */
	void queueFire(uint16_t milliseconds);

	/**
	 * @brief Queues a command to do nothing for a specified duration.
	 *
	 * This can be used to introduce a delay in the command sequence.
	 * @param milliseconds The duration of the linger in milliseconds.
	 */
	void queueLinger(uint8_t milliseconds);

	/**
	 * @brief Updates the system's configuration from a message.
	 * @param config A pointer to the Config message containing the new parameters.
	 */
	void updateConfig(cerializer::Config* config);

	/**
	 * @brief Processes the command queue, executing any commands that are due.
	 */
	void processCommandQueue();

	/**
	 * @brief Updates the physical state of the system based on the current logical state.
	 *
	 * This method should be called repeatedly in the main loop. It handles motor
	 * movement and firing.
	 */
	void actualizeState();

	/**
	 * @brief Fetches a target from the current target array by its index.
	 * @param idx The index of the target to fetch.
	 * @return A reference to the requested Target object.
	 */
	Target& fetchTarget(const uint8_t idx);

	/**
	 * @brief Finds the index of the nearest target to a given 3D point.
	 * @param point The point to compare against.
	 * @return The index of the nearest target.
	 */
	uint8_t fetchNearestTargetIdx(const PositionVector& point);

	/**
	 * @brief Finds the index of the nearest target to a given point, using only 2D distance.
	 * @param point The point to compare against.
	 * @return The index of the nearest target.
	 */
	uint8_t fetchNearestTarget2dIdx(const PositionVector& point);

	/**
	 * @brief Calculates the distance the turret needs to travel to aim at the current target.
	 * @return The travel distance as a fixed-point number.
	 */
	fixed targetTravelDistance();

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

	cerializer::TargetSource target_source; ///< The active source for targeting data (CV, Radar, etc.).
	std::array<Target, 32>   cvTarget;      ///< Array to store targets from the computer vision system.
	std::array<Target, 3>    radarTarget;   ///< Array to store targets from the radar sensor.
	Target                   staticTarget;  ///< A single target for static aiming mode.

private:
	// -- Private Methods --
	/**
	 * @brief Updates the physical position of the motors.
	 *
	 * This method is called by `actualizeState` to move the steppers towards their
	 * target positions.
	 */
	void actualizePosition();

	/**
	 * @brief Activates or deactivates the firing mechanism.
	 *
	 * This method is called by `actualizeState` to control the firing pin.
	 */
	void actualizeFiring();

	/**
	 * @brief Calculates the required aimpoint to hit the current target.
	 *
	 * This method accounts for projectile drop and target movement to determine
	 * where the turret should aim.
	 * @return The calculated aimpoint as a PositionVector.
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

	/// @brief Priority queue for pending commands, ordered by execution time.
	std::priority_queue<Command*, std::vector<Command*>, decltype(CommandPointerComparator)>
		commandQueue;
};

// ======================================================================================
// --- Inline-Defined Methods ---
// ======================================================================================

/**
 * @brief Gets the number of targets for the current target source.
 * @return The size of the current target array.
 */
constexpr size_t SystemState::size() {
	return currentTargetArray().size();
}

/**
 * @brief Updates a target at a specific index in a given target array.
 *
 * This method updates the position and validity of a target. An indifference margin
 * can be used to prevent small, insignificant updates.
 *
 * @param targetArray The array of targets to modify.
 * @param idx The index of the target to update.
 * @param valid The new validity status of the target.
 * @param newPosition The new position of the target.
 * @param indifferenceMargin An optional margin (in motor steps) to prevent minor updates.
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
 * @brief Updates a target in a given array by its ID.
 *
 * Searches for a target with the given ID. If not found, it tries to use an invalid
 * target slot or, as a last resort, overwrites the least recently seen target.
 *
 * @param targetArray The array of targets to modify.
 * @param id The ID of the target to update.
 * @param valid The new validity status of the target.
 * @param newPosition The new position of the target.
 * @param indifferenceMargin An optional margin to prevent minor updates.
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
 * @brief Fetches a target from the current target array by its index.
 * @param idx The index of the target to fetch.
 * @return A reference to the requested Target object.
 */
inline Target& SystemState::fetchTarget(const uint8_t idx) {
	return currentTargetArray()[idx];
}

/**
 * @brief Finds the index of the nearest target to a given 3D point.
 * @param point The point to compare against.
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
 * @brief Finds the index of the nearest target to a given point, using only 2D distance.
 * @param point The point to compare against.
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