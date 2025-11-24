#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#ifdef ARDUINO
	#include "freertos/FreeRTOS.h"
	#include "freertos/semphr.h"
	#include <AccelStepper.h>
	#include <Arduino.h>
#else
	// Forward declare the non-arduino types
	class AccelStepper;
#endif

#include "command.h"
#include "command_queue.h"
#include "fpm_adapter.hpp"
#include "logger.h"
#include "shared_types.h"
#include "target.h"
#include "utilities.h"
#include "vector.hpp"

using fixed = fixed_16_16;
class Command;
namespace cerializer {
	class Config;
}

/**
 * @brief A struct to hold tunable configuration parameters for the system.
 *
 * These values are expected to be set at runtime via a configuration message.
 */
struct ConfigParameters {
	fixed projectile_speed = projectileSpeed; ///< The initial speed of the projectile in meters/second.
	fixed turret_height = altitude;           ///< The height of the turret from the ground in meters.
	fixed projectile_max_range = 40.0;        ///< The maximum effective range of the projectile in meters.
};

/**
 * @brief Manages the overall state of the system.
 *
 * This class encapsulates all system components, including motors, targets,
 * and the command queue. It provides an interface for updating and controlling
 * the system's behavior.
 */
class SystemState {
public:
	// -- Constructors --
	SystemState();

	// -- Public Methods --
	Target*           currentTarget();
	std::span<Target> currentTargetArray();
	inline size_t     size();

	void updateTarget(
		auto&           targetArray,
		const uint8_t   idx,
		const bool      valid,
		PositionVector& newPosition,
		const uint16_t  indifferenceMargin = 0
	);
	void updateTargetById(
		auto&           targetArray,
		const uint8_t   id,
		const bool      valid,
		PositionVector& newPosition,
		const uint16_t  indifferenceMargin = 0
	);
	void    updateNearestTarget(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);
	void    updateNearestTarget2d(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);
	void    setTarget(TargetSource source, uint8_t index, uint8_t speed = 0xFF);
	void    setFire(bool active);
	void    clearFire(bool active);
	void    setMove(bool active);
	void    setStrategy(TurretStrategy strategy);
	void    setStance(TurretStance stance);
	bool    getFireState();
	bool    getMoveState();
	bool    shouldCheckTargetValidity();
	bool    shouldCheckFiringConditions();
	bool    targetIsPotentiallyValid();
	void    queueSelectTarget(TargetSource source, uint8_t index);
	void    queueFire(uint16_t milliseconds);
	void    queueCeaseFire(uint16_t milliseconds);
	void    updateConfig(cerializer::Config* config);
	void    processCommandQueue();
	void    actualizeState();
	Target& fetchTarget(const uint8_t idx);
	uint8_t fetchNearestTargetIdx(const PositionVector& point);
	uint8_t fetchNearestTarget2dIdx(const PositionVector& point);
	fixed   targetTravelDistance();

	// -- Public Attributes --
	const int   stepFraction = 16;   ///< Microstep fraction for the stepper motors.
	const int   h_max = 500;         ///< Maximum horizontal position.
	const int   v_max = 1000;        ///< Maximum vertical position.
	const int   h_min = -500;        ///< Minimum horizontal position.
	const int   v_min = -1000;       ///< Minimum vertical position.
	const fixed angleToStep{0.1125}; ///< Conversion factor from angle to motor steps.

	ConfigParameters config;   ///< Runtime configuration parameters.
	AccelStepper     stepperA; ///< Stepper motor A instance.
	AccelStepper     stepperB; ///< Stepper motor B instance.

	TargetSource           target_source;
	std::array<Target, 32> cvTarget;
	std::array<Target, 3>  radarTarget;
	Target                 staticTarget;

	Target* selectedTarget = &staticTarget; //< A reference to the currently selected target

	const fixed currentYaw();
	const fixed currentPitch();

	TurretStrategy currentStrategy() const { return strategy; }

private:
	// -- Private Methods --
	void           actualizePosition();
	void           actualizeFiring();
	PositionVector targetAimpoint();
	PositionVector getAimpoint();

	// -- Private Attributes --
	const int motorInterfaceType = 1; ///< Stepper motor driver interface type.
	const int stepPinA = 32;          ///< Step pin for stepper motor A.
	const int dirPinA = 33;           ///< Direction pin for stepper motor A.
	const int stepPinB = 25;          ///< Step pin for stepper motor B.
	const int dirPinB = 26;           ///< Direction pin for stepper motor B.
	const int firePin = 2;            ///< Pin for the firing mechanism.

	bool moveState = true;           ///< Flag indicating if movement is enabled.
	bool fireState = false;          ///< Flag indicating the current firing state.
	bool needTrackingUpdate = false; ///< Flag indicating if a tracking update is required.
	bool targetChangeProcessing = false;
	bool fireOrderProcessing = false;

	uint8_t        trackingSpeed = 255; ///< The speed for tracking movements.
	TurretStrategy strategy = TurretStrategy::RANDOM;
	TurretStance   stance;

	CommandQueue commandQueue; ///< Priority queue for pending commands.
};

// ======================================================================================
// --- Inline-Defined Methods ---
// ======================================================================================

inline size_t SystemState::size() {
	return currentTargetArray().size();
}

inline void SystemState::updateTarget(
	auto&           targetArray,
	const uint8_t   idx,
	const bool      valid,
	PositionVector& newPosition,
	const uint16_t //  indifferenceMargin
) {
	bool doUpdate = true;
	bool activeTarget = false;
	if (selectedTarget == &targetArray[idx]) {
		// logger::DEBUG("UPDATING ACTIVE TARGET");
		activeTarget = true;
	}

	// if (activeTarget && indifferenceMargin > 0) {
	// 	logger::DEBUG("Checking indifference");
	// 	auto oldTarget = targetArray[idx];
	// 	auto oldTargetPos = oldTarget.Position();
	// 	if (oldTarget.valid && oldTargetPos) {
	// 		auto travelAngle = oldTargetPos.angleTo(newPosition);// / angleToStep;
	// 		doUpdate = (travelAngle) > indifferenceMargin;
	// 		logger::DEBUG("TRAVEL STEPs", travelAngle);
	// 	}
	// }

	if (doUpdate) {
		// Need something that can indicate that this is a reduced dimension measurement, so we only update fields that
		// are real
		targetArray[idx].Update(newPosition);
		targetArray[idx].valid = valid;
		if (activeTarget) {
			// logger::DEBUG("SETTING TRACKING FOR ACTIVE TARGET");
			needTrackingUpdate = true;
		}
	}
}

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

inline Target& SystemState::fetchTarget(const uint8_t idx) {
	return currentTargetArray()[idx];
}

inline uint8_t SystemState::fetchNearestTargetIdx(const PositionVector& point) {
	auto distance = [&](Target& item) {
		auto pos = item.Position();
		return pow(pos.X_coord - point.X_coord, 2) + pow(pos.Y_coord - point.Y_coord, 2) +
			pow(pos.Z_coord - point.Z_coord, 2);
	};
	auto res = std::ranges::min_element(currentTargetArray(), std::ranges::less{}, distance);
	return std::ranges::distance(currentTargetArray().begin(), res);
}

inline uint8_t SystemState::fetchNearestTarget2dIdx(const PositionVector& point) {
	auto distance = [&](Target& item) {
		auto pos = item.Position();
		return pow(pos.X_coord - point.X_coord, 2) + pow(pos.Y_coord - point.Y_coord, 2);
	};
	auto res = std::ranges::min_element(currentTargetArray(), std::ranges::less{}, distance);
	return std::ranges::distance(currentTargetArray().begin(), res);
}

inline bool SystemState::targetIsPotentiallyValid() {
	auto target = currentTarget();
	return target->valid && (target->Position().magnitude() <= config.projectile_max_range * fixed(1.1));
}

inline const fixed SystemState::currentYaw() {
	return angleToStep * (stepperA.currentPosition() + stepperB.currentPosition()) / 2;
}

inline const fixed SystemState::currentPitch() {
	return angleToStep * (stepperA.currentPosition() - stepperB.currentPosition()) / 2;
}
