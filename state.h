#pragma once

#include <functional>
#include <queue>
#include <stdint.h>

#ifdef ARDUINO
	#include "freertos/semphr.h"

	#include <AccelStepper.h>
#else
	#include "tests/mocks.h"
#endif

#include "command.h"
#include "fpm_adapter.hpp"
#include "target.h"
#include "vector.hpp"

using fixed = fixed_16_16;

class Command;

const inline auto CommandPointerComparator = [](const auto& left,
												const auto& right) {
	return left->run_after >= right->run_after;
};

/**
 * @brief Manages the overall state of the system.
 *
 * This class encapsulates all system components, including motors, targets,
 * and the command queue. It provides an interface for updating and controlling
 * the system's behavior.
 */
class SystemState {
private:
	const int motorInterfaceType = 1;  ///< Stepper motor driver interface type.

	const int stepPinA = 32;  ///< Step pin for stepper motor A.
	const int dirPinA = 33;   ///< Direction pin for stepper motor A.
	const int stepPinB = 25;  ///< Step pin for stepper motor B.
	const int dirPinB = 26;   ///< Direction pin for stepper motor B.
	const int firePin = 2;    ///< Pin for the firing mechanism.

public:
	const int stepFraction =
		16;  ///< Microstep fraction for the stepper motors.

	const int h_max = 500;    ///< Maximum horizontal position.
	const int v_max = 1000;   ///< Maximum vertical position.
	const int h_min = -500;   ///< Minimum horizontal position.
	const int v_min = -1000;  ///< Minimum vertical position.

	const fixed angleToStep{
		0.1125};  ///< Conversion factor from angle to motor steps.

public:
	AccelStepper stepperA;  ///< Stepper motor A instance.
	AccelStepper stepperB;  ///< Stepper motor B instance.

	SemaphoreHandle_t
		xMutex;  ///< Mutex for thread-safe access to shared resources.

public:
	bool cv_system_active = false;

private:
	bool moveState = true;   ///< Flag indicating if movement is enabled.
	bool fireState = false;  ///< Flag indicating the current firing state.
	bool needTrackingUpdate =
		false;  ///< Flag indicating if a tracking update is required.
	uint8_t trackingSpeed = 255;  ///< The speed for tracking movements.
	uint8_t selectedTarget =
		0;  ///< The index of the currently selected target.

private:
	PositionVector         targetAimpoint();
	std::array<Target, 32> target;
	std::array<Target, 3>
		radarTarget;  // Separate the two -- by default populate the radar
					  // target and prefer the target list if possible
	std::priority_queue<Command*, std::vector<Command*>,
						decltype(CommandPointerComparator)>
		commandQueue;  ///< Priority queue for pending commands.

public:
	SystemState();
	Target&          currentTarget();
	constexpr size_t size() { return target.size(); }
	void             updateTarget(const uint8_t idx, const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);

	/**
	 * @brief Updates a target by its ID.
	 * If the ID is not found, it tries to use an invalid target or the oldest
	 * one.
	 */
	inline void updateTargetById(const uint8_t id, const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0) {
		auto pred = [&](Target& item) { return item.id == id; };
		auto found = std::ranges::find_if(target, pred);

		if (found == target.end()) {
			auto pred = [&](Target& item) { return item.valid == false; };
			found = std::ranges::find_if(target, pred);
		}

		if (found == target.end()) {
			auto pred = [&](Target& item) { return item.seen; };
			found = std::ranges::min_element(target, std::ranges::less{}, pred);
		}

		updateTarget(found->index, valid, newPosition, indifferenceMargin);
		target[found->index].id = id;
	};

	void updateNearestTarget(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);
	void updateNearestTarget2d(const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin = 0);
	void setTarget(uint8_t index, uint8_t speed = 0xFF);
	void setFire(bool active);
	bool getFireState();
	void queueSelectTarget(uint8_t index, uint16_t milliseconds);
	void queueFire(uint16_t milliseconds);
	void queueLinger(uint8_t milliseconds);
	void processCommandQueue();
	void actualizeState();

	/**
	 * @brief Fetches a target by its index.
	 */
	inline Target& fetchTarget(const uint8_t idx) { return target[idx]; }

	/**
	 * @brief Finds the index of the nearest target in 3D space.
	 */
	inline uint8_t fetchNearestTargetIdx(const PositionVector& point) {
		auto distance = [&](Target& item) {
			auto pos = item.Position();
			return pow(pos.X_coord - point.X_coord, 2) +
				   pow(pos.Y_coord - point.Y_coord, 2) +
				   pow(pos.Z_coord - point.Z_coord, 2);
		};
		auto res =
			std::ranges::min_element(target, std::ranges::less{}, distance);
		return std::ranges::distance(target.begin(), res);
	}

	/**
	 * @brief Finds the index of the nearest target in 2D space (XY plane).
	 */
	inline uint8_t fetchNearestTarget2dIdx(const PositionVector& point) {
		auto distance = [&](Target& item) {
			auto pos = item.Position();
			return pow(pos.X_coord - point.X_coord, 2) +
				   pow(pos.Y_coord - point.Y_coord, 2);
		};
		auto res =
			std::ranges::min_element(target, std::ranges::less{}, distance);
		return std::ranges::distance(target.begin(), res);
	}

	fixed targetTravelDistance();

private:
	void actualizePosition();
	void actualizeFiring();
};
