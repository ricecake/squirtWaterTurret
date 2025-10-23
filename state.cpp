/**
 * @file state.cpp
 * @brief Implements the SystemState class for managing the turret's state.
 *
 * This file contains the definitions for the methods of the SystemState class,
 * which is responsible for initializing and controlling the turret's hardware,
 * managing targets, and processing commands.
 */
#include "state.h"

#ifdef ARDUINO
	#include "HardwareSerial.h"
#endif
#include <algorithm>
#include <chrono>
#include <ratio>

#include "aproximate_math.hpp"
#include "firecontrol.h"
#include "fpm_adapter.hpp"
#include "target.h"
#include "target_selection.h"
#include "utilities.h"

/**
 * @brief Constructs a new SystemState object.
 *
 * Initializes the stepper motors with their pins and acceleration, sets up the
 * firing pin, creates a mutex for thread safety, and sets the default target
 * source to STATIC. It also initializes the static target's position.
 */
SystemState::SystemState() : staticTarget(0, true, PositionVector(0, 0.01, 0), VelocityVector(0, 0, 0)) {
	stepperA = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
	stepperB = AccelStepper(motorInterfaceType, stepPinB, dirPinB);

	stepperA.setAcceleration(acceleration);
	stepperB.setAcceleration(acceleration);

	pinMode(firePin, OUTPUT);

	xMutex = xSemaphoreCreateMutex();

	target_source = cerializer::TargetSource::STATIC;
	auto p = staticTarget.Position();
	p.Z_coord = config.turret_height;
	staticTarget.Update(p);
}

/**
 * @brief Returns the current target array based on the active target system.
 *
 * This function provides a view into the appropriate target buffer (CV, Radar, or Static)
 * depending on the `target_source` state.
 *
 * @return A `std::span` of `Target` objects, referencing the active target buffer.
 */
std::span<Target> SystemState::currentTargetArray() {
	switch (target_source) {
	case cerializer::TargetSource::CV:
		return std::span(cvTarget.begin(), cvTarget.end());
	case cerializer::TargetSource::RADAR:
		return std::span(radarTarget.begin(), radarTarget.end());
	case cerializer::TargetSource::STATIC:
	default:
		return std::span(&staticTarget, 1);
	}
}

/**
 * @brief Returns a reference to the currently selected target.
 *
 * This function provides direct access to the `Target` object that the system is
 * currently aiming at, based on the `target_source` and `selectedTarget` index.
 *
 * @return A reference to the current `Target` object.
 */
Target& SystemState::currentTarget() {
	switch (target_source) {
	case cerializer::TargetSource::CV:
		return cvTarget[selectedTarget];
	case cerializer::TargetSource::RADAR:
		return radarTarget[selectedTarget];
	case cerializer::TargetSource::STATIC:
	default:
		return staticTarget;
	}
}

/**
 * @brief Sets the active target and tracking speed.
 *
 * This function updates the `selectedTarget` index and `trackingSpeed`, and
 * sets a flag to indicate that the motor positions need to be updated.
 *
 * @param index The index of the new target in the current target array.
 * @param speed The speed at which the motors should track the target.
 */
void SystemState::setTarget(uint8_t index, uint8_t speed) {
	selectedTarget = index;
	trackingSpeed = speed;
	needTrackingUpdate = true;
}

/**
 * @brief Activates or deactivates the firing mechanism.
 * @param active The desired state of the firing mechanism (true for active, false for inactive).
 */
void SystemState::setFire(bool active) {
	fireState = active;
}

/**
 * @brief Enables or disables motor movement.
 * @param active The desired state of motor movement (true for enabled, false for disabled).
 */
void SystemState::setMove(bool active) {
	moveState = active;
}

/**
 * @brief Gets the current state of the firing mechanism.
 * @return `true` if firing is active, `false` otherwise.
 */
bool SystemState::getFireState() {
	return fireState;
}

/**
 * @brief Gets the current state of motor movement.
 * @return `true` if movement is enabled, `false` otherwise.
 */
bool SystemState::getMoveState() {
	return moveState;
}

/**
 * @brief Queues commands to start and stop firing.
 *
 * This function pushes two `FireControl` commands onto the command queue: one to
 * start firing after a short delay, and another to stop firing after the specified duration.
 *
 * @param fireDuration The duration in milliseconds for which to fire.
 */
void SystemState::queueFire(uint16_t fireDuration) {
	auto start = DynamicTimeInterval<uint32_t, std::milli>(5);
	auto end = DynamicTimeInterval<uint32_t, std::milli>(fireDuration) + start;

	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		commandQueue.push(new FireControl(true, fireDuration, start.microseconds()));
		commandQueue.push(new FireControl(false, fireDuration, end.microseconds()));
		xSemaphoreGive(xMutex);
	}
}

/**
 * @brief Queues a command to do nothing for a specified duration.
 *
 * This is useful for creating pauses in a sequence of commands.
 *
 * @param milliseconds The duration of the pause in milliseconds.
 */
void SystemState::queueLinger(uint8_t milliseconds) {
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		commandQueue.push(new LingerCommand(milliseconds * 1000));
		xSemaphoreGive(xMutex);
	}
}

/**
 * @brief Queues a command to select a new target after a delay.
 * @param index The index of the target to select.
 * @param milliseconds The delay in milliseconds before selecting the target.
 */
void SystemState::queueSelectTarget(uint8_t index, uint16_t milliseconds) {
	commandQueue.push(new TargetSelection(index, 0xFF, milliseconds * 1000));
}

/**
 * @brief Updates the system's configuration from a `Config` message.
 *
 * This function takes a `cerializer::Config` object, typically received over a
 * serial link, and updates the system's runtime parameters, including projectile speed,
 * turret height, and motor speed and acceleration.
 *
 * @param config A pointer to the `cerializer::Config` object containing the new settings.
 */
void SystemState::updateConfig(cerializer::Config* config) {
	this->config.projectile_speed = fixed(config->projectile_speed);
	this->config.turret_height = fixed(config->turret_height);
	stepperA.setMaxSpeed(config->max_speed);
	stepperA.setAcceleration(config->acceleration);
	stepperB.setMaxSpeed(config->max_speed);
	stepperB.setAcceleration(config->acceleration);
}

/**
 * @brief Processes the command queue, executing any commands that are due.
 *
 * This function checks the command queue for any commands whose execution time
 * has passed. It executes them in order of priority and removes them from the queue.
 * This should be called repeatedly in the main loop.
 */
void SystemState::processCommandQueue() {
	auto now = esp_timer_get_time();
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		while (!commandQueue.empty()) {
			auto comm = commandQueue.top();

			// Since the queue is sorted by execution time, we can stop when we find a command that is not yet due.
			if (now < comm->run_after) {
				break;
			}

			commandQueue.pop();
			comm->Execute(this);
			delete comm;
		}
		xSemaphoreGive(xMutex);
	}
}

/**
 * @brief Updates the physical state of the system to match the desired state.
 *
 * This function calls the `actualizePosition` and `actualizeFiring` methods to
 * ensure the hardware's state reflects the system's internal state. This should
 * be called repeatedly in the main loop.
 */
void SystemState::actualizeState() {
	actualizePosition();
	actualizeFiring();
}

/**
 * @brief Activates or deactivates the firing pin based on the current fire state.
 */
void SystemState::actualizeFiring() {
	digitalWrite(firePin, fireState ? HIGH : LOW);
}

/**
 * @brief Updates the motor positions to track the current target.
 *
 * If movement is enabled and a tracking update is needed, this function calculates
 * the required pitch and yaw to hit the target's intercept position, converts these
 * angles to motor steps, and commands the motors to move. It also continuously calls
 * the `run()` method for the steppers to ensure smooth movement.
 */
void SystemState::actualizePosition() {
	if (!moveState) {
		return;
	}

	auto target = currentTarget();
	if (needTrackingUpdate && target.valid) {
		// Calculate the aimpoint using the target's intercept position
		auto aimpoint = target.interceptPosition();
		auto pitch = long(std::min(std::max(aimpoint.Pitch(), fixed(-60)), fixed(60)) / angleToStep);
		auto yaw = long(std::min(std::max(aimpoint.Yaw(), fixed(-70)), fixed(70)) / angleToStep);

		// Convert pitch and yaw to motor steps
		int delta_A = yaw + pitch;
		int delta_B = pitch - yaw;

		// Set motor speed based on tracking speed
		double iterMaxSpeed = trackingSpeed / double(0xFF) * maxSpeed * stepFraction;

		stepperA.setMaxSpeed(iterMaxSpeed);
		stepperB.setMaxSpeed(iterMaxSpeed);

		// Move motors to the new target position
		stepperA.moveTo(delta_A);
		stepperB.moveTo(delta_B);

		needTrackingUpdate = false;
	}

	// Continuously run the motors to move towards the target
	if (stepperA.distanceToGo() || stepperB.distanceToGo()) {
		stepperA.run();
		stepperB.run();
	}
}

/**
 * @brief Calculates the angular distance to the current target's aimpoint.
 *
 * This function determines the total angle the turret needs to turn to face the
 * calculated intercept position of the current target.
 *
 * @return The angular distance as a fixed-point number. Returns `INT_MAX` if the target is invalid.
 */
fixed SystemState::targetTravelDistance() {
	auto target = currentTarget();
	if (!target.valid) {
		return INT_MAX;
	}
	auto aimpoint = target.interceptPosition();

	auto yaw = angleToStep * (stepperA.currentPosition() + stepperB.currentPosition()) / 2;
	auto pitch = angleToStep * (stepperA.currentPosition() - stepperB.currentPosition()) / 2;

	auto delta_yaw = aimpoint.Yaw() - yaw;

	auto cos_alpha = sin(pitch) * sin(aimpoint.Pitch()) + cos(pitch) * cos(aimpoint.Pitch()) * cos(delta_yaw);

	return acos(cos_alpha);
}

/**
 * @brief DEPRECATED - This function is not used.
 * @return The calculated position vector of the aimpoint.
 */
PositionVector SystemState::targetAimpoint() {
	const auto target = currentTarget();
	return target.interceptPosition();
}