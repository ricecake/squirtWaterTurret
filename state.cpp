/**
 * @file state.cpp
 * @brief Implements the SystemState class for managing the turret's state.
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
 * Initializes the stepper motors with their corresponding pins and interface type.
 * Sets the default acceleration for the motors. Configures the firing pin as an output.
 * Creates a mutex for thread-safe access to shared state.
 * Sets the initial target source to STATIC and initializes the static target's position.
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
 * @brief Returns the current target array based on the active target source.
 *
 * This function provides a view (as a `std::span`) into the appropriate target
 * buffer (`cvTarget`, `radarTarget`, or `staticTarget`) depending on the value
 * of `target_source`.
 *
 * @return A `std::span<Target>` referencing the active target buffer.
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
 * Based on the active target source and the `selectedTarget` index, this function
 * returns the specific target object that the system should be aiming at.
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
 * @param index The index of the target to select from the current target array.
 * @param speed The speed at which the motors should track the target.
 */
void SystemState::setTarget(uint8_t index, uint8_t speed) {
	selectedTarget = index;
	trackingSpeed = speed;
	needTrackingUpdate = true;
}

/**
 * @brief Sets the firing state of the turret.
 * @param active `true` to activate the firing mechanism, `false` to deactivate it.
 */
void SystemState::setFire(bool active) {
	fireState = active;
}

/**
 * @brief Enables or disables motor movement.
 * @param active `true` to enable movement, `false` to disable.
 */
void SystemState::setMove(bool active) {
	moveState = active;
}

/**
 * @brief Gets the current firing state.
 * @return `true` if the turret is firing, `false` otherwise.
 */
bool SystemState::getFireState() {
	return fireState;
}

/**
 * @brief Gets the current movement state.
 * @return `true` if motor movement is enabled, `false` otherwise.
 */
bool SystemState::getMoveState() {
	return moveState;
}

/**
 * @brief Queues commands to start and stop firing.
 *
 * This function adds two `FireControl` commands to the command queue: one to
 * start firing after a short delay, and another to stop firing after the specified
 * duration has elapsed.
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
 * This adds a `LingerCommand` to the queue, which can be used to introduce a
 * delay in a sequence of commands.
 *
 * @param milliseconds The duration of the linger in milliseconds.
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
 * @param milliseconds The delay in milliseconds before the selection occurs.
 */
void SystemState::queueSelectTarget(uint8_t index, uint16_t milliseconds) {
	commandQueue.push(new TargetSelection(index, 0xFF, milliseconds * 1000));
}

/**
 * @brief Updates the system's configuration parameters from a `Config` message.
 *
 * This function takes a pointer to a deserialized `Config` message and updates
 * the corresponding `config` members and motor settings.
 *
 * @param config A pointer to the `cerializer::Config` object.
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
 * This method iterates through the `commandQueue` and executes any command
 * whose `run_after` timestamp is in the past. It is thread-safe, using a mutex
 * to protect the queue.
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
 * This is a high-level update function that should be called repeatedly in the main
 * loop. It orchestrates the updates for both position (motors) and firing.
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
 * the required pitch and yaw to aim at the target's intercept position. It then
 * converts these angles to motor steps and commands the steppers to move.
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
 * @brief Calculates the angular distance the turret must travel to aim at the current target.
 *
 * This function determines the angle between the current turret orientation and the
 * calculated aimpoint for the current target.
 *
 * @return The travel distance as a fixed-point number representing the angle in radians.
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
 * @brief Calculates the required aimpoint to hit the current target.
 *
 * This is a convenience function that retrieves the current target and returns its
 * calculated intercept position.
 *
 * @return The calculated aimpoint as a `PositionVector`.
 */
PositionVector SystemState::targetAimpoint() {
	const auto target = currentTarget();
	return target.interceptPosition();
}