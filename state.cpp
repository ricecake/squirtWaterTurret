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

SystemState::SystemState(): staticTarget(0, true, PositionVector(0, 0.01, 0), VelocityVector(0, 0, 0)) {
	stepperA = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
	stepperB = AccelStepper(motorInterfaceType, stepPinB, dirPinB);

	stepperA.setAcceleration(acceleration);
	stepperB.setAcceleration(acceleration);

	pinMode(firePin, OUTPUT);

	xMutex = xSemaphoreCreateMutex();

	target_source = TargetSource::STATIC;
	auto p = staticTarget.Position();
	p.Z_coord = config.turret_height;
	staticTarget.Update(p);

	selectedTarget = &staticTarget;
}

/// @brief Return the current target array, based on which target system is active.
/// @return a span of targets, referencing the correct target buffer.
std::span<Target> SystemState::currentTargetArray() {
	switch (target_source) {
	case TargetSource::CV:
		return std::span(cvTarget.begin(), cvTarget.end());
	case TargetSource::RADAR:
		return std::span(radarTarget.begin(), radarTarget.end());
	case TargetSource::STATIC:
	default:
		return std::span(&staticTarget, 1);
	}
}

Target& SystemState::currentTarget() {
	return *selectedTarget;
}

void SystemState::setTarget(TargetSource source, uint8_t index, uint8_t speed) {
	if (source != target_source) {
		return; // No-op because we've changed sources
	}

	switch (target_source) {
	case TargetSource::CV:
		selectedTarget = &cvTarget[index];
		break;
	case TargetSource::RADAR:
		selectedTarget = &radarTarget[index];
		break;
	case TargetSource::STATIC:
		selectedTarget = &staticTarget;
		break;
	default:
		return;
	}

	trackingSpeed = speed;
	needTrackingUpdate = true;
}

void SystemState::setFire(bool active) {
	fireState = active;
}

void SystemState::setMove(bool active) {
	moveState = active;
}

void SystemState::setStrategy(TurretStrategy strategy) {
	this->strategy = strategy;
}

void SystemState::setStance(TurretStance stance) {
	this->stance = stance;
}

bool SystemState::getFireState() {
	return fireState;
}

bool SystemState::getMoveState() {
	return moveState;
}

void SystemState::queueFire(uint16_t fireDuration) {
	commandQueue.addCommand<FireControl>(true, fireDuration, 0);
}

void SystemState::queueCeaseFire(uint16_t fireDuration) {
	auto end = DynamicTimeInterval<uint32_t, std::milli>(fireDuration);
	commandQueue.addCommand<FireControl>(false, fireDuration, end.microseconds());
}

void SystemState::queueSelectTarget(TargetSource source, uint8_t index, uint16_t milliseconds) {
	commandQueue.addCommand<TargetSelection>(source, index, 0xFF, milliseconds * 1000);
}

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
 */
void SystemState::processCommandQueue() {
	commandQueue.process(this);
}

/**
 * @brief Updates the physical state of the system to match the desired state.
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

PositionVector SystemState::targetAimpoint() {
	const auto target = currentTarget();
	return target.interceptPosition();
}