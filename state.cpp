#include "state.h"

#include <algorithm>
#include <chrono>
#include <ratio>

#include "aproximate_math.hpp"
#include "firecontrol.h"
#include "fpm_adapter.hpp"
#include "target_selection.h"
#include "utilities.h"

#ifdef ARDUINO
	#include "HardwareSerial.h"
#else
	#include "tests/mocks.h"
#endif

SystemState::SystemState() {
#ifdef ARDUINO
	stepperA = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
	stepperB = AccelStepper(motorInterfaceType, stepPinB, dirPinB);

	stepperA.setAcceleration(acceleration);
	stepperB.setAcceleration(acceleration);

	pinMode(firePin, OUTPUT);

	xMutex = xSemaphoreCreateMutex();
#endif
}

Target& SystemState::currentTarget() {
	return target[selectedTarget];
}

void SystemState::updateTarget(const auto& targetArray, const uint8_t idx, const bool valid, PositionVector& newPosition, const uint16_t indifferenceMargin) {
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
		// Need something that can indicate that this is a reduced dimension measurement, so we only update fields that are real
		targetArray[idx].Update(newPosition);
		targetArray[idx].valid = valid;
		needTrackingUpdate = true;
	}
}

void SystemState::setTarget(uint8_t index, uint8_t speed) {
	selectedTarget = index;
	trackingSpeed = speed;
	needTrackingUpdate = true;
}

void SystemState::setFire(bool active) {
	fireState = active;
}

void SystemState::setMove(bool active) {
	moveState = active;
}

bool SystemState::getFireState() {
	return fireState;
}

bool SystemState::getMoveState() {
	return moveState;
}

void SystemState::queueFire(uint16_t fireDuration) {
	auto start = DynamicTimeInterval<uint32_t, std::milli>(5);
	auto end = DynamicTimeInterval<uint32_t, std::milli>(fireDuration) + start;

	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		commandQueue.push(new FireControl(true, fireDuration, start.microseconds()));
		commandQueue.push(new FireControl(false, fireDuration, end.microseconds()));
		xSemaphoreGive(xMutex);
	}
}

void SystemState::queueLinger(uint8_t milliseconds) {
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		commandQueue.push(new LingerCommand(milliseconds * 1000));
		xSemaphoreGive(xMutex);
	}
}

void SystemState::queueSelectTarget(uint8_t index, uint16_t milliseconds) {
	commandQueue.push(new TargetSelection(
		index,
		0xFF,
		milliseconds * 1000));
}

/**
 * @brief Processes the command queue, executing any commands that are due.
 */
void SystemState::processCommandQueue() {
	auto now = esp_timer_get_time();
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		while (!commandQueue.empty()) {
			auto comm = commandQueue.top();

			// Since the queue is sorted by execution time, we can stop when we find a command that is not yet due.
			if (now <= comm->run_after) {
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

	auto cos_alpha = sin(pitch) * sin(aimpoint.Pitch()) +
					 cos(pitch) * cos(aimpoint.Pitch()) * cos(delta_yaw);

	return acos(cos_alpha);
}

PositionVector SystemState::targetAimpoint() {
	const auto target = currentTarget();
	return target.interceptPosition();
}