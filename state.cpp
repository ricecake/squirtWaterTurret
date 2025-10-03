#include <chrono>
#include <ratio>
#include "HardwareSerial.h"
#include "utilities.h"
#include "state.h"
#include "firecontrol.h"
#include "target_selection.h"
#include "fpm_adapter.hpp"
#include "aproximate_math.hpp"

SystemState::SystemState()
{
	stepperA = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
	stepperB = AccelStepper(motorInterfaceType, stepPinB, dirPinB);

	stepperA.setAcceleration(acceleration);
	stepperB.setAcceleration(acceleration);

	steppers.addStepper(stepperA);
	steppers.addStepper(stepperB);

	pinMode(firePin, OUTPUT);

	xMutex = xSemaphoreCreateMutex();
}

Target &SystemState::currentTarget()
{
	return target[selectedTarget];
}

void SystemState::updateNearestTarget(const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin)
{
	auto idx = fetchNearestTargetIdx(newPosition);
	updateTarget(idx, valid, newPosition, indifferenceMargin);
}

void SystemState::updateNearestTarget2d(const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin)
{
	auto idx = fetchNearestTarget2dIdx(newPosition);
	auto prev = fetchTarget(idx);
	newPosition.Z_coord = prev.Position().Z_coord;
	updateTarget(idx, valid, newPosition, indifferenceMargin);
}

void SystemState::updateTarget(const uint8_t idx, const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin)
{
	bool doUpdate = true;
	if (indifferenceMargin > 0)
	{
		auto oldTarget = target[idx];
		auto oldTargetPos = oldTarget.Position();
		if (oldTargetPos)
		{

			auto travelAngle = abs(oldTargetPos.angleTo(newPosition)) / angleToStep;
			doUpdate = (travelAngle) > indifferenceMargin;
		}
	}

	if (doUpdate)
	{
		// Need something that can indicate that this is a reduced dimension measurement, so we only update fields that are real
		target[idx].Update(newPosition);
		target[idx].valid = valid;
		needTrackingUpdate = true;
	}
}

void SystemState::setTarget(uint8_t index, uint8_t speed)
{
	selectedTarget = index;
	trackingSpeed = speed;
	needTrackingUpdate = true;
}

void SystemState::setFire(bool active)
{
	fireState = active;
}

bool SystemState::getFireState()
{
	return fireState;
}

void SystemState::queueFire(uint16_t fireDuration)
{
	auto start = DynamicTimeInterval<uint32_t, std::milli>(5);
	auto end = DynamicTimeInterval<uint32_t, std::milli>(fireDuration) + start;

	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
	{
		commandQueue.push(new FireControl(true, fireDuration, start.microseconds()));
		commandQueue.push(new FireControl(false, fireDuration, end.microseconds()));
		xSemaphoreGive(xMutex);
	}
}

void SystemState::queueLinger(uint8_t milliseconds)
{
}

void SystemState::queueSelectTarget(uint8_t index, uint16_t milliseconds)
{
	commandQueue.push(new TargetSelection(
		index, 0xFF, milliseconds * 1000));
}

void SystemState::processCommandQueue()
{
	auto now = esp_timer_get_time();
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
	{
		while (!commandQueue.empty())
		{
			auto comm = commandQueue.top();

			if (now <= comm->run_after)
			{
				break;
			}

			commandQueue.pop();
			comm->Execute(this);
			delete comm;
		}
		xSemaphoreGive(xMutex);
	}
}

void SystemState::actualizeState()
{
	actualizePosition();
	actualizeFiring();
}

void SystemState::actualizeFiring()
{
	digitalWrite(firePin, fireState ? HIGH : LOW);
}

void SystemState::actualizePosition()
{
	auto target = currentTarget();
	if (needTrackingUpdate && target.valid)
	{
		/*
			This might need to be some form of smoothing function that takes target positions and smooths them out into a motion path?
			Basically take multiple target positions over time, and try to match the targets velocity, and also their positiono.
			I think that's something that a pid controller does?
			Yes, pid controller.
			Pitch and yaw each get a controller, and it should output a movement speed for each motor.
			We should set the speed for each of them and use runSpeed to move at that velocity.
			It's inputs should be the current position in the respective dimension.

			Specifically, a pid controller on the quaternion of the firing angle with a kalman tracker of some sort for smoothing and prediction.
		*/

		auto aimpoint = target.interceptPosition();
		auto pitch = long(min(max(aimpoint.Pitch(), -60), 60) / angleToStep);
		auto yaw = long(min(max(aimpoint.Yaw(), -70), 70) / angleToStep);

		int delta_A = yaw + pitch;
		int delta_B = pitch - yaw;

		double iterMaxSpeed = trackingSpeed / double(0xFF) * maxSpeed * stepFraction;

		long delta[2] = {
			delta_A,
			delta_B,
		};
		stepperA.setMaxSpeed(iterMaxSpeed);
		stepperB.setMaxSpeed(iterMaxSpeed);

		stepperA.moveTo(delta_A);
		stepperB.moveTo(delta_B);

		needTrackingUpdate = false;
	}

	if (stepperA.distanceToGo() || stepperB.distanceToGo())
	{
		stepperA.run();
		stepperB.run();
	}
}

fixed SystemState::targetTravelDistance()
{
	auto target = currentTarget();
	if (!target.valid)
	{
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

PositionVector SystemState::targetAimpoint()
{
	const auto target = currentTarget();
	return target.interceptPosition();
}