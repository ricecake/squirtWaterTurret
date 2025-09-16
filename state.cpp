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

void SystemState::updateNearestTarget(const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin) {
	auto idx = fetchNearestTargetIdx(newPosition);
	updateTarget(idx, valid, newPosition, indifferenceMargin);
}

void SystemState::updateTarget(const uint8_t idx, const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin)
{
	// Serial.printf("saw %f %f %f %f/%f\n", float(newPosition.X_coord), float(newPosition.Y_coord), float(newPosition.Z_coord), float(newPosition.Pitch()), float(newPosition.Yaw()));
	bool doUpdate = true;
	if (indifferenceMargin > 0)
	{
		auto oldTarget = target[idx];
		auto oldTargetPos = oldTarget.Position();
		if (oldTargetPos) {
			// auto distance = newPosition - oldTargetPos;

			auto travelAngle = abs(oldTargetPos.angleTo(newPosition)) / angleToStep;
			doUpdate = (travelAngle) > indifferenceMargin;
			// auto yawDist = fixed(distance.yaw());
			// auto pitchDist = fixed(distance.pitch());
			// doUpdate = indifferenceMargin <= sqrt(pow(yawDist, 2), pow(pitchDist, 2))/angleToStep;

			// doUpdate = distance.magnitude() >= fixed(indifferenceMargin)/100 + oldTarget.Velocity().magnitude();
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

void SystemState::queueFire(uint8_t fireDuration)
{
	auto start = DynamicTimeInterval<fixed, std::milli>(5);
	auto end = DynamicTimeInterval<fixed, std::milli>(fireDuration) + start;

	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
	{
		commandQueue.push(new FireControl(true, start.microseconds()));
		commandQueue.push(new FireControl(false, end.microseconds()));
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
		// if (!newQueue.empty()) {
		// Serial.println("BEGIN QUEUE DUMP");
		// auto newQueue = commandQueue;
		// while(!newQueue.empty()) {
		// 	auto i = newQueue.top();
		// 	newQueue.pop();
		// 	Serial.printf("\t\tDUMP %lld: %lld %lld\n", now, i->run_after, i->id);
		// }
		// Serial.println("END QUEUE DUMP");
		// }
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
	// Serial.println("pos check");

	// Serial.println(target.valid);
	// Serial.println(target.X_coord);
	// Serial.println(target.Y_coord);
	// Serial.println(target.Z_coord);
	// Serial.printf("V: %i    loc: %f    %f\n", target.valid, float(target.Pitch()), float(target.Yaw()));
	if (needTrackingUpdate && target.valid)
	{
		// Serial.println("Updating Tracking");
		/*
		This might need to be some form of smoothing function that takes target positions and smooths them out into a motion path?
		Basically take multiple target positions over time, and try to match the targets velocity, and also their positiono.
		I think that's something that a pid controller does?
		Yes, pid controller.
		Pitch and yaw each get a controller, and it should output a movement speed for each motor.
		We should set the speed for each of them and use runSpeed to move at that velocity.
		It's inputs should be the current position in the respective dimension.
		*/

		auto aimpoint = target.interceptPosition();
		// auto position = target.Position();
		// Serial.printf("At %f %f %f %f/%f\n", float(position.X_coord), float(position.Y_coord), float(position.Z_coord), float(position.Pitch()), float(position.Yaw()));
		// Serial.printf("Aim %f %f %f %f/%f\n", float(aimpoint.X_coord), float(aimpoint.Y_coord), float(aimpoint.Z_coord), float(aimpoint.Pitch()), float(aimpoint.Yaw()));
		// auto cyaw = angleToStep * (stepperA.currentPosition() + stepperB.currentPosition()) / 2;
		// auto cpitch = angleToStep * (stepperA.currentPosition() - stepperB.currentPosition()) / 2;

		// Serial.printf("Curr %f %f\n", float(cpitch), float(cyaw));

		// Serial.printf("loc: %f    %f\n", float(target.Pitch()), float(target.Yaw()));
		auto pitch = long(min(max(aimpoint.Pitch(), -60), 60) / angleToStep);
		auto yaw = long(min(max(aimpoint.Yaw(), -70), 70) / angleToStep);

		// int delta_A = pitch + yaw;
		// int delta_B = yaw - pitch;

		int delta_A = yaw + pitch;
		int delta_B = pitch - yaw;

		// long moveA = delta_A - stepperA.currentPosition();
		// long moveB = delta_B - stepperB.currentPosition();
		// long distance = pow(moveA, 2) + pow(moveB, 2);

		// if (distance <= 50)
		// {
		// 	return;
		// }

		double iterMaxSpeed = trackingSpeed / double(0xFF) * maxSpeed * stepFraction;
		// long moveA = delta_A - stepperA.currentPosition();
		// long moveB = delta_B - stepperB.currentPosition();
		// long distance = sqrt(pow(moveA, 2) + pow(moveB, 2));
		// iterMaxSpeed *= distance / (distance + 1);

		// iterMaxSpeed *= iterMaxSpeed/maxSpeed * min(distance/float(400), float(1));
		// iterMaxSpeed = max(min(iterMaxSpeed, float(maxSpeed)), float(25));
		// iterMaxSpeed = min(iterMaxSpeed, float(maxSpeed));

		// Serial.printf("Moving to (%f, %f) [%i, %i] at %f via delta (%i, %i) -> %i\n", target.Pitch(), target.Yaw(), pitch, yaw, iterMaxSpeed, moveA, moveB, distance);

		long delta[2] = {
			delta_A,
			delta_B,
		};
		stepperA.setMaxSpeed(iterMaxSpeed);
		stepperB.setMaxSpeed(iterMaxSpeed);

		stepperA.moveTo(delta_A);
		stepperB.moveTo(delta_B);

		// steppers.moveTo(delta);
		needTrackingUpdate = false;
	}

	if (stepperA.distanceToGo() || stepperB.distanceToGo())
	{
		// steppers.run();
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

	// Serial.printf("At %f %f Want %f %f\n", pitch, yaw, target.Pitch(), target.Yaw());

	return sqrt(pow(yaw - aimpoint.Yaw(), 2) + pow(pitch - aimpoint.Pitch(), 2));
}

PositionVector SystemState::targetAimpoint()
{
	const auto target = currentTarget();
	return target.interceptPosition();
}