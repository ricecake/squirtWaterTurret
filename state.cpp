#include "utilities.h"
#include "state.h"
#include "firecontrol.h"
#include "target_selection.h"

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

void SystemState::updateTarget(Target &newTarget, uint16_t indifferenceMargin)
{
	bool doUpdate = true;
	if (indifferenceMargin > 0)
	{
		auto oldTarget = target[newTarget.index];
		auto distance = pow(newTarget.X_coord - oldTarget.X_coord, 2) + pow(newTarget.Y_coord - oldTarget.Y_coord, 2);
		indifferenceMargin += max(abs(newTarget.speed), abs(oldTarget.speed));
		doUpdate = distance >= indifferenceMargin;
		// if (doUpdate) {
		// 	Serial.println(distance);
		// 	Serial.println(newTarget.speed);
		// 	Serial.println(oldTarget.speed);
		// 	Serial.println(doUpdate? "Move" : "No Move");
		// }
	}

	if (doUpdate)
	{
		target[newTarget.index].Update(newTarget);
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
	auto start = milliseconds(5);
	auto end = milliseconds(fireDuration, start);

	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
	{
		commandQueue.push(new FireControl(true, start));
		commandQueue.push(new FireControl(false, end));
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

		auto pitch = long(min(max(target.Pitch(), -60.0), 60.0) / angleToStep);
		auto yaw = long(min(max(target.Yaw(), -70.0), 70.0) / angleToStep);

		int delta_A = pitch + yaw;
		int delta_B = yaw - pitch;

		// long moveA = delta_A - stepperA.currentPosition();
		// long moveB = delta_B - stepperB.currentPosition();
		// long distance = pow(moveA, 2) + pow(moveB, 2);

		// if (distance <= 50)
		// {
		// 	return;
		// }

		double iterMaxSpeed = trackingSpeed / double(0xFF) * maxSpeed * stepFraction;

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

long SystemState::targetTravelDistance()
{
	auto target = currentTarget();
	if (!target.valid)
	{
		return INT_MAX;
	}

	auto yaw = angleToStep * (stepperA.currentPosition() + stepperB.currentPosition()) / 2;
	auto pitch = angleToStep * (stepperA.currentPosition() - stepperB.currentPosition()) / 2;

	// Serial.printf("At %f %f Want %f %f\n", pitch, yaw, target.Pitch(), target.Yaw());

	return pow(yaw - target.Yaw(), 2) + pow(pitch - target.Pitch(), 2);
}
