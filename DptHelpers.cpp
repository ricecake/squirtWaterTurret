#include "HardwareSerial.h"
#include <climits>
#include "esp32-hal-gpio.h"
#include <stdint.h>
#include <Arduino.h>
#include "esp_timer.h"
#include "DptHelpers.h"

SystemState::SystemState() {
	stepperA = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
	stepperB = AccelStepper(motorInterfaceType, stepPinB, dirPinB);
	steppers.addStepper(stepperA);
	steppers.addStepper(stepperB);

	pinMode(firePin, OUTPUT);

	xMutex = xSemaphoreCreateMutex();
}

Target& SystemState::currentTarget() {
	return target[selectedTarget];
}

void SystemState::updateTarget(Target& newTarget, uint16_t indifferenceMargin) {
	bool doUpdate = true;
	if (indifferenceMargin > 0) {
		auto oldTarget = target[newTarget.index];
		auto distance = pow(newTarget.X_coord - oldTarget.X_coord, 2) + pow(newTarget.Y_coord - oldTarget.Y_coord, 2);
		doUpdate = distance >= indifferenceMargin;
		if (doUpdate) {
			Serial.println(distance);
			Serial.println(doUpdate? "Move" : "No Move");
		}
	}

	if (doUpdate) {
		target[newTarget.index].Update(newTarget);
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

void SystemState::queueFire(uint8_t fireDuration)
{
	auto start = milliseconds(5);
	auto end   = milliseconds(fireDuration, start);

	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		commandQueue.push(new FireControl(true, start));
		commandQueue.push(new FireControl(false, end));
		xSemaphoreGive(xMutex);
	}
}

void SystemState::queueLinger(uint8_t milliseconds)
{
}

void SystemState::queueSelectTarget(uint8_t index, uint16_t milliseconds) {
	commandQueue.push(new TargetSelection(
		index, 0xFF, milliseconds*1000
	));
}

void SystemState::processCommandQueue() {
	auto now = esp_timer_get_time();
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
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
		while(!commandQueue.empty()) {
			auto comm = commandQueue.top();

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

void SystemState::actualizeState() {
	actualizePosition();
	actualizeFiring();
}

void SystemState::actualizeFiring() {
	digitalWrite(firePin, fireState ? HIGH : LOW);
}


void SystemState::actualizePosition()
{
	auto target = currentTarget();
	if(needTrackingUpdate && target.valid) {
		/*
		This might need to be some form of smoothing function that takes target positions and smooths them out into a motion path?
		Basically take multiple target positions over time, and try to match the targets velocity, and also their positiono.
		I think that's something that a pid controller does?
		Yes, pid controller.  
		Pitch and yaw each get a controller, and it should output a movement speed for each motor.
		We should set the speed for each of them and use runSpeed to move at that velocity.
		It's inputs should be the current position in the respective dimension.
		*/

		auto pitch = long(min(max(target.Pitch(), -60.0), 60.0)/angleToStep);
		auto yaw   = long(min(max(target.Yaw(), -70.0), 70.0)/angleToStep);

		int delta_A = pitch + yaw;
		int delta_B = yaw - pitch;

		// long moveA = delta_A - stepperA.currentPosition();
		// long moveB = delta_B - stepperB.currentPosition();
		// long distance = pow(moveA, 2) + pow(moveB, 2);

		// if (distance <= 50)
		// {
		// 	return;
		// }

		double iterMaxSpeed = trackingSpeed/double(0xFF) * maxSpeed * stepFraction;

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
		steppers.moveTo(delta);
	}

	if (stepperA.distanceToGo() || stepperB.distanceToGo()) {
		steppers.run();
	}
}

long SystemState::targetTravelDistance() {
	auto target = currentTarget();
	if(!target.valid) {
		return INT_MAX;
	}

	auto yaw = angleToStep*(stepperA.currentPosition() + stepperB.currentPosition())/2;
	auto pitch = angleToStep*(stepperA.currentPosition() - stepperB.currentPosition())/2;

	// Serial.printf("At %f %f Want %f %f\n", pitch, yaw, target.Pitch(), target.Yaw());

	return pow(yaw - target.Yaw(), 2) + pow(pitch - target.Pitch(), 2);
}






Command::Command(int64_t run_after)
{
	id = esp_timer_get_time();
	this->run_after = id + run_after;
}


void TargetSelection::Execute(SystemState* state)
{
	state->setTarget(target_id, speed);
	auto currTarget = state->currentTarget();

	uint16_t timeout = 0;
	if (currTarget.valid) {
		timeout = 1*1000;
	}

	state->queueSelectTarget(((target_id+1)%4), timeout);
}
TargetSelection::TargetSelection(uint8_t target_id, int speed, int64_t run_after) : Command(run_after), target_id(target_id), speed(speed)
{
}

FireControl::FireControl(bool active, int64_t run_after) : Command(run_after), active(active) {
}
void FireControl::Execute(SystemState* state) {
	state->setFire(active);

	Target& curr = state->currentTarget();
	curr.IncrementAction();
}
















Target::Target() : index(0), X_coord(0), Y_coord(0), Z_coord(0), speed(0), valid(false)
{
	seen = esp_timer_get_time();
}

Target::Target(uint8_t index, long X, long Y, long speed, bool valid) : index(index), X_coord(X), Y_coord(Y), speed(speed), valid(valid)
{
	seen = esp_timer_get_time();
}

Target::Target(uint8_t index, long X, long Y, long Z, long speed, bool valid) : index(index), X_coord(X), Y_coord(Y), Z_coord(Z), speed(speed), valid(valid)
{
	seen = esp_timer_get_time();
}

void Target::Update(Target & updated) {
	valid = updated.valid;
	seen  = updated.seen;

	if (updated.X_coord) {
		X_coord = updated.X_coord;
	}
	if (updated.Y_coord) {
		Y_coord = updated.Y_coord;
	}
	if (updated.Z_coord) {
		Z_coord = updated.Z_coord;
	}
	if (updated.speed) {
		speed = updated.speed;
	}

	if (updated.X_coord || updated.Y_coord) {
		distance = updated.distance;
		pitch = updated.pitch;
		yaw = updated.yaw;
	}
}


double Target::Pitch(){
	if(!pitch) {
		pitch = atan(double(X_coord) / double(Y_coord)) * -180.0 / PI;
	}
	return pitch;
}
double Target::Yaw(){
	if(!yaw) {
		yaw = atan(double(1320-Z_coord) / double(Distance())) * 180.0 / PI; // Height of default target - height of turret = angle to aim at (table height is 1320)	
	}
	return yaw;
}
long Target::Distance() {
	if (!distance) {
		distance = sqrt(pow(X_coord, 2) +  pow(Y_coord, 2));
	}
	return distance;
}

int64_t Target::timeSinceLastAction() {
	auto now = esp_timer_get_time();
	return now - last_action;
}

bool Target::actionIdleExceeds(int64_t limit) {
	return timeSinceLastAction() >= limit;
}

void Target::IncrementAction() {
	last_action = esp_timer_get_time();
}

