#include <stdint.h>
#include <Arduino.h>
#include "esp_timer.h"
#include "DptHelpers.h"

SystemState::SystemState() {
	stepperA = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
	stepperB = AccelStepper(motorInterfaceType, stepPinB, dirPinB);
	steppers.addStepper(stepperA);
	steppers.addStepper(stepperB);

	xMutex = xSemaphoreCreateMutex();
}

Target SystemState::currentTarget() {
	return target[selectedTarget];
}

void SystemState::updateTarget(Target& newTarget) {
	// Serial.printf("Target %i at %f by %f\n", newTarget.index, newTarget.Pitch(), newTarget.Yaw());
	target[newTarget.index] = newTarget;
	needTrackingUpdate = true;
}

void SystemState::setTarget(uint8_t index) {
	selectedTarget = index;
	needTrackingUpdate = true;
}

void SystemState::queueFire(uint8_t milliseconds)
{
}

void SystemState::queueLinger(uint8_t milliseconds)
{
}

void SystemState::processCommandQueue() {
	auto now = esp_timer_get_time();
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		if (!commandQueue.empty()){
			auto comm = commandQueue.top();
			while(now > comm->run_after) {
				comm->Execute(this);
				commandQueue.pop();
			}
		}
		xSemaphoreGive(xMutex);
	}
}

void SystemState::actualizeState() {
	actualizePosition();
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

		long moveA = delta_A - stepperA.currentPosition();
		long moveB = delta_B - stepperB.currentPosition();
		long distance = pow(moveA, 2) + pow(moveB, 2);

		if (distance <= 50)
		{
			return;
		}

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






bool Command::operator<(const Command &other) const
{
	return this->run_after < other.run_after;
}

Command::Command(int64_t run_after) : run_after(run_after)
{
	id = esp_timer_get_time();
}


void TargetSelection::Execute(SystemState &state)
{
}
TargetSelection::TargetSelection(uint8_t target_id, int speed, int64_t run_after) : Command(run_after), target_id(target_id), speed(speed)
{
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


double Target::Pitch(){
	if(!pitch) {
		pitch = atan(double(X_coord) / double(Y_coord)) * -180.0 / PI;
	}
	return pitch;
}
double Target::Yaw(){
	if(!yaw) {
		yaw = atan(double(Z_coord) / double(Distance())) * -180.0 / PI; // Height of default target - height of turret = angle to aim at (table height is 1320)	
	}
	return yaw;
}
long Target::Distance() {
	if (!distance) {
		distance = sqrt(pow(X_coord, 2) +  pow(Y_coord, 2));
	}
	return distance;
}


