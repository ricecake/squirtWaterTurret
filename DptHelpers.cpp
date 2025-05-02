#include "esp_timer.h"
#include <stdint.h>
#include <Arduino.h>
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
	if (!commandQueue.empty()){
		auto comm = commandQueue.top();
		while(now > comm->run_after) {
			comm->Execute(this);
			commandQueue.pop();
		}
	}
}

void SystemState::actualizeState() {
	actualizePosition();
}


void SystemState::actualizePosition()
{
	auto target = currentTarget();
	if(needTrackingUpdate && target.valid) {

		auto pitch = min(max(int(target.Pitch() / 0.1125), h_min), h_max);
		auto yaw   = min(max(int(target.Yaw() / 0.1125), v_min), v_max);

		int delta_A = pitch + yaw;
		int delta_B = yaw - pitch;

		long moveA = delta_A - stepperA.currentPosition();
		long moveB = delta_B - stepperB.currentPosition();
		long distance = (moveA * moveA) + (moveB * moveB);
		// Serial.printf("Moving to (%i, %i) [%f, %f] at %u via delta (%i, %i)  [%i, %i]\n", H, V, H*0.225, V*0.225, S, delta_A, delta_B, stepperA.currentPosition(), stepperB.currentPosition());

		if (distance <= 50)
		{
			// Serial.printf("Skipping [%i %i] [%i %i] [%i %i] [%i %i])\n", H, V, delta_A, delta_B, stepperA.currentPosition(), stepperB.currentPosition(), moveA, moveB);
			return;
		}

		float iterMaxSpeed = trackingSpeed/double(255) * maxSpeed;

		// iterMaxSpeed *= iterMaxSpeed/maxSpeed * min(distance/float(400), float(1));
		// iterMaxSpeed = max(min(iterMaxSpeed, float(maxSpeed)), float(25));
		// iterMaxSpeed = min(iterMaxSpeed, float(maxSpeed));

		// Serial.printf("Moving to (%i, %i) [%f, %f] at %f via delta (%i, %i) -> %i\n", H, V, H * angleToStep, V * angleToStep, iterMaxSpeed, moveA, moveB, distance);

		iterMaxSpeed *= stepFraction;

	/*
		// This might need to be some form of smoothing function that takes target positions and smooths them out into a motion path?
			Basically take multiple target positions over time, and try to match the targets velocity, and also their positiono.
			I think that's something that a pid controller does?
			Yes, pid controller.  
			Pitch and yaw each get a controller, and it should output a movement speed for each motor.
			We should set the speed for each of them and use runSpeed to move at that velocity.
			It's inputs should be the current position in the respective dimension.
	*/

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


