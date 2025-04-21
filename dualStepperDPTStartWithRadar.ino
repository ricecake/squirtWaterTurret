#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <HardwareSerial.h>
#include "LD2450.h"
#include <vector>
#include <queue>

// Define pin connections
const int stepPinB = 32;
const int dirPinB = 33;
const int stepPinA = 25;
const int dirPinA = 26;

// Define motor limits
const int maxSpeed = 400;
const int acceleration = 120;

// Define other constants
const int stepFraction = 16;  // The microstep fraction

const int h_max = 500;
const int v_max = 1000;
const int h_min = -500;
const int v_min = -1000;


const float angleToStep = 360/200/1/16; // circle / steps per circle / gear ratio / step division

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper stepperA(motorInterfaceType, stepPinA, dirPinA);
AccelStepper stepperB(motorInterfaceType, stepPinB, dirPinB);

MultiStepper steppers;
HardwareSerial RadarSerial(1);
LD2450 ld2450;


struct MoveCmd {
	int H;
	int V;
	int S;
	int D;
	MoveCmd(int h = 0, int v = 0, int s = maxSpeed, int d = 50)
	  : H(h), V(v), S(s), D(d) {}
};

std::deque<MoveCmd> positions;

void initSpeedTest(){
	for (auto i = 25; i < maxSpeed*2; i+=25) {
		positions.push_back(MoveCmd(h_max, 0, i));
		positions.push_back(MoveCmd(h_min, 0, i));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 100));

		positions.push_back(MoveCmd(0, v_min, i));
		positions.push_back(MoveCmd(0, v_max, i));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 100));
	}
}
void initDemoTest(){
	positions.push_back(MoveCmd(h_max, 0, maxSpeed/4, 750));
	positions.push_back(MoveCmd(h_min, 0, maxSpeed/4, 750));
	positions.push_back(MoveCmd(0, 0, maxSpeed/4, 750));
	positions.push_back(MoveCmd(0, v_min, maxSpeed/4, 750));
	positions.push_back(MoveCmd(0, v_max, maxSpeed/4, 750));
	positions.push_back(MoveCmd(0, 0, maxSpeed/4, 1000));
	
	for (auto i = 0; i < 2; i++) {
		positions.push_back(MoveCmd(h_max, 0, maxSpeed, 100));
		positions.push_back(MoveCmd(h_min, 0, maxSpeed, 100));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
		positions.push_back(MoveCmd(0, v_min, maxSpeed, 100));
		positions.push_back(MoveCmd(0, v_max, maxSpeed, 100));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
	}
}

void initRangeTest() {
	positions.push_back(MoveCmd(h_max, v_max, maxSpeed/2));
	positions.push_back(MoveCmd(h_max, v_min, maxSpeed/2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(0, v_min, maxSpeed/2));
	positions.push_back(MoveCmd(0, v_max, maxSpeed/2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_min, v_max, maxSpeed/2));
	positions.push_back(MoveCmd(h_min, v_min, maxSpeed/2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_max, v_max, maxSpeed/2));
	positions.push_back(MoveCmd(h_min, v_max, maxSpeed/2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_min, 0, maxSpeed/2));
	positions.push_back(MoveCmd(h_max, 0, maxSpeed/2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));

	positions.push_back(MoveCmd(h_max, v_min, maxSpeed/2));
	positions.push_back(MoveCmd(h_min, v_min, maxSpeed/2));

	positions.push_back(MoveCmd(0, 0, maxSpeed, 500));
}

void initOscilateTest() {
	for (auto i = 0; i < 10; i++) {
		positions.push_back(MoveCmd(0, v_min, maxSpeed, 0));
		positions.push_back(MoveCmd(0, v_max, maxSpeed, 0));
	}

	for (auto i = 0; i < 10; i++) {
		positions.push_back(MoveCmd(h_min, 0, maxSpeed, 0));
		positions.push_back(MoveCmd(h_max, 0, maxSpeed, 0));
	}

}

void initTestData() {
	// initSpeedTest();
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));

	initRangeTest();
	positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));

	initOscilateTest();
	positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));

	initDemoTest();
	positions.push_back(MoveCmd(0, 0, maxSpeed, 2000));
}

void setup() {
	Serial.begin(9600);

	while (!Serial) {
		;  // wait for serial port to connect. Needed for native USB
	}

	RadarSerial.begin(256000, SERIAL_8N1, 16, 17);
	ld2450.begin(RadarSerial, false);


	if (!ld2450.waitForSensorMessage()) {
		Serial.println("SENSOR CONNECTION SEEMS OK");
	} else {
		Serial.println("SENSOR TEST: GOT NO VALID SENSORDATA - PLEASE CHECK CONNECTION!");
	}

	randomSeed(analogRead(0));
	// set the maximum speed, acceleration factor,
	// initial speed and the target position
	stepperA.setMaxSpeed(maxSpeed);
	stepperB.setMaxSpeed(maxSpeed);
	stepperA.setAcceleration(acceleration);
	stepperB.setAcceleration(acceleration);

	steppers.addStepper(stepperA);
	steppers.addStepper(stepperB);

	Serial.println("SETUP_FINISHED");


	initTestData();


	Serial.println("Ready!");

	delay(5000);
	Serial.println("Starting!");
}

long deltas[2];
MoveCmd last;

void findTarget() {}
void executeMove() {}
void executeIdle() {}

enum ControlState {
	Setup,
	Running,
	Homing,
	Idle,
};

ControlState currentState = Setup;

void loop() {
	switch (currentState) {
		case Setup:
		break;
		case Homing:
		break;
		case Idle:
		break;
	}
	// if(!positions.empty() && (stepperA.distanceToGo() || stepperB.distanceToGo())) {
	// 	Serial.println("Stopping");
	// 	stepperA.stop();
	// 	stepperB.stop();
	// 	return;
	// }

	if (!(stepperA.distanceToGo() || stepperB.distanceToGo())) {
		if (positions.empty()) {
			Serial.println("Target Check");
			const int sensor_got_valid_targets = ld2450.read();
			if (sensor_got_valid_targets > 0) {
				// GET THE DETECTED TARGETS
				for (int i = 0; i < sensor_got_valid_targets; i++)
				{
					LD2450::RadarTarget result_target = ld2450.getTarget(i);

					if (result_target.valid && result_target.speed > 0)
					{
						double x_offset = atan(double(result_target.x)/double(result_target.y))* 180.0 / PI;
						double y_offset = atan(double(1000)/double(result_target.distance))* 180.0 / PI;
						
						Serial.printf("Target %i at %f by %f, %i mm away going %i cm/s\n", i, x_offset, y_offset, result_target.distance, result_target.speed);

						MoveCmd newCmd;
						newCmd.H = min(max(int(x_offset / -0.1125), h_min), h_max);
						newCmd.V = min(max(int(y_offset / 0.1125), v_min), v_max);
						positions.push_back(newCmd);
					}
				}
			}
		}


		if (positions.empty()) {
			// return;
			if ((stepperA.currentPosition() == 0) && (stepperB.currentPosition() == 0)) {
				return;
			}

			Serial.println("Homeing");
			positions.push_back(MoveCmd(0, 0));

			// positions.push_back(MoveCmd(h_max, 0, 5, 1000));
			// positions.push_back(MoveCmd(h_min, 0, 5, 1000));
		}

		if (last.D > 0) {
			delay(last.D);
		}

		Serial.println("Execute Move");
		MoveCmd position = positions[0];

		positions.pop_front();

		int H = position.H;
		int V = position.V;
		int iterMaxSpeed = position.S > 0 ? position.S : maxSpeed;

		iterMaxSpeed *= stepFraction;

		int delta_A = H + V;
		int delta_B = V - H;

		Serial.printf("Moving to (%i, %i) [%f, %f] at %u via delta (%i, %i)  [%i, %i]\n", position.H, position.V, position.H*0.225, position.V*0.225, position.S, delta_A, delta_B, stepperA.currentPosition(), stepperB.currentPosition());

		deltas[0] = delta_A;
		deltas[1] = delta_B;
		stepperA.setMaxSpeed(iterMaxSpeed);
		stepperB.setMaxSpeed(iterMaxSpeed);
		steppers.moveTo(deltas);
		last = position;
	}
	steppers.run();
}