#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <HardwareSerial.h>
#include "LD2450.h"
#include <vector>
#include <queue>

// Define pin connections
const int stepPinA = 32;
const int dirPinA = 33;
const int stepPinB = 25;
const int dirPinB = 26;

// Define motor limits
const int maxSpeed = 300;
const int acceleration = 120;

// Define other constants
const int stepFraction = 16;  // The microstep fraction
// const int h_max = 100;
// const int v_max = 400;
// const int h_min = -100;
// const int v_min = -400;

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

	// positions.push_back(MoveCmd(h_min, 0, 10, 250));
	// positions.push_back(MoveCmd(h_max, 0, 10, 250));
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
	// positions.push_back(MoveCmd(0, v_min, 10, 250));
	// positions.push_back(MoveCmd(0, v_max, 10, 250));
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 250));

	// positions.push_back(MoveCmd(h_max, v_max, 250));
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
	// positions.push_back(MoveCmd(h_max, v_min, 250));
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
	// positions.push_back(MoveCmd(h_min, v_min, 250));
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 250));
	// positions.push_back(MoveCmd(h_min, v_max, 250));
	// positions.push_back(MoveCmd(0, 0, maxSpeed, 250));


	// for (auto i =0; i < 10; i++){
	// positions.push_back(MoveCmd(h_min, 0, 100));
	// positions.push_back(MoveCmd(h_max, 0, 100));
	// }

	// for (auto i =0; i < 10; i++){
	// positions.push_back(MoveCmd(0, v_min, 150));
	// positions.push_back(MoveCmd(0, v_max, 150));
	// }

	// for (auto i =0; i < 100; i++){
	// positions.push_back(MoveCmd(h_max, v_max));
	// positions.push_back(MoveCmd(h_max, 0));
	// positions.push_back(MoveCmd(h_max, v_min));
	// positions.push_back(MoveCmd(0, v_min));
	// positions.push_back(MoveCmd(0, 0));
	// positions.push_back(MoveCmd(0, v_max));
	// positions.push_back(MoveCmd(h_min, v_max));
	// positions.push_back(MoveCmd(h_min, 0));
	// positions.push_back(MoveCmd(h_min, v_min));

	// positions.push_back(MoveCmd(h_max, v_max));
	// positions.push_back(MoveCmd(0, v_max));
	// positions.push_back(MoveCmd(h_min, v_max));

	// positions.push_back(MoveCmd(h_min, 0));
	// positions.push_back(MoveCmd(0, 0));
	// positions.push_back(MoveCmd(h_max, 0));

	// positions.push_back(MoveCmd(h_max, v_min));
	// positions.push_back(MoveCmd(0, v_min));
	// positions.push_back(MoveCmd(h_min, v_min));

	// }

	// for (auto i = 0; i < 10; i++) {
	// positions.push_back(MoveCmd(0, v_min, 89));
	// positions.push_back(MoveCmd(0, v_max, 89));
	// }

	positions.push_back(MoveCmd(h_max, 0, 30, 750));
	positions.push_back(MoveCmd(h_min, 0, 30, 750));
	positions.push_back(MoveCmd(0, 0, 30, 750));
	positions.push_back(MoveCmd(0, v_min, 30, 750));
	positions.push_back(MoveCmd(0, v_max, 30, 750));
	positions.push_back(MoveCmd(0, 0, 30, 5000));
	
	for (auto i = 0; i < 2; i++) {
		positions.push_back(MoveCmd(h_max, 0, maxSpeed, 100));
		positions.push_back(MoveCmd(h_min, 0, maxSpeed, 100));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 100));
		positions.push_back(MoveCmd(0, v_min, maxSpeed, 100));
		positions.push_back(MoveCmd(0, v_max, maxSpeed, 100));
		positions.push_back(MoveCmd(0, 0, maxSpeed, 50));
	}

	// positions.push_back(MoveCmd(0, 100*v_max, 150, 100));
	// positions.push_back(MoveCmd(0, 0, 150, 100));

	Serial.println("Ready!");

	delay(5000);
	Serial.println("Starting!");
}

long deltas[2];
MoveCmd last;

void loop() {
	if (!(stepperA.distanceToGo() || stepperB.distanceToGo())) {
		if (positions.empty()) {
			const int sensor_got_valid_targets = ld2450.read();
			if (sensor_got_valid_targets > 0) {
				// GET THE DETECTED TARGETS
				for (int i = 0; i < sensor_got_valid_targets; i++)
				{
					LD2450::RadarTarget result_target = ld2450.getTarget(i);

					if (result_target.valid && result_target.speed > 0)
					{
						// Serial.println(result_target.y);
						// double x_offset = asin(double(result_target.x)/double(result_target.distance))* 180.0 / PI;
						// double y_offset = asin(double(result_target.y)/double(result_target.distance))* 180.0 / PI;
						// Serial.printf("X=%f deg, Y=%f deg V=%f - D=%i mm, X=%i mm, Y=%i mm\n", x_offset, y_offset, result_target.speed, result_target.distance, result_target.x, result_target.y);

						double x_offset = atan(double(result_target.x)/double(result_target.y))* 180.0 / PI;
						double y_offset = atan(double(1000)/double(result_target.distance))* 180.0 / PI;
						
						Serial.printf("Target %i at %f by %f, %i mm away going %i cm/s\n", i, x_offset, y_offset, result_target.distance, result_target.speed);

						MoveCmd newCmd;
						newCmd.H = min(max(int(x_offset / -0.1125), h_min), h_max);
						newCmd.V = min(max(int(y_offset / 0.1125), v_min), v_max);
						// newCmd.D = sensor_got_valid_targets > 1? 2500 : 0;
						// newCmd.H = min(max(int(x_offset / -0.1125), h_min), h_max);
						// newCmd.V = min(max(int(y_offset / -0.1125), h_min), h_max);
						// newCmd.H = map(x_offset*100, -35*100, 35*100, -96*100, 96*100)/100;
						// newCmd.V = map(y_offset*100, -60*100, 60*100, -400*100, 400*100)/100;
						// newCmd.S = 20;
						positions.push_back(newCmd);
						// break;
					}
				}
			}
		}

		if (positions.empty()) {
			return;
			if ((stepperA.currentPosition() == 0) && (stepperB.currentPosition() == 0)) {
				return;
			}

			positions.push_back(MoveCmd(h_max, 0, 5, 1000));
			positions.push_back(MoveCmd(h_min, 0, 5, 1000));
		}

		if (last.D > 0) {
			delay(last.D);
		}

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

		// delay(1000);
	}

	steppers.run();
}