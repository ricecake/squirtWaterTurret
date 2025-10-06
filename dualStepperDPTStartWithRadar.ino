#ifdef ARDUINO
#include <AccelStepper.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <MultiStepper.h>
#include <vector>

#include "DptHelpers.h"
#include "LD2450.h"
#include "serializer.hpp"
#include "state.h"
#include "utilities.h"

HardwareSerial RadarSerial(1);
HardwareSerial testSerial(2);

struct IOWrapper {
	HardwareSerial& io;
	size_t          readsome(char* buf, size_t count) {
        return io.readBytes(buf, count);
	};
	bool good() {
		return bool(io);
	};
	IOWrapper(HardwareSerial& io) :
		io(io) {};
};

IOWrapper wrapped(testSerial);

cerializer::Deserializer deserializer(wrapped);
LD2450                   ld2450;

SystemState dptState;

TaskHandle_t targeting;
TaskHandle_t systemControl;

void setup() {
	Serial.begin(9600);

	while (!Serial) {
		;  // wait for serial port to connect. Needed for native USB
	}

	RadarSerial.begin(256000, SERIAL_8N1, 16, 17);
	ld2450.begin(RadarSerial, false);

	if (!ld2450.waitForSensorMessage(true)) {
		Serial.println("SENSOR CONNECTION SEEMS OK");
	} else {
		Serial.println("SENSOR TEST: GOT NO VALID SENSORDATA - PLEASE CHECK CONNECTION!");
	}

	testSerial.begin(9600, SERIAL_8N1, 19, 18);
	randomSeed(analogRead(0));

	dptState.queueSelectTarget(1, 3 * 1000);

	Serial.println("SETUP_FINISHED");

	Serial.println("Ready!");

	delay(1000);

	Serial.println("Starting!");

	xTaskCreatePinnedToCore(
		targetingLoop,
		"Targeting",
		10000,
		NULL,
		1,
		&targeting,
		0);

	xTaskCreatePinnedToCore(
		systemControlLoop,
		"Control",
		10000,
		NULL,
		1,
		&systemControl,
		1);
}

void loop() {
	vTaskDelay(1000);
}

int  last_time = 0;
void systemControlLoop(void* pvParameters) {
	for (;;) {
		dptState.processCommandQueue();
		dptState.actualizeState();
		vTaskDelay(1);
	}
}

void refreshTargets() {
	const int sensor_got_valid_targets = ld2450.read();
	if (sensor_got_valid_targets > 0) {
		for (int i = 0; i < sensor_got_valid_targets; i++) {
			LD2450::RadarTarget result_target = ld2450.getTarget(i);

			if (result_target.valid) {
				auto newPositionObservation = PositionVector(fixed(result_target.x) / 1000, fixed(result_target.y) / 1000, 1.1);
				dptState.updateNearestTarget2d(result_target.valid, newPositionObservation, 8);
				// Have this update a list of radar targets, and the other update a list of external targets.
				// Radar targets get a pre-defined guess at average height of target point.
				// This should reduce the amount of calculation in refresh cycle.
				// When selecting a target, can instead pick the best from each list, falling back to radar if no external targets.
				// Separate the two -- by default populate the radar target and prefer the target list if possible
			}
		}
	}

	deserializer.ParseStream(std::function<void(cerializer::BasePointer&)>([](cerializer::BasePointer& thing) {
		auto thingCode = thing->Code();
		switch (thing->Code()) {
		case cerializer::Target::Type():
			auto target = static_cast<cerializer::Target*>(thing.get());

			auto newPositionObservation = PositionVector(fixed(target->x) / 1000, fixed(target->y) / 1000, fixed(target->z) / 1000);
			dptState.updateTargetById(target->id, target->valid, newPositionObservation, 8);
		}
	}));
}

void generateFireActions() {
	Target& target = dptState.currentTarget();

	if (dptState.targetTravelDistance() <= 2) {
		dptState.queueFire(500);
	}
}

void selectTarget() {
	/*
		// Should find the nearest target not acted upon recently
		Then the distance should be calculated in the fire action
		the fire action should be told about the target, not the duration.
		Queue fire should take the number of shots, and then calculate the times to start and stop.
		select target will also pay attention to if the position has been overridden, so that it can make smart choices
		refresh targets will need to be reworked into "check radar" and "check external comms"
		since external comms may include things like position changes, and fire commands.
	*/
}

void targetingLoop(void* pvParameters) {
	for (;;) {
		refreshTargets();
		selectTarget();
		generateFireActions();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
#endif