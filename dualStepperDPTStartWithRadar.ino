#include <climits>
#include <vector>

#include "HardwareSerial.h"
#include "LD2450.h"
#include "esp32-hal-gpio.h"
#include "esp_timer.h"
#include "serializer.hpp"
#include "state.h"
#include "utilities.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>

HardwareSerial RadarSerial(1);
HardwareSerial testSerial(2);

struct IOWrapper {
	HardwareSerial& io;
	size_t          readsome(char* buf, size_t count) {
        size_t available = io.available();
        if (available > 0) {
            return io.readBytes(buf, min(count, available));
        }
        return 0;
	};
	bool good() { return bool(io); };
	IOWrapper(HardwareSerial& io) : io(io){};
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
		; // wait for serial port to connect. Needed for native USB
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

	xTaskCreatePinnedToCore(targetingLoop, "Targeting", 10000, NULL, 1, &targeting, 0);

	xTaskCreatePinnedToCore(systemControlLoop, "Control", 10000, NULL, 1, &systemControl, 1);
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

void refreshRadarTargets() {
	const int sensor_got_valid_targets = ld2450.read();
	if (sensor_got_valid_targets > 0) {
		for (int i = 0; i < sensor_got_valid_targets; i++) {
			LD2450::RadarTarget result_target = ld2450.getTarget(i);

			if (result_target.valid) {
				auto newPositionObservation =
					PositionVector(fixed(result_target.x) / 1000, fixed(result_target.y) / 1000, 1.1);
				dptState.updateTarget(
					dptState.radarTarget, result_target.id, result_target.valid, newPositionObservation, 8
				);
				// Have this update a list of radar targets, and the other update a list of external targets.
				// Radar targets get a pre-defined guess at average height of target point.
				// This should reduce the amount of calculation in refresh cycle.
				// When selecting a target, can instead pick the best from each list, falling back to radar if no
				// external targets. Separate the two -- by default populate the radar target and prefer the target list
				// if possible
			}
		}
	}
}

void readSerialCommands() {
	deserializer.ParseStream(std::function<void(cerializer::BasePointer&)>([](cerializer::BasePointer& thing) {
		if (!thing) {
			return;
		}

		switch (thing->Code()) {
		case cerializer::Target::Type(): {
			auto target = static_cast<cerializer::Target*>(thing.get());
			auto newPositionObservation =
				PositionVector(fixed(target->x) / 1000, fixed(target->y) / 1000, fixed(target->z) / 1000);
			dptState.updateTargetById(dptState.cvTarget, target->id, target->valid, newPositionObservation, 8);
			break;
		}
		case cerializer::Config::Type(): {
			dptState.updateConfig(static_cast<cerializer::Config*>(thing.get()));
			break;
		}
		case cerializer::SetTargetSourceMessage::Type(): {
			auto source_msg = static_cast<cerializer::SetTargetSourceMessage*>(thing.get());
			dptState.target_source = source_msg->source;
			break;
		}
		case cerializer::StaticTargetMessage::Type(): {
			auto static_target_msg = static_cast<cerializer::StaticTargetMessage*>(thing.get());
			dptState.staticTarget.position.X_coord = fixed(static_target_msg->x) / 1000;
			dptState.staticTarget.position.Y_coord = fixed(static_target_msg->y) / 1000;
			dptState.staticTarget.position.Z_coord = fixed(static_target_msg->z) / 1000;
			break;
		}
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
		refreshRadarTargets();
		readSerialCommands();
		selectTarget();
		generateFireActions();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}