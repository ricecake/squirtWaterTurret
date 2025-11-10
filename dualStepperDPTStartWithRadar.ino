#include <climits>
#include <cstdint>
#include <vector>

#include "serializer.hpp"
#include "state.h"
#include "utilities.h"

#ifdef ARDUINO
	#include "HardwareSerial.h"
	#include "LD2450.h"
	#include "esp32-hal-gpio.h"
	#include "esp_timer.h"
	#include <AccelStepper.h>
	#include <Arduino.h>
#else
	#include "tests/mocks.h"

void setup();

int main() {
	Clock::setClock(DefaultClock::now);
	setup();
	while (!threads.empty()) {
		for (auto& i : threads) {
			if (i.joinable()) {
				i.join();
			}
		}
	}
}
#endif

struct IOWrapper {
	HardwareSerial& io;

	size_t readsome(char* buf, size_t count) {
		size_t available = io.available();
		if (available > 0) {
			return io.readBytes(buf, min(count, available));
		}
		return 0;
	};

	void write(const char* cbuf, size_t count) { io.write(cbuf, count); };

	bool good() { return bool(io); };

	IOWrapper(HardwareSerial& io): io(io) {};
};

HardwareSerial RadarSerial(1);
HardwareSerial testSerial(2);

IOWrapper wrapped(testSerial);

cerializer::StreamHandler streamHandler(wrapped);
LD2450                    ld2450;

SystemState dptState;

TaskHandle_t targeting;
TaskHandle_t systemControl;

void loop() {
	vTaskDelay(1000);
}

void refreshRadarTargets() {
	const int sensor_got_valid_targets = ld2450.read();
	if (sensor_got_valid_targets > 0) {
		for (int i = 0; i < sensor_got_valid_targets; i++) {
			LD2450::RadarTarget result_target = ld2450.getTarget(i);

			if (result_target.valid) {
				auto newPositionObservation = PositionVector(
					fixed(result_target.x) / 1000,
					fixed(result_target.y) / 1000,
					1.1
				);
				dptState.updateTarget(
					dptState.radarTarget,
					result_target.id,
					result_target.valid,
					newPositionObservation,
					8
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
	streamHandler.ParseStream<cerializer::BasePacket>(
		std::function<void(cerializer::BasePointer&)>([](cerializer::BasePointer& thing) {
			if (!thing) {
				return;
			}

			switch (thing->Code()) {
			case cerializer::Target::Type(): {
				auto target = static_cast<cerializer::Target*>(thing.get());
				auto newPositionObservation = PositionVector(
					fixed(target->x) / 1000,
					fixed(target->y) / 1000,
					fixed(target->z) / 1000
				);

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
				auto newPosition = PositionVector(
					fixed(static_target_msg->x) / 1000,
					fixed(static_target_msg->y) / 1000,
					fixed(static_target_msg->z) / 1000
				);

				dptState.staticTarget.Update(newPosition);
				break;
			}
			}
		})
	);
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
	switch (dptState.target_source) {
	case TargetSource::STATIC:
		break;
	// Set current target to static target
	case TargetSource::RADAR:
		break;
	// Check if the current target matches our criteria.
	// If not, set target to closest target that does.  Distance should factor in action time.
	case TargetSource::CV:
		break;
		// Check if the current target matches our criteria.
		// If not, set target to closest target that does.
	}
	// Target switch commands should use the queue back method, so they come after any commands to stop firing.

	/*
Depending on the source and the mode (persistent, closest, farthest, random, least, most, etc (basically how we pick the
target from the set of sources)), we pick the new target.  Should have a new target source "scan", that just does an
idle sentry scan, and a mode for "aggressive" that will have it test fire. Also changes the fire cadenece in other
modes. Scan will issue a command that moves around and re-queues itself.  It will stop if it sees that a target has been
selected or sources have changed. Select target is now what will be used to actually put a change of target order into
the queue at the back.  It will change sources and everything
*/
}

void generateFireActions() {
	if (dptState.targetTravelDistance() <= 2) {
		dptState.queueFire(500);
	}
}

void systemControlLoop(void*) {
	for (;;) {
		try {
			dptState.processCommandQueue();
		} catch (const std::exception& e) {
			std::cerr << "Caught exception: " << e.what() << std::endl;
		}
		try {
			dptState.actualizeState();
		} catch (const std::exception& e) {
			std::cerr << "Caught exception: " << e.what() << std::endl;
		}
		vTaskDelay(1);
	}
}

void targetingLoop(void*) {
	for (;;) {
		try {
			refreshRadarTargets();
		} catch (const std::exception& e) {
			std::cerr << "Caught exception: " << e.what() << std::endl;
		}

		try {
			readSerialCommands();
		} catch (const std::exception& e) {
			std::cerr << "Caught exception: " << e.what() << std::endl;
		}

		try {
			selectTarget();
		} catch (const std::exception& e) {
			std::cerr << "Caught exception: " << e.what() << std::endl;
		}

		try {
			generateFireActions();
		} catch (const std::exception& e) {
			std::cerr << "Caught exception: " << e.what() << std::endl;
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

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

	dptState.queueSelectTarget(TargetSource::RADAR, 1, 3 * 1000);

	Serial.println("SETUP_FINISHED");

	Serial.println("Ready!");

	delay(1000);

	Serial.println("Starting!");

	xTaskCreatePinnedToCore(targetingLoop, "Targeting", 10000, NULL, 1, &targeting, 0);

	xTaskCreatePinnedToCore(systemControlLoop, "Control", 10000, NULL, 1, &systemControl, 1);
}
