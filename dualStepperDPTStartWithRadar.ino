#include <climits>
#include <cstdint>
#include <cstdlib>
#include <ranges>
#include <vector>

#include "logger.h"
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

	void write(const char* cbuf, size_t count) { io.write(reinterpret_cast<const uint8_t*>(cbuf), count); };

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
				dptState.queueSelectTarget(TargetSource::STATIC, 0);
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

// bool selectTarget() {
// 	/*
// 	    // Should find the nearest target not acted upon recently
// 	    Then the distance should be calculated in the fire action
// 	    the fire action should be told about the target, not the duration.
// 	    Queue fire should take the number of shots, and then calculate the times to start and stop.
// 	    select target will also pay attention to if the position has been overridden, so that it can make smart choices
// 	    refresh targets will need to be reworked into "check radar" and "check external comms"
// 	    since external comms may include things like position changes, and fire commands.
// 	*/
// 	switch (dptState.target_source) {
// 	case TargetSource::STATIC:
// 		// If we're on static target, we stay on static target
// 		return false;
// 	case TargetSource::RADAR:
// 		// Check if the current target matches our criteria.
// 		// If not, set target to closest target that does.  Distance should factor in action time.
// 		return true;
// 	case TargetSource::CV:
// 		// Check if the current target matches our criteria.
// 		// If not, set target to closest target that does.
// 		return true;
// 	}
// 	return false;
// 	// Target switch commands should use the queue back method, so they come after any commands to stop firing.

/*
Depending on the source and the mode (persistent, closest, farthest, random, least, most, etc (basically how we pick the
target from the set of sources)), we pick the new target.  Should have a new target source "scan", that just does an
idle sentry scan, and a mode for "aggressive" that will have it test fire. Also changes the fire cadence in other
modes. Scan will issue a command that moves around and re-queues itself.  It will stop if it sees that a target has been
selected or sources have changed. Select target is now what will be used to actually put a change of target order into
the queue at the back.  It will change sources and everything
*/
// }

struct Target* selectTarget() {
	if (dptState.currentTarget()->actionable()) {
		return nullptr;
	}

	auto filter = std::views::filter([](Target& t) {
		if (t.valid && t.idleExceeds(seconds(5))) {
			t.valid = false;
		}
		return t.valid;
	});

	auto targets = filter(dptState.currentTargetArray());

	if (targets.empty()) {
		return nullptr;
	}

	// A helper lambda to find the best target using a custom projection
	auto find_best = [&](auto projection, bool find_max = false) {
		if (find_max) {
			return &(*std::ranges::max_element(targets, std::less<>{}, projection));
		}
		return &(*std::ranges::min_element(targets, std::less<>{}, projection));
	};

	Target* nextTarget;
	switch (dptState.currentStrategy()) {
	case TurretStrategy::CLOSEST:
		nextTarget = find_best(&Target::Distance);
		break;
	case TurretStrategy::FURTHEST:
		nextTarget = find_best(&Target::Distance, true);
		break;
	case TurretStrategy::LEAST_RECENT:
		nextTarget = find_best(&Target::last_action);
		break;
	case TurretStrategy::MOST_RECENT:
		nextTarget = find_best(&Target::last_action, true);
		break;
	// case TurretStrategy::LEAST_HIT:
	// 	return find_best(&Target::action_count);
	// case TurretStrategy::MOST_HIT:
	// 	return find_best(&Target::action_count, true);
	// case TurretStrategy::SMALLEST_TRAVEL:
	// There's something to be done to consider calculating most of the distance calc up front, and then solving for
	// each target individually.  Later optimization though.
	// 	return find_best([&](const Target& t) { return
	// calculateTravelDistance(t); }); case TurretStrategy::LONGEST_TRAVEL: 	return find_best([&](const Target& t) {
	// return calculateTravelDistance(t); }, true);
	case TurretStrategy::RANDOM: {
		// This is a bit more complex, so we'll handle it separately.
		nextTarget = &dptState.currentTargetArray()[std::rand() % dptState.size()];
		break;
	}
	default:
		nextTarget = &(*targets.begin());
		break;
	}
	if (nextTarget == dptState.currentTarget()) {
		return nullptr;
	}
	return nextTarget;
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
			bool tryFire = true;
			if (dptState.shouldCheckTargetValidity()) {
				auto newTarget = selectTarget();
				if (newTarget != nullptr) {
					dptState.queueSelectTarget(dptState.target_source, newTarget->index);
					tryFire = false;
				}
			}
			if (tryFire && dptState.shouldCheckFiringConditions()) {
				generateFireActions();
			}
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
		logger::LOG("SENSOR CONNECTION SEEMS OK");
	} else {
		logger::LOG("SENSOR TEST: GOT NO VALID SENSOR DATA - PLEASE CHECK CONNECTION!");
	}

	testSerial.begin(9600, SERIAL_8N1, 19, 18);
	randomSeed(analogRead(0));

	logger::LOG("SETUP_FINISHED");

	logger::LOG("Ready!");

	delay(1000);

	logger::LOG("Starting!");

	xTaskCreatePinnedToCore(targetingLoop, "Targeting", 10000, NULL, 1, &targeting, 0);

	xTaskCreatePinnedToCore(systemControlLoop, "Control", 10000, NULL, 1, &systemControl, 1);
}
