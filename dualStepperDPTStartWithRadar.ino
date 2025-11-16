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

bool selectTarget() {
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
		// If we're on static target, we stay on static target
		return false;
	case TargetSource::RADAR:
		// Check if the current target matches our criteria.
		// If not, set target to closest target that does.  Distance should factor in action time.
		return true;
	case TargetSource::CV:
		// Check if the current target matches our criteria.
		// If not, set target to closest target that does.
		return true;
	}
	return false;
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

template <typename T, typename C>
struct TargetEvalCache {
	Target* target;
	T       metric;

	bool checkUpdate(Target* candidate, T value, C cmp = C{}) {
		if (target == nullptr || cmp(value, metric)) {
			metric = value;
			target = candidate;
			return true;
		}
		return false;
	}
};

template <typename C>
struct TargetDistanceEval: public TargetEvalCache<fixed, C> {};

template <typename C>
struct TargetTimeEval: public TargetEvalCache<TimePoint, C> {};

template <typename C>
struct TargetSeekEval: public TargetEvalCache<fixed, C> {};

struct TargetEvaluationState {
	Target* currentTarget;
	Target* newTarget;

	bool currentTargetActionable;
	bool newTargetActionable;

	TargetDistanceEval<std::less<>>    leastDistance;
	TargetDistanceEval<std::greater<>> mostDistance;

	TargetTimeEval<std::less<>>    leastRecent;
	TargetTimeEval<std::greater<>> mostRecent;

	TargetSeekEval<std::less<>> leastSeek;
	TargetSeekEval<std::less<>> mostSeek;

	TargetSource   source;
	TurretStrategy strategy;
	TurretStance   stance;

	TargetEvaluationState(SystemState& state) {
		if (source == TargetSource::STATIC) {
			// short circuit for no change.
		}
		if (strategy == TurretStrategy::RANDOM) {
			// short circuit for selecting a random target
		}
		currentTarget = state.selectedTarget;
		source = state.target_source;
		// strategy = state.getStrategy();
		// stance = state.getStance();
		for (auto target : state.currentTargetArray()) {
			if (target.actionable() && state.config.projectile_max_range > target.Position().magnitude()) {
				auto distance = target.Position().magnitude();
				auto interval = target.last_action;

				leastDistance.checkUpdate(&target, distance);
				mostDistance.checkUpdate(&target, distance);
				leastRecent.checkUpdate(&target, interval);
				mostRecent.checkUpdate(&target, interval);
			}
		}

		switch (strategy) {
		case TurretStrategy::LEAST_HIT: {
		}
		case TurretStrategy::MOST_HIT: {
		}
		case TurretStrategy::CLOSEST: {
		}
		case TurretStrategy::FURTHEST: {
		}
		case TurretStrategy::LEAST_RECENT: {
		}
		case TurretStrategy::MOST_RECENT: {
		}
		case TurretStrategy::SMALLEST_TRAVEL: {
		}
		case TurretStrategy::LONGEST_TRAVEL: {
		}
		case TurretStrategy::RANDOM: {
		}
		}
	}

	// Basically, run a loop builiding this state.
	// Then it can serve as a sort of "per round cache" to avoid needing to recalculate things so aggressively.
	// then at the end it can say what action to  take and then emit the right one, rather than calling function with a
	// complicated check.
};

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
			// auto evalState = TargetEvaluationState(dptState);
			bool tryFire = true;
			if (dptState.shouldCheckTargetValidity()) {
				tryFire = !selectTarget();
			}
			if (tryFire) {
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
		Serial.println("SENSOR CONNECTION SEEMS OK");
	} else {
		Serial.println("SENSOR TEST: GOT NO VALID SENSORDATA - PLEASE CHECK CONNECTION!");
	}

	testSerial.begin(9600, SERIAL_8N1, 19, 18);
	randomSeed(analogRead(0));

	dptState.queueSelectTarget(TargetSource::RADAR, 1);

	Serial.println("SETUP_FINISHED");

	Serial.println("Ready!");

	delay(1000);

	Serial.println("Starting!");

	xTaskCreatePinnedToCore(targetingLoop, "Targeting", 10000, NULL, 1, &targeting, 0);

	xTaskCreatePinnedToCore(systemControlLoop, "Control", 10000, NULL, 1, &systemControl, 1);
}
