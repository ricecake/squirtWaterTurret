#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <HardwareSerial.h>
#include <vector>

#include "LD2450.h"
#include "DptHelpers.h"
#include "state.h"
#include "utilities.h"

HardwareSerial RadarSerial(1);
LD2450 ld2450;

SystemState dptState;

TaskHandle_t targeting;
TaskHandle_t systemControl;

void setup()
{
	Serial.begin(9600);

	while (!Serial)
	{
		; // wait for serial port to connect. Needed for native USB
	}

	RadarSerial.begin(256000, SERIAL_8N1, 16, 17);
	ld2450.begin(RadarSerial, false);

	if (!ld2450.waitForSensorMessage(true))
	{
		Serial.println("SENSOR CONNECTION SEEMS OK");
	}
	else
	{
		Serial.println("SENSOR TEST: GOT NO VALID SENSORDATA - PLEASE CHECK CONNECTION!");
	}

	randomSeed(analogRead(0));

	Serial.println("SETUP_FINISHED");

	// initTestData();

	dptState.setTarget(1);
	// dptState.queueSelectTarget(1, 5*1000);

	Serial.println("Ready!");

	delay(5000);

	Serial.println("Starting!");

	xTaskCreatePinnedToCore(
		targetingLoop,
		"Targeting",
		10000,
		NULL,
		1,
		&targeting,
		0
	);

	xTaskCreatePinnedToCore(
		systemControlLoop,
		"Control",
		10000,
		NULL,
		1,
		&systemControl,
		1
	);
}

void loop()
{
	vTaskDelay(1000);
}


int last_time = 0;
void systemControlLoop(void *pvParameters)
{
	for (;;)
	{
		/*
		Change the flow to be fetching from the command queue while the commands are scheduled for now or earlier.
		Each command will get a reference to the dptState, and can mutate it.
		Then we actualize the dptState.
		set firing pin to the set dptState.
		load move orders

		This means that targeting system finds targets and updates the dptState
		the command queue contains updates on which target to select

		later, when pose estimation is in place, it can send data and the targeter can tweak the specific coordinates of each target as appropriate
		*/
		dptState.processCommandQueue();
		dptState.actualizeState();
		dptState.steppers.run();
		vTaskDelay(1);

/*
		// x_Setpoint = next.H *-0.1125;
		// y_Setpoint = next.V * 0.1125;
		// x_Input = (a_pos - b_pos) / 2;
		// y_Input = (a_pos + b_pos) / 2;
		int interval = 2000;
		if (millis() - last_time > interval) {
			last_time += interval;
			long a_pos = dptState.stepperA.currentPosition();
			long b_pos = dptState.stepperB.currentPosition();
			Serial.printf("Target: [%0.2f %0.2f] At: [[%0.2f %0.2f]]\n", next.H *-0.1125, next.V * 0.1125, dptState.angleToStep*float(b_pos - a_pos) / 2, dptState.angleToStep*float(a_pos + b_pos) / 2);
		}
*/
	}
}

void refreshTargets() {
	const int sensor_got_valid_targets = ld2450.read();
	if (sensor_got_valid_targets > 0)
	{
		// GET THE DETECTED TARGETS
		for (int i = 0; i < sensor_got_valid_targets; i++)
		{
			LD2450::RadarTarget result_target = ld2450.getTarget(i);

			if (result_target.valid)
			{
				auto newTarget = Target(result_target.id, result_target.x, result_target.y, result_target.speed, result_target.valid);
				dptState.updateTarget(newTarget, 16);
			}
		}
	}
}

void generateFireActions() {
	Target& target = dptState.currentTarget();

	if (target.actionIdleExceeds(seconds(1)) && dptState.targetTravelDistance() < 10) {
		dptState.queueFire(250);
		target.IncrementAction();
	}
}

void targetingLoop(void *pvParameters)
{
	for (;;)
	{
		refreshTargets();
		generateFireActions();
		vTaskDelay(10/portTICK_PERIOD_MS);
		// vTaskDelay(1);
	}
}
