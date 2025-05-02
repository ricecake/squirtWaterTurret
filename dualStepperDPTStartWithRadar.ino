#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <HardwareSerial.h>
#include "LD2450.h"
#include <vector>

#include "DptHelpers.h"

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

	// dptState = SystemState();

	Serial.println("SETUP_FINISHED");

	// initTestData();

	Serial.println("Ready!");

	delay(5000);

	Serial.println("Starting!");

	// create a task that will be executed in the targetingLoop() function, with priority 1 and executed on core 0
	xTaskCreatePinnedToCore(
		targetingLoop, /* Task function. */
		"Targeting",   /* name of task. */
		10000,		   /* Stack size of task */
		NULL,		   /* parameter of the task */
		1,			   /* priority of the task */
		&targeting,	   /* Task handle to keep track of created task */
		0			   /* pin task to core 0 */
	);

	// create a task that will be executed in the systemControlLoop() function, with priority 1 and executed on core 1
	xTaskCreatePinnedToCore(
		systemControlLoop, /* Task function. */
		"Control",		   /* name of task. */
		10000,			   /* Stack size of task */
		NULL,			   /* parameter of the task */
		1,				   /* priority of the task */
		&systemControl,	   /* Task handle to keep track of created task */
		1				   /* pin task to core 0 */
	);
	// vTaskStartScheduler();
}

long deltas[2];

void loop()
{
	vTaskDelay(1000);
	// uart_enable_intr_mask(uart_port_t uart_num, uint32_t enable_mask)
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

		// if (!(dptState.stepperA.distanceToGo() || dptState.stepperB.distanceToGo())) {
		// Serial.println("DONE");


/*
		if ((next.H != last.H) && (next.V != last.V))
		{
			if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
			{
				MoveCmd position = next;
				last = position;
				xSemaphoreGive(xMutex);
				doMoveOrder(position.H, position.V, position.S);
			}
		}
		// }
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

/*
bool seekTarget(MoveCmd &newCmd)
{
	// GET THE DETECTED TARGETS
	for (int i = 0; i < LD2450_MAX_SENSOR_TARGETS; i++)
	{
		LD2450::RadarTarget result_target = ld2450.getTarget(i);

		if (result_target.valid)
		{
			double x_offset = atan(double(result_target.x) / double(result_target.y)) * -180.0 / PI;
			double y_offset = atan(double(1000) / double(result_target.distance)) * -180.0 / PI; // Height of default target - height of turret = angle to aim at (table height is 1320)

			Serial.printf("Target %i at %f by %f, %i mm away going %i cm/s\n", i, x_offset, y_offset, result_target.distance, result_target.speed);

			newCmd.H = min(max(int(x_offset / 0.1125), h_min), h_max);
			newCmd.V = min(max(int(y_offset / 0.1125), v_min), v_max);
			return true;
		}
	}
}
*/

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
				dptState.updateTarget(newTarget);
			}
		}
	}
}

void targetingLoop(void *pvParameters)
{
	for (;;)
	{
		refreshTargets();
		// vTaskDelay(100);
		vTaskDelay(50/portTICK_PERIOD_MS);
		// taskYIELD();
	}
}
