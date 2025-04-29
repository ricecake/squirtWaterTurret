#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <HardwareSerial.h>
#include "LD2450.h"
#include <vector>

#include "DptHelpers.h"

// Creates an instance
AccelStepper stepperA(motorInterfaceType, stepPinA, dirPinA);
AccelStepper stepperB(motorInterfaceType, stepPinB, dirPinB);

MultiStepper steppers;
HardwareSerial RadarSerial(1);
LD2450 ld2450;

SemaphoreHandle_t xMutex;
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
	// set the maximum speed, acceleration factor,
	// initial speed and the target position
	stepperA.setMaxSpeed(maxSpeed);
	stepperB.setMaxSpeed(maxSpeed);
	stepperA.setAcceleration(acceleration);
	stepperB.setAcceleration(acceleration);

	steppers.addStepper(stepperA);
	steppers.addStepper(stepperB);

	xMutex = xSemaphoreCreateMutex();

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
MoveCmd last;
MoveCmd next;

void loop()
{
	vTaskDelay(1000);
}

void doMoveOrder(int H, int V, int S)
{
	int delta_A = H + V;
	int delta_B = V - H;

	long moveA = delta_A - stepperA.currentPosition();
	long moveB = delta_B - stepperB.currentPosition();
	long distance = (moveA * moveA) + (moveB * moveB);
	// Serial.printf("Moving to (%i, %i) [%f, %f] at %u via delta (%i, %i)  [%i, %i]\n", H, V, H*0.225, V*0.225, S, delta_A, delta_B, stepperA.currentPosition(), stepperB.currentPosition());

	if (distance <= 50)
	{
		// Serial.printf("Skipping [%i %i] [%i %i] [%i %i] [%i %i])\n", H, V, delta_A, delta_B, stepperA.currentPosition(), stepperB.currentPosition(), moveA, moveB);
		return;
	}

	float iterMaxSpeed = S > 0 ? S : maxSpeed;

	iterMaxSpeed *= iterMaxSpeed/maxSpeed * min(distance/float(400), float(1));

	// iterMaxSpeed = max(min(iterMaxSpeed, float(maxSpeed)), float(25));
	iterMaxSpeed = min(iterMaxSpeed, float(maxSpeed));

	Serial.printf("Moving to (%i, %i) [%f, %f] at %f via delta (%i, %i) -> %i\n", H, V, H * 0.225, V * 0.225, iterMaxSpeed, moveA, moveB, distance);

	iterMaxSpeed *= stepFraction;

	deltas[0] = delta_A;
	deltas[1] = delta_B;
/*
	// This might need to be some form of smoothing function that takes target positions and smooths them out into a motion path?
		Basically take multiple target positions over time, and try to match the targets velocity, and also their positiono.
		I think that's something that a pid controller does?
		Yes, pid controller.  
		Pitch and yaw each get a controller, and it should output a movement speed for each motor.
		We should set the speed for each of them and use runSpeed to move at that velocity.
		It's inputs should be the current position in the respective dimension.
*/
	stepperA.setMaxSpeed(iterMaxSpeed);
	stepperB.setMaxSpeed(iterMaxSpeed);
	steppers.moveTo(deltas);
	// stepperA.moveTo(delta_A);
	// stepperB.moveTo(delta_B);
}

void systemControlLoop(void *pvParameters)
{
	for (;;)
	{
		/*
		Change the flow to be fetching from the command queue while the commands are scheduled for now or earlier.
		Each command will get a reference to the state, and can mutate it.
		Then we actualize the state.
		set firing pin to the set state.
		load move orders

		This means that targeting system finds targets and updates the state
		the command queue contains updates on which target to select

		later, when pose estimation is in place, it can send data and the targeter can tweak the specific coordinates of each target as appropriate
		*/

		// if (!(stepperA.distanceToGo() || stepperB.distanceToGo())) {
		// Serial.println("DONE");
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
		steppers.run();
		// stepperA.run();
		// stepperB.run();
		// vTaskDelay(1);
		taskYIELD();
	}
}

bool seekTarget(MoveCmd &newCmd)
{
	// GET THE DETECTED TARGETS
	for (int i = 0; i < LD2450_MAX_SENSOR_TARGETS; i++)
	{
		LD2450::RadarTarget result_target = ld2450.getTarget(i);

		if (result_target.valid)
		{
			double x_offset = atan(double(result_target.x) / double(result_target.y)) * 180.0 / PI;
			double y_offset = atan(double(1000) / double(result_target.distance)) * 180.0 / PI;

			// Serial.printf("Target %i at %f by %f, %i mm away going %i cm/s\n", i, x_offset, y_offset, result_target.distance, result_target.speed);

			newCmd.H = min(max(int(x_offset / -0.1125), h_min), h_max);
			newCmd.V = min(max(int(y_offset / 0.1125), v_min), v_max);
			return true;
		}
	}
}

void targetingLoop(void *pvParameters)
{
	for (;;)
	{
		uint64_t s = esp_timer_get_time();
		const int sensor_got_valid_targets = ld2450.read();
		if (sensor_got_valid_targets > 0)
		{
			// uint64_t f = esp_timer_get_time();
			// Serial.println(f-s);
			MoveCmd newCmd;
			if (seekTarget(newCmd))
			{
				if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
				{
					next = newCmd;
					xSemaphoreGive(xMutex);
				}
			}
		}
		vTaskDelay(1);
		// taskYIELD();
	}
}
