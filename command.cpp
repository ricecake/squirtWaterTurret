/**
 * @file command.cpp
 * @brief Implements the Command base class and related command types.
 *
 * This file contains the method definitions for the Command and LingerCommand classes.
 */
#include "command.h"

#ifdef ARDUINO
	#include <esp_timer.h>
#else
	#include "tests/mocks.h"
#endif

#include <chrono>

/**
 * @brief Constructs a new Command object.
 *
 * Initializes the command with a unique ID based on the current high-resolution
 * timer. It calculates the absolute time at which the command should be executed
 * by adding the `run_after` delay to the current time.
 *
 * @param run_after The time delay (in microseconds) from the current time at which
 *                  the command should be executed.
 */
Command::Command(int64_t run_after) {
	// Assign a unique ID using the current timer value
	id = esp_timer_get_time();
	// Calculate the absolute execution time by adding the delay to the current time
	this->run_after = id + run_after;
}

/**
 * @brief Constructs a new LingerCommand object.
 *
 * This constructor simply forwards the `run_after` delay to the base `Command` constructor.
 *
 * @param run_after The time delay (in microseconds) after which the command should execute.
 */
LingerCommand::LingerCommand(int64_t run_after) : Command(run_after) {}

/**
 * @brief Executes the linger command.
 *
 * This method is intentionally empty as the purpose of a LingerCommand is to
 * create a pause in the command queue without performing any action.
 *
 * @param state A pointer to the system state (unused).
 */
void LingerCommand::Execute(SystemState* /*state*/) {
	// This command intentionally does nothing.
}
