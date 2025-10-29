#include "command.h"

#include <chrono>

#include "utilities.h"

/**
 * @brief Constructs a new Command object.
 *
 * Initializes the command with a unique ID based on the current time and calculates
 * the absolute time at which it should be executed.
 *
 * @param run_after The time delay (in microseconds) after which the command should be executed.
 */
Command::Command(uint64_t run_after) {
	// Assign a unique ID using the current timer value
	id = microSinceEpoch();
	// Calculate the absolute execution time
	this->run_after = id + run_after;
}
