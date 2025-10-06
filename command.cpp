#include "command.h"
#include <chrono>

#ifdef ARDUINO
#include <esp_timer.h>
#else
#include "tests/mocks.h"
#endif

/**
 * @brief Constructs a new Command object.
 *
 * Initializes the command with a unique ID based on the current time and calculates
 * the absolute time at which it should be executed.
 *
 * @param run_after The time delay (in microseconds) after which the command should be executed.
 */
Command::Command(int64_t run_after)
{
	// Assign a unique ID using the current timer value
	id = esp_timer_get_time();
	// Calculate the absolute execution time
	this->run_after = id + run_after;
}

/**
 * @brief Constructs a new LingerCommand object.
 * @param run_after The time delay (in microseconds) after which the command should run.
 */
LingerCommand::LingerCommand(int64_t run_after) : Command(run_after) {}

/**
 * @brief Executes the linger command. This method does nothing.
 * @param state A pointer to the system state (unused).
 */
void LingerCommand::Execute(SystemState * /*state*/)
{
    // This command intentionally does nothing.
}
