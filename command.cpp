#include "command.h"

#include <chrono>

#include "state.h"
#include "utilities.h"

/**
 * @brief Constructs a new Command object.
 *
 * Initializes the command with a unique ID based on the current time and calculates
 * the absolute time at which it should be executed.
 *
 * @param delay_us The time delay (in microseconds) after which the command should be executed.
 */
Command::Command(uint64_t delay_us) {
	// Assign a unique ID using the current timer value
	id = id_counter++;
	// Calculate the absolute execution time
	run_after = microSinceEpoch() + delay_us;
}

SetStrategyCommand::SetStrategyCommand(TurretStrategy s, uint64_t delay_us): Command(delay_us), strategy(s) {}

void SetStrategyCommand::Execute(SystemState* state) {
	state->setStrategy(strategy);
}

SetStanceCommand::SetStanceCommand(TurretStance st, uint64_t delay_us): Command(delay_us), stance(st) {}

void SetStanceCommand::Execute(SystemState* state) {
	state->setStance(stance);
}
