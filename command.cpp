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
 * @param initial_run_after The time delay (in microseconds) after which the command should be executed.
 */
Command::Command(uint64_t initial_run_after) {
	// Assign a unique ID using the current timer value
	id = id_counter++;
	// Calculate the absolute execution time
	this->run_after = microSinceEpoch() + initial_run_after;
}

SetStrategyCommand::SetStrategyCommand(TurretStrategy initial_strategy, uint64_t initial_run_after):
	Command(initial_run_after), strategy(initial_strategy) {}

void SetStrategyCommand::Execute(SystemState* state) {
	state->setStrategy(strategy);
}

SetStanceCommand::SetStanceCommand(TurretStance initial_stance, uint64_t initial_run_after):
	Command(initial_run_after), stance(initial_stance) {}

void SetStanceCommand::Execute(SystemState* state) {
	state->setStance(stance);
}
