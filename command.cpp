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
 * @param run_after The time delay (in microseconds) after which the command should be executed.
 */
Command::Command(uint64_t run_after) {
	// Assign a unique ID using the current timer value
	id = id_counter++;
	// Calculate the absolute execution time
	this->run_after = microSinceEpoch() + run_after;
}

SetStrategyCommand::SetStrategyCommand(TurretStrategy strategy, uint64_t run_after):
	Command(run_after), strategy(strategy) {}

void SetStrategyCommand::Execute(SystemState* state) {
	state->setStrategy(strategy);
}

SetStanceCommand::SetStanceCommand(TurretStance stance, uint64_t run_after): Command(run_after), stance(stance) {}

void SetStanceCommand::Execute(SystemState* state) {
	state->setStance(stance);
}
