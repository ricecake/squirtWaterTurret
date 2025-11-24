#include "target_selection.h"

#include "state.h"
#include "utilities.h"

/**
 * @brief Executes the target selection command.
 *
 * This method sets the specified target as the current one and then queues the
 * next target selection command. This creates a cycle of target evaluation.
 *
 * @param state A pointer to the system state.
 */
void TargetSelection::Execute(SystemState* state) {
	state->setTarget(target_source, target_id, speed);
}

/**
 * @brief Constructs a new TargetSelection object.
 *
 * @param target_id The ID of the target to select.
 * @param speed The tracking speed to use.
 * @param run_after The time delay (in microseconds) after which the command should run.
 */
TargetSelection::TargetSelection(TargetSource target_source, uint8_t target_id, int speed, int64_t run_after):
	Command(run_after), target_source(target_source), target_id(target_id), speed(speed) {}
