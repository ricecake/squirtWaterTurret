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
TargetSelection::TargetSelection(
	TargetSource init_target_source,
	uint8_t      init_target_id,
	uint8_t      init_speed,
	uint64_t     init_run_after
):
	Command(init_run_after), target_source(init_target_source), target_id(init_target_id), speed(init_speed) {}
