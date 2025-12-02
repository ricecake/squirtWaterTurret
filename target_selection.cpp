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
	state->setTarget(target_source, target_id, static_cast<uint8_t>(speed));
}

/**
 * @brief Constructs a new TargetSelection object.
 *
 * @param target_id The ID of the target to select.
 * @param initial_speed The tracking speed to use.
 * @param initial_run_after The time delay (in microseconds) after which the command should run.
 */
TargetSelection::TargetSelection(
	TargetSource initial_target_source,
	uint8_t      initial_target_id,
	int          initial_speed,
	int64_t      initial_run_after
):
	Command(static_cast<uint64_t>(initial_run_after)),
	target_source(initial_target_source),
	target_id(initial_target_id),
	speed(initial_speed) {}
