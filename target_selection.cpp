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
	state->setTarget(target_id, speed);
	auto currTarget = state->currentTarget();

	uint16_t timeout = 0;
	if (currTarget.valid) {
		if (currTarget.idleExceeds(seconds(5))) {
			currTarget.valid = false;
		} else {
			timeout = 3 * 1000;
		}
	}

	// Queue the next target selection to create a round-robin evaluation
	state->queueSelectTarget(((target_id + 1) % state->size()), timeout);
}

/**
 * @brief Constructs a new TargetSelection object.
 *
 * @param target_id The ID of the target to select.
 * @param speed The tracking speed to use.
 * @param run_after The time delay (in microseconds) after which the command should run.
 */
TargetSelection::TargetSelection(uint8_t target_id, int speed, int64_t run_after) :
    Command(run_after), target_id(target_id), speed(speed) {}
