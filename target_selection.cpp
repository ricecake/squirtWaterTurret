/**
 * @file target_selection.cpp
 * @brief Implements the TargetSelection command class.
 */
#include "target_selection.h"

#include "state.h"
#include "utilities.h"

/**
 * @brief Constructs a new TargetSelection command.
 *
 * @param target_id The ID of the target to select.
 * @param speed The tracking speed to use.
 * @param run_after The time delay (in microseconds) after which the command should run.
 */
TargetSelection::TargetSelection(uint8_t target_id, int speed, int64_t run_after) :
	Command(run_after), target_id(target_id), speed(speed) {}

/**
 * @brief Executes the target selection command.
 *
 * This method is called by the command processor. It performs the following actions:
 * 1. Sets the specified `target_id` as the current target in the system state.
 * 2. Checks the validity of the newly selected target. If the target has been idle for
 *    too long (e.g., not seen by sensors), it is marked as invalid.
 * 3. Determines the timeout for the next target selection. If the current target is
 *    valid, a longer timeout is used to allow the system to track and engage it. If
 *    invalid, a short timeout is used to quickly cycle to the next potential target.
 * 4. Queues the next `TargetSelection` command for the subsequent target ID, creating a
 *    round-robin evaluation cycle.
 *
 * @param state A pointer to the `SystemState` to be modified.
 */
void TargetSelection::Execute(SystemState* state) {
	state->setTarget(target_id, speed);
	auto currTarget = state->currentTarget();

	// Set a non-zero timeout to prevent a rapid, blocking loop in the test environment
	// when no valid targets are present.
	uint16_t timeout = 10;
	if (currTarget.valid) {
		if (currTarget.idleExceeds(seconds(5))) {
			currTarget.valid = false;
		} else {
			// If target is valid and recently seen, set a longer timeout to track it.
			timeout = 3 * 1000;
		}
	}

	// Queue the next target selection to create a round-robin evaluation
	state->queueSelectTarget(((target_id + 1) % state->size()), timeout);
}
