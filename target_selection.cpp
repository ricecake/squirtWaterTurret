/**
 * @file target_selection.cpp
 * @brief Implements the TargetSelection command class.
 *
 * This file contains the method definitions for the TargetSelection command,
 * which is used to change the system's active target.
 */
#include "target_selection.h"

#include "state.h"
#include "utilities.h"

/**
 * @brief Executes the target selection command.
 *
 * This method instructs the SystemState to set the specified target as the current
 * one. It then evaluates the validity of the new target. If the target is valid
 * but has been idle for too long, it is marked as invalid. Based on the validity,
 * it schedules the next target selection command to run, creating a cycle of
// * target evaluation. This round-robin scheduling allows the system to periodically
 * re-evaluate all potential targets.
 *
 * @param state A pointer to the SystemState object to be modified.
 */
void TargetSelection::Execute(SystemState* state) {
	state->setTarget(target_id, speed);
	auto currTarget = state->currentTarget();

	// Set a timeout for the next evaluation. If the target is valid, wait longer.
	// If the target is invalid, we want to cycle to the next one quickly.
	uint16_t timeout = 1;
	if (currTarget.valid) {
		// If a valid target has not been seen for 5 seconds, mark it as invalid.
		if (currTarget.idleExceeds(seconds(5))) {
			currTarget.valid = false;
		} else {
			// If it is valid and recently seen, set a longer timeout.
			timeout = 3 * 1000;
		}
	}

	// Queue the next target selection to create a round-robin evaluation cycle.
	state->queueSelectTarget(((target_id + 1) % state->size()), timeout);
}

/**
 * @brief Constructs a new TargetSelection command.
 *
 * @param target_id The index of the target to be selected in the system's target list.
 * @param speed The tracking speed to use for this target.
 * @param run_after The time delay (in microseconds) from the current time at which
 *                  this command should be executed.
 */
TargetSelection::TargetSelection(uint8_t target_id, int speed, int64_t run_after) :
	Command(run_after), target_id(target_id), speed(speed) {}
