/**
 * @file firecontrol.cpp
 * @brief Implements the FireControl command class.
 *
 * This file contains the method definitions for the FireControl command,
 * which is responsible for activating and deactivating the firing mechanism.
 */
#include "firecontrol.h"

#include "utilities.h"
#include <stdint.h>

/**
 * @brief Constructs a new FireControl command object.
 *
 * @param active The desired state of the firing pin (`true` to fire, `false` to stop).
 * @param duration The duration for which the firing state should be maintained.
 * @param run_after The time delay (in microseconds) from the current time after which
 *                  the command should be executed.
 */
FireControl::FireControl(bool active, uint16_t duration, int64_t run_after) :
	Command(run_after), active(active), duration(duration) {}

/**
 * @brief Executes the fire control command, setting the system's firing state.
 *
 * This method contains the logic for whether to fire or not. It checks if the
 * desired state (`active`) is already the current state in the `SystemState`.
 * When activating the firing pin (`active` is true), it checks if enough time has
 * passed since the last action on the current target to prevent rapid re-firing.
 * When deactivating, it also checks a time condition and then increments the
 * target's action counter to reset the firing cooldown.
 *
 * @param state A pointer to the `SystemState` whose `fireState` will be modified.
 */
void FireControl::Execute(SystemState* state) {
	Target& target = state->currentTarget();

	// Do nothing if the desired fire state is already the current state.
	if (state->getFireState() == active) {
		return;
	}

	// Logic to activate the firing pin.
	// Check if enough time has passed since the last action on this target.
	if (active && target.actionIdleExceeds(fireActionInterval)) {
		state->setFire(active);
	}
	// Logic to deactivate the firing pin.
	// Check if enough time has passed since the last action, including the firing duration.
	else if (!active && target.actionIdleExceeds(milliseconds(duration, fireActionInterval))) {
		state->setFire(active);
		target.IncrementAction(); // Mark that an action has been completed on this target.
	}
}
