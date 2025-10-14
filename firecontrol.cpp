#include "firecontrol.h"

#include "utilities.h"

#include <stdint.h>
/**
 * @brief Constructs a new FireControl object.
 *
 * @param active The desired state of the firing pin.
 * @param duration The duration for the firing state.
 * @param run_after The time delay (in microseconds) after which the command should run.
 */
FireControl::FireControl(bool active, uint16_t duration, int64_t run_after) :
	Command(run_after),
	active(active),
	duration(duration) {
}

/**
 * @brief Executes the fire control command, setting the firing state.
 *
 * This method checks conditions before changing the firing state, such as whether
 * the requested state is already active and if enough time has passed since the
 * last action.
 *
 * @param state A pointer to the system state.
 */
void FireControl::Execute(SystemState* state) {
	Target& target = state->currentTarget();

	// Do nothing if the desired fire state is already the current state
	if (state->getFireState() == active) {
		return;
	}

	// Activate firing if the target has been idle longer than the action interval
	if (active && target.actionIdleExceeds(fireActionInterval)) {
		state->setFire(active);
	}
	// Deactivate firing after the specified duration and increment the action counter
	else if (!active && target.actionIdleExceeds(milliseconds(duration, fireActionInterval))) {
		state->setFire(active);
		target.IncrementAction();
	}
}
