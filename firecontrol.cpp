#include "firecontrol.h"

#include "utilities.h"
#include <stdint.h>
FireControl::FireControl(bool active, uint16_t duration, int64_t run_after) :
	Command(run_after), active(active), duration(duration) {}

/**
 * @brief Executes the fire control command, setting the firing state.
 *
 * This method activates or deactivates the firing pin based on the `active` flag.
 * Before changing the state, it performs several checks:
 * - It ensures the requested firing state is not already the current state.
 * - It enforces a cooldown period (`fireActionInterval`) between firing actions to prevent
 *   rapid toggling.
 * - When deactivating, it waits for the specified `duration` before stopping the firing
 *   and records that a firing action has been completed for the current target.
 *
 * @param state A pointer to the system state, which is modified by this command.
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
