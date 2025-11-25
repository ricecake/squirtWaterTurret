#include "firecontrol.h"

#include <cstdint>
#include <functional>

#include "utilities.h"

/**
 * @brief Constructs a new FireControl object.
 *
 * @param active The desired state of the firing pin.
 * @param duration The duration for the firing state.
 * @param run_after The time delay (in microseconds) after which the command should run.
 */
FireControl::FireControl(bool active, uint16_t duration, int64_t run_after):
	Command(run_after), active(active), duration(duration) {}

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
	// Do nothing if the desired fire state is already the current state
	if (state->getFireState() == active) {
		return;
	}

	Target* target = state->currentTarget();
	auto    actionable = target->actionable();

	// Activate firing if the target has been idle longer than the action interval
	if (active && actionable) {
		state->setFire(active);
		state->queueCeaseFire(duration);
	}
	// Deactivate firing after the specified duration and increment the action counter
	else if (!active && target->actionIdleExceeds(milliseconds(duration))) {
		state->clearFire(active);
		target->IncrementAction();
	}
}
