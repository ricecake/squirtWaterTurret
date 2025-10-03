#include "utilities.h"
#include <stdint.h>
#include "firecontrol.h"

FireControl::FireControl(bool active, uint16_t duration, int64_t run_after) : Command(run_after), active(active), duration(duration)
{
}
void FireControl::Execute(SystemState *state)
{
	Target &target = state->currentTarget();

	if (state->getFireState() == active) {
		return;
	}

	if (active && target.actionIdleExceeds(fireActionInterval))
	{
		state->setFire(active);
	}
	else if (!active && target.actionIdleExceeds(milliseconds(duration, fireActionInterval)))
	{
		state->setFire(active);
		target.IncrementAction();
	}
}
