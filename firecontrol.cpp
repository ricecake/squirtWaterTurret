#include "firecontrol.h"

FireControl::FireControl(bool active, int64_t run_after) : Command(run_after), active(active)
{
}
void FireControl::Execute(SystemState *state)
{
	state->setFire(active);

	Target &curr = state->currentTarget();
	curr.IncrementAction();
}

