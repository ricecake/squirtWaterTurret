#include "target_selection.h"

void TargetSelection::Execute(SystemState *state)
{
	state->setTarget(target_id, speed);
	auto currTarget = state->currentTarget();

	uint16_t timeout = 0;
	if (currTarget.valid)
	{
		if (currTarget.idleExceeds(seconds(5))) {
			currTarget.valid = false;
		}
		else {
			timeout = 3 * 1000;
		}
	}

	state->queueSelectTarget(((target_id + 1) % state->size()), timeout);
}
TargetSelection::TargetSelection(uint8_t target_id, int speed, int64_t run_after) : Command(run_after), target_id(target_id), speed(speed)
{
}
