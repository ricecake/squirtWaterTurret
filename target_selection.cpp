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
			// Serial.print("Tracking target ");
			// Serial.print(target_id);
			// Serial.print(":::");
			// Serial.print(currTarget.id);
			// Serial.print(":::");
			// Serial.print(float(currTarget.Position().X_coord));
			// Serial.print(":::");
			// Serial.print(float(currTarget.Position().Y_coord));
			// Serial.print(":::");
			// Serial.print(float(currTarget.Position().Z_coord));
			// Serial.println();
		}
	}

	state->queueSelectTarget(((target_id + 1) % state->size()), timeout);
}
TargetSelection::TargetSelection(uint8_t target_id, int speed, int64_t run_after) : Command(run_after), target_id(target_id), speed(speed)
{
}
