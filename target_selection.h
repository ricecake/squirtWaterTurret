#pragma once

#include "command.h"

class TargetSelection : public Command
{
	uint8_t target_id;
	int speed = 0xFF;

public:
	void Execute(SystemState *state);
	TargetSelection(uint8_t, int, int64_t);

	// This should actually just be speed and target index.
	// The radar will set the targets as it finds them, and then we schedule which one we're interested in.
};

