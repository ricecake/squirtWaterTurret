#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "command.h"
#include "state.h"

class FireControl : public Command
{
	bool active;
	uint16_t duration;

public:
	void Execute(SystemState *state);
	FireControl(bool, uint16_t, int64_t);
};
