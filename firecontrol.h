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

public:
	void Execute(SystemState *state);
	FireControl(bool, int64_t);
};
