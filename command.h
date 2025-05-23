#pragma once

#include <stdint.h>
#include "state.h"

class SystemState;

class Command
{
public:
	int64_t id = 0;
	int64_t run_after = 0;

	virtual void Execute(SystemState *state) = 0;
	Command(int64_t run_after);
};
