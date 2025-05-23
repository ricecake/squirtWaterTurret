#include "command.h"
#include <esp_timer.h>

Command::Command(int64_t run_after)
{
	id = esp_timer_get_time();
	this->run_after = id + run_after;
}

