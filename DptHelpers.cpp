#include <Arduino.h>
#include "DptHelpers.h"

void SystemState::queueFire(uint8_t milliseconds)
{
}

void SystemState::queueLinger(uint8_t milliseconds)
{
}

bool Command::operator<(const Command &other) const
{
	return this->run_after < other.run_after;
}

void MoveCommand::Execute(SystemState &state)
{
}

Command::Command(int64_t run_after = 0) : run_after(run_after)
{
	id = esp_timer_get_time();
}
MoveCommand::MoveCommand(long pitch, long yaw, int speed, int64_t run_after) : Command(run_after), pitch(pitch), yaw(yaw), speed(speed)
{
}