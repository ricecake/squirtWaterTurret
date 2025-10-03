#pragma once

#include "command.h"

/**
 * @brief A command to select a target for the system to focus on.
 *
 * This class implements the Command interface to schedule a change in the
 * currently selected target. The radar identifies potential targets, and this
 * command is used to specify which one to track and engage.
 */
class TargetSelection : public Command
{
	uint8_t target_id; ///< The ID of the target to be selected.
	int speed = 0xFF;  ///< The tracking speed to use for the selected target.

public:
    /**
     * @brief Executes the target selection command.
     * @param state A pointer to the system state.
     */
	void Execute(SystemState *state);

    /**
     * @brief Constructs a new TargetSelection object.
     * @param target_id The ID of the target to select.
     * @param speed The tracking speed to use.
     * @param run_after The time delay (in microseconds) after which the command should run.
     */
	TargetSelection(uint8_t, int, int64_t);
};

