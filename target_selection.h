/**
 * @file target_selection.h
 * @brief Defines the TargetSelection command class.
 *
 * This file contains the declaration of the TargetSelection class, which is a
 * command used to change the currently active target in the SystemState.
 */
#pragma once

#include "command.h"

/**
 * @brief A command to select a target for the system to focus on.
 *
 * This class implements the Command interface to schedule a change in the
 * currently selected target. When executed, it tells the SystemState to aim
 * at a different target from its active target list.
 */
class TargetSelection: public Command {
public:
	// -- Constructors --
	/**
	 * @brief Constructs a new TargetSelection command.
	 * @param id The index of the target to select.
	 * @param speed The speed at which to track the new target.
	 * @param run_after The time in microseconds from now when the command should be executed.
	 */
	TargetSelection(uint8_t id, int speed, int64_t run_after);

	// -- Public Methods --
	/**
	 * @brief Executes the command, changing the system's selected target.
	 * @param state A pointer to the SystemState to be modified.
	 */
	void Execute(SystemState* state) override;

private:
	// -- Private Attributes --
	uint8_t target_id;    ///< The ID or index of the target to be selected.
	int     speed = 0xFF; ///< The tracking speed to use for the selected target.
};
