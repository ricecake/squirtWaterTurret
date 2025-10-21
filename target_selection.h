/**
 * @file target_selection.h
 * @brief Defines the TargetSelection command class.
 */
#pragma once

#include "command.h"

/**
 * @brief A command to select a target for the system to focus on.
 *
 * This class implements the `Command` interface to schedule a change in the
 * currently selected target. It is used to instruct the system to aim at a
 * specific target identified by its ID, potentially from a radar or CV system.
 */
class TargetSelection: public Command {
public:
	// -- Constructors --
	/**
	 * @brief Constructs a new TargetSelection command.
	 * @param target_id The ID of the target to be selected.
	 * @param speed The tracking speed to use for the selected target.
	 * @param run_after The time in microseconds after which this command should be executed.
	 */
	TargetSelection(uint8_t, int, int64_t);

	// -- Public Methods --
	/**
	 * @brief Executes the target selection command.
	 *
	 * This method is called by the command processor when the command is due.
	 * It validates the target, sets it as the active target in the system state,
	 * and may queue subsequent actions like firing.
	 * @param state A pointer to the current `SystemState`.
	 */
	void Execute(SystemState* state) override;

private:
	// -- Private Attributes --
	uint8_t target_id;    ///< The ID of the target to be selected.
	int     speed = 0xFF; ///< The tracking speed to use for the selected target.
};
