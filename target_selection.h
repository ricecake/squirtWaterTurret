#pragma once

#include "command.h"
#include "shared_types.h"

/**
 * @brief A command to select a target for the system to focus on.
 *
 * This class implements the Command interface to schedule a change in the
 * currently selected target. The radar identifies potential targets, and this
 * command is used to specify which one to track and engage.
 */
class TargetSelection: virtual public Command, public AutoCommand<TargetSelection> {
public:
	// -- Constructors --
	TargetSelection(TargetSource, uint8_t, uint8_t, uint64_t);

	// -- Public Methods --
	void Execute(SystemState* state) override;

private:
	// -- Private Attributes --
	TargetSource target_source;
	uint8_t      target_id;    ///< The ID of the target to be selected.
	uint8_t      speed = 0xFF; ///< The tracking speed to use for the selected target.
};
