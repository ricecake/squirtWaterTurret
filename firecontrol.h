#pragma once

#include <cstdint>

#include "command.h"
#include "state.h"

/**
 * @brief A command to control the firing mechanism.
 *
 * This class implements the Command interface to activate or deactivate the firing pin.
 * It is used to schedule firing events at specific times.
 */
class FireControl: virtual public Command, public AutoCommand<FireControl> {
public:
	// -- Constructors --
	FireControl(bool initial_active, uint16_t initial_duration, int64_t initial_run_after);

	// -- Public Methods --
	void Execute(SystemState* state) override;

private:
	// -- Private Attributes --
	bool     active;   ///< The desired state of the firing pin (true for active, false for inactive).
	uint16_t duration; ///< The duration for which the firing state should be maintained.
};
