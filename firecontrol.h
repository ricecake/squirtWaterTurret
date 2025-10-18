#pragma once

#include <cstdint>
#include <functional>
#include <queue>

#ifdef ARDUINO
	#include <AccelStepper.h>
#else
	#include "tests/mocks.h"
#endif

#include "command.h"
#include "state.h"

/**
 * @brief A command to control the firing mechanism.
 *
 * This class implements the Command interface to activate or deactivate the firing pin.
 * It is used to schedule firing events at specific times.
 */
class FireControl : public Command {
public:
	// -- Constructors --
	FireControl(bool, uint16_t, int64_t);

	// -- Public Methods --
	void Execute(SystemState* state) override;

private:
	// -- Private Attributes --
	bool     active;   ///< The desired state of the firing pin (true for active, false for inactive).
	uint16_t duration; ///< The duration for which the firing state should be maintained.
};
