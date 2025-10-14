#pragma once

#include <functional>
#include <queue>
#include <stdint.h>

#include <AccelStepper.h>

#include "command.h"
#include "state.h"

/**
 * @brief A command to control the firing mechanism.
 *
 * This class implements the Command interface to activate or deactivate the firing pin.
 * It is used to schedule firing events at specific times.
 */
class FireControl: public Command {
	bool     active;    ///< The desired state of the firing pin (true for active, false for inactive).
	uint16_t duration;  ///< The duration for which the firing state should be maintained.

public:
	/**
	 * @brief Executes the fire control command.
	 * @param state A pointer to the system state.
	 */
	void Execute(SystemState* state);

	/**
	 * @brief Constructs a new FireControl object.
	 * @param active The desired state of the firing pin.
	 * @param duration The duration for the firing state.
	 * @param run_after The time delay (in microseconds) after which the command should run.
	 */
	FireControl(bool, uint16_t, int64_t);
};
