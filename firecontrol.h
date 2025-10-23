/**
 * @file firecontrol.h
 * @brief Defines the FireControl command class.
 *
 * This file contains the declaration for the `FireControl` class, a command
 * used to schedule the activation and deactivation of the firing mechanism.
 */
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
 * It is used to schedule firing events at specific times. When executed, it checks
 * if the target is valid and within range before activating the firing pin.
 */
class FireControl: public Command {
public:
	// -- Constructors --
	/**
	 * @brief Constructs a FireControl command.
	 *
	 * @param active The desired state of the firing pin (`true` to fire, `false` to stop).
	 * @param duration The duration (in microseconds) to maintain the firing state. This parameter is
	 *                 currently unused but is intended for future enhancements.
	 * @param run_after The time delay (in microseconds) from the current time after which the
	 *                  command should be executed.
	 */
	FireControl(bool active, uint16_t duration, int64_t run_after);

	// -- Public Methods --
	/**
	 * @brief Executes the fire control command.
	 *
	 * This method sets the `fireState` in the `SystemState` to the command's `active` state.
	 * It performs a check to ensure the target is valid and the turret is aimed correctly
	 * before activating the firing pin.
	 *
	 * @param state A pointer to the `SystemState` to be modified.
	 */
	void Execute(SystemState* state) override;

private:
	// -- Private Attributes --
	bool     active;   ///< The desired state of the firing pin (`true` for active, `false` for inactive).
	uint16_t duration; ///< The duration for which the firing state should be maintained (currently unused).
};
