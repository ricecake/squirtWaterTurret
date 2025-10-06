#pragma once

#include <stdint.h>
#include "state.h"
#include <compare>

class SystemState;

/**
 * @brief Abstract base class for commands in the command pattern.
 *
 * This class defines an interface for executing operations. Each command is a standalone
 * object that encapsulates a request.
 */
class Command
{
public:
	virtual ~Command() = default;
	/// @brief Unique identifier for the command, typically based on a timestamp.
	int64_t id = 0;

	/// @brief The time at which the command should be executed.
	int64_t run_after = 0;

	/**
	 * @brief Executes the command.
	 *
	 * This is a pure virtual function that must be implemented by derived classes.
	 * @param state A pointer to the system state, allowing the command to interact with the system.
	 */
	virtual void Execute(SystemState *state) = 0;

	/**
	 * @brief Constructs a new Command object.
	 * @param run_after The time delay (in microseconds) after which the command should run.
	 */
	Command(int64_t run_after);
};

/// @brief Spaceship overload for command pointers
/// @param  left command
/// @param  right command
/// @return ordering
constexpr auto operator<=>(const Command &left, const Command &right)
{
	if (left.run_after > right.run_after)
	{
		return std::weak_ordering::less;
	}
	else if (left.run_after < right.run_after)
	{
		return std::weak_ordering::greater;
	}
	else
	{
		return std::weak_ordering::equivalent;
	}
}

/**
 * @brief A command that does nothing, effectively creating a delay.
 *
 * This command is used to introduce a pause in the command queue processing.
 */
class LingerCommand : public Command
{
public:
	/**
	 * @brief Executes the linger command (does nothing).
	 * @param state A pointer to the system state.
	 */
	void Execute(SystemState *state) override;

	/**
	 * @brief Constructs a new LingerCommand object.
	 * @param run_after The time delay (in microseconds) after which the command should run.
	 */
	LingerCommand(int64_t run_after);
};
