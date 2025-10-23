/**
 * @file command.h
 * @brief Defines the base Command class and related command types.
 *
 * This file contains the abstract base class for the command pattern,
 * which encapsulates a request as an object. It also defines concrete
 * command implementations like `LingerCommand`.
 */
#pragma once

#include <compare>
#include <cstdint>

#include "state.h"

class SystemState;

/**
 * @brief Abstract base class for commands in the command pattern.
 *
 * This class defines an interface for executing operations. Each command is a standalone
 * object that encapsulates a request. Commands are stored in a priority queue
 * and executed based on their `run_after` timestamp.
 */
class Command {
public:
	// -- Constructors --
	/**
	 * @brief Constructs a new Command.
	 * @param run_after The time in microseconds from the epoch after which this command should be run.
	 */
	Command(int64_t run_after);
	/**
	 * @brief Virtual destructor for the base class.
	 */
	virtual ~Command() = default;

	// -- Public Methods --
	/**
	 * @brief Pure virtual function to execute the command's action.
	 *
	 * Derived classes must implement this method to perform their specific action.
	 * @param state A pointer to the SystemState, allowing the command to modify the system's state.
	 */
	virtual void Execute(SystemState* state) = 0;

	// -- Public Attributes --
	int64_t id = 0;        ///< Unique identifier for the command, typically based on a timestamp.
	int64_t run_after = 0; ///< The time in microseconds from the epoch at which the command should be executed.
};

/**
 * @brief A command that does nothing, effectively creating a delay.
 *
 * When this command is executed, it performs no action, but its presence in the
 * command queue can be used to introduce a pause in a sequence of operations.
 */
class LingerCommand: public Command {
public:
	// -- Constructors --
	/**
	 * @brief Constructs a new LingerCommand.
	 * @param run_after The time in microseconds from the epoch after which this command should be "run" (i.e., the pause ends).
	 */
	LingerCommand(int64_t run_after);

	// -- Public Methods --
	/**
	 * @brief Executes the linger command, which does nothing.
	 * @param state A pointer to the SystemState (unused in this implementation).
	 */
	void Execute(SystemState* state) override;
};

// ======================================================================================
// --- Operator Overloads ---
// ======================================================================================

/**
 * @brief Spaceship operator overload for comparing command execution times.
 *
 * This allows `Command` objects (and pointers to them in the priority queue)
 * to be sorted based on their `run_after` timestamp. Commands scheduled to run
 * earlier are considered "less" than commands scheduled to run later.
 *
 * @param left  The left-hand side command.
 * @param right The right-hand side command.
 * @return A `std::weak_ordering` indicating which command should run first.
 */
constexpr auto operator<=>(const Command& left, const Command& right) {
	if (left.run_after < right.run_after) {
		return std::weak_ordering::less;
	} else if (left.run_after > right.run_after) {
		return std::weak_ordering::greater;
	} else {
		return std::weak_ordering::equivalent;
	}
}
