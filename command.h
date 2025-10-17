#pragma once

#include <compare>
#include <cstdint>

#include "state.h"

class SystemState;

/**
 * @brief Abstract base class for commands in the command pattern.
 *
 * This class defines an interface for executing operations. Each command is a standalone
 * object that encapsulates a request.
 */
class Command {
public:
	// -- Constructors --
	Command(int64_t run_after);
	virtual ~Command() = default;

	// -- Public Methods --
	virtual void Execute(SystemState* state) = 0;

	// -- Public Attributes --
	int64_t id = 0;        ///< Unique identifier for the command, typically based on a timestamp.
	int64_t run_after = 0; ///< The time at which the command should be executed.
};


/**
 * @brief A command that does nothing, effectively creating a delay.
 *
 * This command is used to introduce a pause in the command queue processing.
 */
class LingerCommand : public Command {
public:
	// -- Constructors --
	LingerCommand(int64_t run_after);

	// -- Public Methods --
	void Execute(SystemState* state) override;
};


// ======================================================================================
// --- Operator Overloads ---
// ======================================================================================

/**
 * @brief Spaceship overload for comparing command execution times.
 * @param left  The left-hand side command.
 * @param right The right-hand side command.
 * @return A weak ordering indicating which command should run first.
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
