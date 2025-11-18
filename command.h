#pragma once

#include <compare>
#include <cstdint>

#include "shared_types.h"
#include "utilities.h"

class SystemState;

/**
 * @brief Abstract base class for commands in the command pattern.
 *
 * This class defines an interface for executing operations. Each command is a standalone
 * object that encapsulates a request.
 */
class Command {
	inline static uint64_t id_counter = 0;

public:
	// -- Constructors --
	Command(uint64_t run_after = 0);
	virtual ~Command() = default;

	// -- Public Methods --
	virtual void Execute(SystemState* state) = 0;

	virtual constexpr const std::string_view Type() const = 0;

	// -- Public Attributes --
	uint64_t id = 0;        ///< Unique identifier for the command, typically based on a timestamp.
	uint64_t run_after = 0; ///< The time at which the command should be executed.
};

template <typename Derived>
class AutoCommand: virtual public Command {
public:
	constexpr const std::string_view Type() const override {
		// The string is generated once at compile time for this type
		return type_name<Derived>();
	}
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

class SetStrategyCommand: virtual public Command, public AutoCommand<SetStrategyCommand> {
public:
	SetStrategyCommand(TurretStrategy strategy, uint64_t run_after);
	void Execute(SystemState* state) override;

private:
	TurretStrategy strategy;
};

class SetStanceCommand: virtual public Command, public AutoCommand<SetStanceCommand> {
public:
	SetStanceCommand(TurretStance stance, uint64_t run_after);
	void Execute(SystemState* state) override;

private:
	TurretStance stance;
};
