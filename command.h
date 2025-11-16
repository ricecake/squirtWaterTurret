#pragma once

#include <compare>
#include <cstdint>

#include "shared_types.h"

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
	virtual void      Execute(SystemState* state) = 0;
	virtual uint8_t Code() = 0;

	// -- Public Attributes --
	uint64_t id = 0;        ///< Unique identifier for the command, typically based on a timestamp.
	uint64_t run_after = 0; ///< The time at which the command should be executed.
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

class SetStrategyCommand: public Command {
public:
	SetStrategyCommand(TurretStrategy strategy, uint64_t run_after);
	void    Execute(SystemState* state) override;
	uint8_t Code() override { return 4; };

private:
	TurretStrategy strategy;
};

class SetStanceCommand: public Command {
public:
	SetStanceCommand(TurretStance stance, uint64_t run_after);
	void    Execute(SystemState* state) override;
	uint8_t Code() override { return 5; };

private:
	TurretStance stance;
};

class SetTargetSourceCommand: public Command {
public:
	SetTargetSourceCommand(TargetSource source, uint64_t run_after);
	void    Execute(SystemState* state) override;
	uint8_t Code() override { return 2; };

private:
	TargetSource source;
};
