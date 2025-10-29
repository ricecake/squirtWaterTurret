#include "command_queue.h"
#include "doctest/doctest.h"
#include "state.h"
#include "utilities.h"

#include <iostream>

// Mock command for testing
class MockCommand : public Command {
public:
	using Command::Command; // Inherit constructors

	void execute(SystemState& state) override {
		execution_count++;
		last_executed_at = Clock::now();
		state.test_value = payload;
	}

	int         payload = 0;
	static int  execution_count;
	static TimePoint last_executed_at;
};

int       MockCommand::execution_count = 0;
TimePoint MockCommand::last_executed_at;

// Helper to reset mock state
void resetMockState() {
	MockCommand::execution_count = 0;
	MockCommand::last_executed_at = TimePoint();
}

// Set up a mock clock
static TimePoint mock_time;
void             setMockTime(TimePoint t) {
	mock_time = t;
}
TimePoint getMockTime() {
	return mock_time;
}

TEST_CASE("CommandQueue tests") {
	CommandQueue queue;
	SystemState  state;

	// Set up the mock clock
	Clock::setClock(getMockTime);
	setMockTime(TimePoint(seconds(0)));

	// Reset mock state before each test
	resetMockState();

	SUBCASE("Commands are processed in order") {
		// Add commands with specific run times
		queue.addCommand<MockCommand>(100);
		queue.addCommand<MockCommand>(50);
		queue.addCommand<MockCommand>(200);

		// Process commands at different time points
		setMockTime(TimePoint(microseconds(49)));
		queue.process(&state);
		CHECK(MockCommand::execution_count == 0);

		setMockTime(TimePoint(microseconds(50)));
		queue.process(&state);
		CHECK(MockCommand::execution_count == 1);
		CHECK(MockCommand::last_executed_at == TimePoint(microseconds(50)));

		setMockTime(TimePoint(microseconds(150)));
		queue.process(&state);
		CHECK(MockCommand::execution_count == 2);
		CHECK(MockCommand::last_executed_at == TimePoint(microseconds(150)));

		setMockTime(TimePoint(microseconds(201)));
		queue.process(&state);
		CHECK(MockCommand::execution_count == 3);
	}

	SUBCASE("runCommandIn schedules command correctly") {
		// Schedule a command to run in 100 microseconds
		queue.runCommandIn<MockCommand>(100, 42); // payload = 42

		// Process queue before the scheduled time
		setMockTime(TimePoint(microseconds(99)));
		queue.process(&state);
		CHECK(MockCommand::execution_count == 0);

		// Process queue at the scheduled time
		setMockTime(TimePoint(microseconds(100)));
		queue.process(&state);
		CHECK(MockCommand::execution_count == 1);
		CHECK(state.test_value == 42);
	}

	SUBCASE("addCommand correctly passes arguments") {
		// Add a command with a specific payload
		queue.addCommand<MockCommand>(10, 123); // run_after = 10, payload = 123

		// Process the queue
		setMockTime(TimePoint(microseconds(10)));
		queue.process(&state);

		CHECK(MockCommand::execution_count == 1);
		CHECK(state.test_value == 123);
	}
}
