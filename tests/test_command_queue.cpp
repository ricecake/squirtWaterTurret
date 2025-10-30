#include <iostream>

#include "command_queue.h"
#include "doctest/doctest.h"
#include "mock_time.h"
#include "state.h"
#include "utilities.h"

// Mock command for testing
class MockCommand: public Command {
public:
	MockCommand(int payload, uint64_t run_after): Command(run_after), payload(payload) {}

	void Execute(SystemState*) override { execution_count++; }

	int        payload;
	static int execution_count;
};

int MockCommand::execution_count = 0;

// Helper to reset mock state
void resetMockState() {
	MockCommand::execution_count = 0;
}

TEST_CASE("CommandQueue tests") {
	// Reset mock state before each test
	resetMockState();
	mock_clock.reset();
	CommandQueue queue;
	SystemState  state;
	INFO(queue.serialize());

	SUBCASE("Commands are processed in order") {
		// Add commands with specific run times
		queue.runCommandIn<MockCommand>(10000, 0);
		queue.runCommandIn<MockCommand>(5000, 0);
		queue.runCommandIn<MockCommand>(20000, 0);
		INFO(queue.serialize());

		// Process commands at different time points
		mock_clock.set(4999);
		queue.process(&state);
		INFO(queue.serialize());

		CHECK(MockCommand::execution_count == 0);

		mock_clock.set(5500);
		queue.process(&state);
		INFO(queue.serialize());

		CHECK(MockCommand::execution_count == 1);

		mock_clock.set(10500);
		queue.process(&state);
		INFO(queue.serialize());

		CHECK(MockCommand::execution_count == 2);

		mock_clock.set(20500);
		queue.process(&state);
		INFO(queue.serialize());

		CHECK(MockCommand::execution_count == 3);
	}

	SUBCASE("runCommandIn schedules command correctly") {
		// Schedule a command to run in 100 microseconds
		queue.runCommandIn<MockCommand>(100, 42); // payload = 42

		// Process queue before the scheduled time
		queue.process(&state);
		CHECK(MockCommand::execution_count == 0);

		// Process queue at the scheduled time
		mock_clock += Microseconds(100);
		queue.process(&state);
		CHECK(MockCommand::execution_count == 1);
	}

	SUBCASE("runCommandIn correctly passes arguments") {
		// Add a command with a specific payload
		queue.runCommandIn<MockCommand>(10, 123); // run_after = 10, payload = 123

		// Process the queue
		mock_clock += Microseconds(15);
		queue.process(&state);

		CHECK(MockCommand::execution_count == 1);
	}
}
