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
    TestClock::ScopedDeterministicClock det_clock;
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
        auto scheduled_time = microSinceEpoch() + 100;
		queue.runCommandIn<MockCommand>(100, 42); // payload = 42

		// Process queue before the scheduled time
		queue.process(&state);
		CHECK(MockCommand::execution_count == 0);

		// Process queue at the scheduled time
		mock_clock.set(scheduled_time);
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

    SUBCASE("addCommand schedules command correctly") {
        mock_clock.set(1000);
        auto scheduled_time = microSinceEpoch() + 50;
        // Add a command with a 50 microsecond delay
        queue.addCommand<MockCommand>(/*payload=*/77, /*run_after_delay=*/50);

        queue.process(&state);
        CHECK(MockCommand::execution_count == 0);

        mock_clock.set(scheduled_time - 1);
        queue.process(&state);
        CHECK(MockCommand::execution_count == 0);

        mock_clock.set(scheduled_time);
        queue.process(&state);
        CHECK(MockCommand::execution_count == 1);
    }

    SUBCASE("addCommandAfter schedules command after the last one") {
        mock_clock.set(1000);
        // Schedule a command to run at time 3000
        auto first_run_time = microSinceEpoch() + 2000;
        queue.runCommandIn<MockCommand>(2000, /*payload=*/1);

        mock_clock.set(1500);
        // Schedule another command to run after the previous one.
        queue.addCommandAfter<MockCommand>(/*payload=*/2);

        // Process just before the first command
        mock_clock.set(first_run_time - 1);
        queue.process(&state);
        CHECK(MockCommand::execution_count == 0);

        // Process at the time of the first command
        mock_clock.set(first_run_time);
        queue.process(&state);
        CHECK(MockCommand::execution_count == 1);

        // Process just before the second command
        mock_clock.set(first_run_time);
        queue.process(&state);
        CHECK(MockCommand::execution_count == 1);

        // Process at the time of the second command
        mock_clock.set(first_run_time + 1);
        queue.process(&state);
        CHECK(MockCommand::execution_count == 2);
    }
}
