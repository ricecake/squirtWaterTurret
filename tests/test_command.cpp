#include "command.h"
#include "doctest/doctest.h"
#include "mock_time.h"
#include "state.h"
#include "utilities.h"

// A concrete command for testing purposes
class TestCommand: public Command {
public:
	TestCommand(uint64_t run_after_delay): Command(run_after_delay) {}

	void Execute(SystemState*) override { /* Do nothing */ }
};

TEST_CASE("Command Constructor") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("run_after is calculated correctly") {
		uint64_t delay = 1000; // 1000 microseconds
		mock_clock.set(5000);

		TestCommand cmd(delay);

		// With a deterministic clock, the time should be exact.
		CHECK(cmd.run_after == 5000 + delay);
	}

	SUBCASE("run_after with zero delay") {
		uint64_t delay = 0;
		mock_clock.set(12345);

		TestCommand cmd(delay);

		CHECK(cmd.run_after == 12345);
	}

	SUBCASE("id is set and unique") {
		mock_clock.set(10000);
		TestCommand cmd(100);
		CHECK(cmd.id == 10000000); // nanoSinceEpoch should be precise

		mock_clock.set(10001);
		TestCommand cmd2(100);
		CHECK(cmd2.id > cmd.id);
	}
}
