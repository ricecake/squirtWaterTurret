#include <ostream>
#include "doctest.h"
#include "command.h"
#include "state.h"
#include "tests/mocks.h"

TEST_CASE("LingerCommand initialization") {
	// Create a LingerCommand with a specific run_after value
	advance_mock_time(1);
	LingerCommand cmd(100);
	int64_t       now = esp_timer_get_time();

	// Verify that the run_after is initialized correctly.
	// It should be roughly now + 100.
	REQUIRE(cmd.run_after >= now);
}

TEST_CASE("LingerCommand execute") {
	// Create a SystemState and a LingerCommand
	SystemState   state;
	LingerCommand cmd(0);

	// Execute the command
	cmd.Execute(&state);

	// Verify that the state remains unchanged
	// The default selected target is 0. LingerCommand should not change it.
	REQUIRE(&state.currentTarget() == &state.fetchTarget(0));
}
