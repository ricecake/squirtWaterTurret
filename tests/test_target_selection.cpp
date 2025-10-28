#include <ostream>
#include "doctest.h"
#include "state.h"
#include "target_selection.h"
#include "tests/mocks.h"

// Test case for the TargetSelection command
TEST_CASE("TargetSelection execute") {
	// Create a SystemState and a TargetSelection command
	SystemState state;
	state.target_source = cerializer::TargetSource::CV;
	TargetSelection cmd(5, 0xFF, 0);

	// Execute the command
	cmd.Execute(&state);

	// Verify that the selected target has been updated in the state
	// We can't access selectedTarget directly, but we can check if currentTarget()
	// returns the target we expect.
	REQUIRE(&state.currentTarget() == &state.fetchTarget(5));
}

// Test that the next target selection is queued
TEST_CASE("TargetSelection queues next") {
	SystemState state;
	state.target_source = cerializer::TargetSource::CV;
	// Ensure target is invalid so the timeout is 0
	state.fetchTarget(1).valid = false;

	TargetSelection cmd(1, 0xFF, 0);
	cmd.Execute(&state);

	// The command should have set the target to 1
	REQUIRE(&state.currentTarget() == &state.fetchTarget(1));

	// And it should have queued a command to select target 2 immediately.
	// Let's process the queue.
	for (int i = 0; i < 10 && &state.currentTarget() != &state.fetchTarget(2); ++i) {
		advance_mock_time(1000);
		state.processCommandQueue();
	}

	// Now the target should be 2
	REQUIRE(&state.currentTarget() == &state.fetchTarget(2));
}
