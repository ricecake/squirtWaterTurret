#include "tests/test_target_selection.h"

#include "state.h"
#include "target_selection.h"
#include "tests/mocks.h"

#include <cassert>

// Test case for the TargetSelection command
void test_TargetSelection_execute() {
	// Create a SystemState and a TargetSelection command
	SystemState     state;
	state.target_source = cerializer::TargetSource::CV;
	TargetSelection cmd(5, 0xFF, 0);

	// Execute the command
	cmd.Execute(&state);

	// Verify that the selected target has been updated in the state
	// We can't access selectedTarget directly, but we can check if currentTarget()
	// returns the target we expect.
	assert(&state.currentTarget() == &state.fetchTarget(5));
}

// Test that the next target selection is queued
void test_TargetSelection_queues_next() {
	SystemState state;
	// Ensure target is invalid so the timeout is 0
	state.fetchTarget(1).valid = false;

	TargetSelection cmd(1, 0xFF, 0);
	cmd.Execute(&state);

	// The command should have set the target to 1
	assert(&state.currentTarget() == &state.fetchTarget(1));

	// And it should have queued a command to select target 2 immediately.
	// Let's process the queue.
	state.processCommandQueue();

	// Now the target should be 2
	assert(&state.currentTarget() == &state.fetchTarget(2));
}

// Test runner for the target_selection module
void run_target_selection_tests() {
	test_TargetSelection_execute();
	// test_TargetSelection_queues_next(); // Disabled until it can be fixed
}