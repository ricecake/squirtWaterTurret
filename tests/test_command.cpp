#include "tests/test_command.h"

#include <cassert>

#include "command.h"
#include "state.h"
#include "tests/mocks.h"

// Test case for the LingerCommand
#include "tests/mocks.h"

void test_LingerCommand_initialization() {
	// Create a LingerCommand with a specific run_after value
	advance_mock_time(1);
	LingerCommand cmd(100);
	int64_t       now = esp_timer_get_time();

	// Verify that the run_after is initialized correctly.
	// It should be roughly now + 100.
	assert(cmd.run_after >= now);
}

void test_LingerCommand_execute() {
	// Create a SystemState and a LingerCommand
	SystemState   state;
	LingerCommand cmd(0);

	// Execute the command
	cmd.Execute(&state);

	// Verify that the state remains unchanged
	// The default selected target is 0. LingerCommand should not change it.
	assert(&state.currentTarget() == &state.fetchTarget(0));
}

// Test runner for the command module
void run_command_tests() {
	test_LingerCommand_initialization();
	test_LingerCommand_execute();
}