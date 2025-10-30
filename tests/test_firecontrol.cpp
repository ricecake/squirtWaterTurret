#include "tests/test_firecontrol.h"

#include <chrono>

#include "doctest/doctest.h"
#include "firecontrol.h"
#include "state.h"
#include "tests/mocks.h"
#include "utilities.h"

// Test case for activating the firing mechanism
TEST_CASE("FireControl activate") {
	SystemState state;
	state.setFire(false); // Ensure initial state is off
	// Target& target = state.currentTarget(); // Unused but kept for clarity

	// To make actionIdleExceeds true, we can just not set last_action
	// Or set it to a time in the past. The default constructor of Target sets it to epoch.
	// We need to wait a bit to make sure the time difference is large enough.
	mock_clock += Seconds(50);

	FireControl cmd(true, 10, 0);
	cmd.Execute(&state);

	CHECK(state.getFireState() == true);
}

// Test case for deactivating the firing mechanism
TEST_CASE("FireControl deactivate") {
	SystemState state;
	state.setFire(true); // Ensure initial state is on
	// Target& target = state.currentTarget(); // Unused.
	// By not calling IncrementAction(), the target's last_action remains in the
	// distant past, allowing the deactivation check to pass.

	// Wait for a duration longer than the command's duration
	mock_clock += Seconds(20);

	FireControl cmd(false, 10, 0);
	cmd.Execute(&state);

	CHECK(state.getFireState() == false);
}

// Test case to ensure firing doesn't happen if already active
TEST_CASE("FireControl already active") {
	SystemState state;
	state.setFire(true);

	FireControl cmd(true, 10, 0);
	cmd.Execute(&state);

	CHECK(state.getFireState() == true);
}

// Test case to ensure no change if already inactive
TEST_CASE("FireControl already inactive") {
	SystemState state;
	state.setFire(false);

	FireControl cmd(false, 10, 0);
	cmd.Execute(&state);

	CHECK(state.getFireState() == false);
}

// Test case for when action idle time has not been exceeded
TEST_CASE("FireControl activation too soon") {
	SystemState state;
	state.setFire(false);
	Target& target = state.currentTarget();
	target.IncrementAction(); // Set last action time to now

	FireControl cmd(true, 10, 0);
	cmd.Execute(&state);

	CHECK(state.getFireState() == false);
}
