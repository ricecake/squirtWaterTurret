#include "doctest/doctest.h"
#include "state.h"
#include "target_selection.h"
#include "tests/mocks.h"
#include <ranges>

// Test case for the TargetSelection command
TEST_CASE("TargetSelection execute") {
	// Create a SystemState and a TargetSelection command
	SystemState state;
	state.target_source = TargetSource::CV;
	TargetSelection cmd(state.target_source, 5, 0xFF, 0);

	// Execute the command
	cmd.Execute(&state);

	// Verify that the selected target has been updated in the state
	// We can't access selectedTarget directly, but we can check if currentTarget()
	// returns the target we expect.
	CHECK(&state.currentTarget() == &state.fetchTarget(5));
}

// Test that the next target selection is queued
TEST_CASE("TargetSelection queues next") {
	SystemState state;
	state.target_source = TargetSource::CV;
	// Ensure target is invalid so the timeout is 0
	state.fetchTarget(1).valid = false;

	TargetSelection cmd(state.target_source, 1, 0xFF, 0);
	cmd.Execute(&state);

	// The command should have set the target to 1
	CHECK(&state.currentTarget() == &state.fetchTarget(1));

	// And it should have queued a command to select target 2 immediately.
	// Let's process the queue.
	mock_clock += microseconds(2000).get_duration();
	state.processCommandQueue();

	// Now the target should be 2
	CHECK(&state.currentTarget() == &state.fetchTarget(2));
}

// Test that a valid target queues the next selection with a timeout
TEST_CASE("TargetSelection queues next with timeout for valid target") {
	SystemState state;
	state.target_source = TargetSource::CV;
	mock_clock.reset();

	// Make the target valid and not idle
	Target& target = state.fetchTarget(3);
	target.valid = true;
	target.IncrementAction();

	TargetSelection cmd(state.target_source, 3, 0xFF, 0);
	cmd.Execute(&state);

	// Target 3 should now be selected
	CHECK(&state.currentTarget() == &state.fetchTarget(3));

	// Process the queue. The next command (for target 4) should be queued
	// but not executed yet because of the timeout.
	state.processCommandQueue();
	CHECK(&state.currentTarget() == &state.fetchTarget(3));

	// Advance time just before the timeout expires
	mock_clock += milliseconds(2999).get_duration();
	state.processCommandQueue();
	CHECK(&state.currentTarget() == &state.fetchTarget(3));

	// Advance time past the timeout
	mock_clock += milliseconds(2).get_duration();
	state.processCommandQueue();
	CHECK(&state.currentTarget() == &state.fetchTarget(4));
}

// Test that an idle valid target is marked as invalid
TEST_CASE("TargetSelection invalidates idle target") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;
	SystemState                         state;
	state.target_source = TargetSource::CV;

	// Make the target valid, but leave its last_action time at epoch
	Target& target = state.fetchTarget(7);
	target.valid = true;

	// Advance the clock to make it idle
	mock_clock += seconds(10).get_duration();

	TargetSelection cmd(state.target_source, 7, 0xFF, 0);
	cmd.Execute(&state);

	// The target should now be marked as invalid
	CHECK(state.fetchTarget(7).valid == false);

	// And the next command (for target 8) should be queued with no timeout
	mock_clock += microseconds(2000).get_duration();
	state.processCommandQueue();
	CHECK(&state.currentTarget() == &state.fetchTarget(8));
}

TEST_CASE("findBestTarget selects correct target for each strategy") {
	SystemState state;
	state.target_source = TargetSource::CV;

	// Set up targets with varying properties
	state.fetchTarget(1).valid = true;
	state.fetchTarget(1).last_action = TimePoint(microseconds(100).get_duration());
	state.fetchTarget(1).action_count = 5;
	state.fetchTarget(1).Update(PositionVector(10, 5, 0)); // Off-axis

	state.fetchTarget(2).valid = true;
	state.fetchTarget(2).last_action = TimePoint(microseconds(200).get_duration());
	state.fetchTarget(2).action_count = 2;
	state.fetchTarget(2).Update(PositionVector(20, 2, 0)); // Off-axis

	state.fetchTarget(3).valid = true;
	state.fetchTarget(3).last_action = TimePoint(microseconds(50).get_duration());
	state.fetchTarget(3).action_count = 10;
	state.fetchTarget(3).Update(PositionVector(5, 10, 0));  // Off-axis

	// --- Test each strategy ---

	state.setStrategy(TurretStrategy::LEAST_HIT);
	CHECK(state.findBestTarget() == &state.fetchTarget(2));

	state.setStrategy(TurretStrategy::MOST_HIT);
	CHECK(state.findBestTarget() == &state.fetchTarget(3));

	state.setStrategy(TurretStrategy::CLOSEST);
	CHECK(state.findBestTarget() == &state.fetchTarget(1));

	state.setStrategy(TurretStrategy::FURTHEST);
	CHECK(state.findBestTarget() == &state.fetchTarget(2));

	state.setStrategy(TurretStrategy::LEAST_RECENT);
	CHECK(state.findBestTarget() == &state.fetchTarget(3));

	state.setStrategy(TurretStrategy::MOST_RECENT);
	CHECK(state.findBestTarget() == &state.fetchTarget(2));

	// For travel-based strategies, we need to set the current position
	state.stepperA.moveTo(0);
	state.stepperA.run();
	state.stepperB.moveTo(0);
	state.stepperB.run();

	// Programmatically find the best target for travel-based strategies
	auto targets = state.currentTargetArray() | std::views::filter([](const Target& t) { return t.valid; });
	auto smallest_travel_target = &(*std::ranges::min_element(
		targets,
		[&](const Target& a, const Target& b) {
			return state.calculateTravelDistance(a) < state.calculateTravelDistance(b);
		}
	));
	auto longest_travel_target = &(*std::ranges::max_element(
		targets,
		[&](const Target& a, const Target& b) {
			return state.calculateTravelDistance(a) < state.calculateTravelDistance(b);
		}
	));

	state.setStrategy(TurretStrategy::SMALLEST_TRAVEL);
	CHECK(state.findBestTarget() == smallest_travel_target);

	state.setStrategy(TurretStrategy::LONGEST_TRAVEL);
	CHECK(state.findBestTarget() == longest_travel_target);

	state.setStrategy(TurretStrategy::RANDOM);
	auto random_target = state.findBestTarget();
	CHECK(random_target->valid);
	CHECK((random_target == &state.fetchTarget(1) || random_target == &state.fetchTarget(2) ||
	       random_target == &state.fetchTarget(3)));
}
