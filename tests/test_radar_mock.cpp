#include "common.h"
#include "tests/mocks.h"

TEST_CASE("Stateful Radar Mock") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;
	LD2450                            radar_mock;

	// --- Targets 0 & 2 (Continuous Movers) ---
	auto initial_pos_0 = radar_mock.getTarget(0);
	auto initial_pos_2 = radar_mock.getTarget(2);

	// Advance time
	mock_clock += std::chrono::seconds(1);

	auto new_pos_0 = radar_mock.getTarget(0);
	auto new_pos_2 = radar_mock.getTarget(2);

	// Verify they moved
	CHECK(initial_pos_0.x != new_pos_0.x);
	CHECK(initial_pos_0.y != new_pos_0.y);
	CHECK(initial_pos_2.x != new_pos_2.x);
	CHECK(initial_pos_2.y != new_pos_2.y);

	// --- Target 1 (Walk and Pause) ---
	// It starts at its destination, so its first action is to set a pause timer
	// and choose a new destination. It does not move on the first call.
	auto pos1_initial = radar_mock.getTarget(1);

	// Advance time past the maximum possible pause (7 seconds)
	mock_clock += std::chrono::seconds(8);

	// Now that the pause is over, the target should move towards its new destination.
	auto pos1_after_pause = radar_mock.getTarget(1);
	CHECK(pos1_initial.x != pos1_after_pause.x);
	CHECK(pos1_initial.y != pos1_after_pause.y);

	// Advance time again to ensure it continues to move.
	mock_clock += std::chrono::seconds(1);
	auto pos1_after_move = radar_mock.getTarget(1);
	CHECK(pos1_after_pause.x != pos1_after_move.x);
	CHECK(pos1_after_pause.y != pos1_after_move.y);
}
