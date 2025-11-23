#include "common.h"
#include "tests/mocks.h"

TEST_CASE("Stateful Radar Mock") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;
	LD2450                            radar_mock;

	// --- Target 0 (Random Walk) ---
	// Just check that it moves. Its movement is unpredictable by design.
	SUBCASE("Target 0 - Random Walk") {
		auto initial_pos_0 = radar_mock.getTarget(0);
		mock_clock += std::chrono::seconds(1);
		auto new_pos_0 = radar_mock.getTarget(0);
		CHECK(initial_pos_0.x != new_pos_0.x);
		CHECK(initial_pos_0.y != new_pos_0.y);
	}

	// --- Target 2 (Continuous Patrol) ---
	SUBCASE("Target 2 - Continuous Patrol") {
		for (int i = 0; i < 3; ++i) {
			auto  current_pos = radar_mock.getTarget(2);
			auto& mock_target_before_step = radar_mock.mock_targets[2];
			double dest_x_before = mock_target_before_step.dest_x;
			double dest_y_before = mock_target_before_step.dest_y;
			double initial_dist = std::hypot(dest_x_before - current_pos.x, dest_y_before - current_pos.y);

			mock_clock += std::chrono::seconds(1);

			auto  new_pos = radar_mock.getTarget(2);
			auto& mock_target_after_step = radar_mock.mock_targets[2];
			double dest_x_after = mock_target_after_step.dest_x;
			double dest_y_after = mock_target_after_step.dest_y;

			// If the destination did NOT change during this step, it must have moved closer.
			if (dest_x_before == dest_x_after && dest_y_before == dest_y_after) {
				double new_dist = std::hypot(dest_x_after - new_pos.x, dest_y_after - new_pos.y);
				INFO("Target 2, Iteration " << i);
				CHECK(new_dist < initial_dist);
			}
			// If dest changed, it means it arrived and got a new goal, which is correct behavior.
		}
	}

	// --- Target 1 (Walk and Pause) ---
	SUBCASE("Target 1 - Walk and Pause") {
		// First call sets its initial pause and new destination.
		radar_mock.getTarget(1);

		for (int i = 0; i < 3; ++i) {
			auto& target_state = radar_mock.mock_targets[1];

			// Advance clock past the current pause period
			auto now = Clock::now();
			if (now < target_state.pause_until) {
				mock_clock += (target_state.pause_until - now) + std::chrono::microseconds(1);
			}

			// Now it should be ready to move. Get current position and destination.
			auto   pos_before_move = radar_mock.getTarget(1);
			double dest_x = target_state.dest_x;
			double dest_y = target_state.dest_y;
			double dist_before = std::hypot(dest_x - pos_before_move.x, dest_y - pos_before_move.y);

			// Advance time and check that it moved closer
			mock_clock += std::chrono::seconds(1);
			auto pos_after_move = radar_mock.getTarget(1);

			// It's possible it reached its destination. If it didn't, it must be closer.
			if (dest_x == target_state.dest_x && dest_y == target_state.dest_y) {
				double dist_after = std::hypot(dest_x - pos_after_move.x, dest_y - pos_after_move.y);
				INFO("Target 1, Iteration " << i);
				CHECK(dist_after < dist_before);
			}
		}
	}
}
