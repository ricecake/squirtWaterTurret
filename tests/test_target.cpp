#include "doctest/doctest.h"
#include "mock_time.h"
#include "spatial.h"
#include "target.h"

extern TestClock mock_clock;

TEST_CASE("Target PredictedPosition") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	PositionVector initial_pos = {10, 20, 30};
	VelocityVector velocity = {1, 2, 3};
	Target         target(initial_pos, velocity);

	SUBCASE("Prediction at time of creation") {
		auto predicted_pos = target.PredictedPositionAtTime(microseconds(0));
		CHECK(predicted_pos.X_coord == initial_pos.X_coord);
		CHECK(predicted_pos.Y_coord == initial_pos.Y_coord);
		CHECK(predicted_pos.Z_coord == initial_pos.Z_coord);
	}

	SUBCASE("Prediction after time has passed") {
		auto predicted_pos = target.PredictedPositionAtTime(seconds(1));
		CHECK(predicted_pos.X_coord == initial_pos.X_coord + velocity.X_coord);
		CHECK(predicted_pos.Y_coord == initial_pos.Y_coord + velocity.Y_coord);
		CHECK(predicted_pos.Z_coord == initial_pos.Z_coord + velocity.Z_coord);
	}
}

TEST_CASE("Target InterceptPosition") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Stationary target") {
		// Target is at x=10, y=0, z=1.5 (same height as projectile)
		// Projectile initial position is (0, 0, 1.5)
		PositionVector target_pos(10, 0, 1.5);
		Target         target(target_pos, VelocityVector(0, 0, 0));

		// Get the required initial velocity vector
		PositionVector launch_velocity = target.interceptPosition();

		// Check if the calculated velocity is correct
		CHECK(is_close(launch_velocity.X_coord, fixed(19.85), fixed(0.1)));
		CHECK(is_close(launch_velocity.Y_coord, fixed(0.0), fixed(0.1)));
		CHECK(is_close(launch_velocity.Z_coord, fixed(2.47), fixed(0.1)));
	}

	SUBCASE("Moving target") {
		// Target starts at x=10 and moves towards the launcher at 5 m/s
		PositionVector target_pos(10, 0, 1.5);
		VelocityVector target_vel(-5, 0, 0);
		Target         target(target_pos, target_vel);

		// Get the required initial velocity for the projectile
		PositionVector launch_velocity = target.interceptPosition();

		// Manually calculate the time to intercept from the launch velocity
		// V_launch_x = (target_pos_x / t) + V_target_x  => t = target_pos_x / (V_launch_x - V_target_x)
		fixed time_to_intercept = target_pos.X_coord / (launch_velocity.X_coord - target_vel.X_coord);

		// Determine the intercept position
		PositionVector intercept_pos = target_pos + target_vel * time_to_intercept;

		// Determine where the projectile will be at the intercept time
		const PositionVector proj_pos_initial(0, 0, 1.5);
		const fixed          g = fixed(9.814);
		PositionVector       proj_pos_at_intercept;
		proj_pos_at_intercept.X_coord = launch_velocity.X_coord * time_to_intercept;
		proj_pos_at_intercept.Y_coord = launch_velocity.Y_coord * time_to_intercept;
		proj_pos_at_intercept.Z_coord = proj_pos_initial.Z_coord + launch_velocity.Z_coord * time_to_intercept -
		                                fixed(0.5) * g * time_to_intercept * time_to_intercept;

		// Check that the projectile and target meet
		CHECK(is_close(intercept_pos.X_coord, proj_pos_at_intercept.X_coord, fixed(0.1)));
		CHECK(is_close(intercept_pos.Y_coord, proj_pos_at_intercept.Y_coord, fixed(0.1)));
		CHECK(is_close(intercept_pos.Z_coord, proj_pos_at_intercept.Z_coord, fixed(0.1)));
	}

	SUBCASE("Impossible target") {
		// Target is extremely far away, making intercept impossible with a projectile speed of 20
		PositionVector       target_pos(1000, 0, 1.5);
		Target               target(target_pos, VelocityVector(0, 0, 0));
		const PositionVector proj_pos(0, 0, 1.5);

		// Get the required initial velocity vector
		PositionVector launch_velocity = target.interceptPosition();

		// When the root finder fails, it should return the difference vector
		DistanceVector diff = target_pos - proj_pos;
		CHECK(is_close(launch_velocity.X_coord, diff.X_coord, fixed(0.1)));
		CHECK(is_close(launch_velocity.Y_coord, diff.Y_coord, fixed(0.1)));
		CHECK(is_close(launch_velocity.Z_coord, diff.Z_coord, fixed(0.1)));
	}
}
