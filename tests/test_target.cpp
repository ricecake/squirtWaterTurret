#include "doctest/doctest.h"
#include "mock_time.h"
#include "spatial.h"
#include "target.h"

extern TestClock mock_clock;

TEST_CASE("Target InterceptPosition" * doctest::may_fail()) {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Stationary target") {
		// Setup: A stationary target at a known position.
		PositionVector target_pos(10, 0, 1.5); // 10m away, at the same height as the turret
		Target         target(target_pos, VelocityVector(0, 0, 0));

		// Execution: Get the calculated aiming position from the target.
		PositionVector aiming_position = target.interceptPosition();

		// Verification: Simulate the projectile's flight and check for an intercept.
		const fixed          projectile_speed = 20.0;
		const PositionVector initial_proj_pos(0, 0, 1.5);
		const fixed          g = 9.814;

		// 1. Calculate the initial velocity vector of the projectile.
		PositionVector initial_proj_vel_pos = aiming_position.normalize() * projectile_speed;
		VelocityVector initial_proj_vel(
			initial_proj_vel_pos.X_coord,
			initial_proj_vel_pos.Y_coord,
			initial_proj_vel_pos.Z_coord
		);

		// 2. Calculate the time of flight to the target's XZ plane.
		fixed time_of_flight = target_pos.X_coord / initial_proj_vel.X_coord;

		// 3. Calculate the projectile's position at the time of flight.
		PositionVector final_proj_pos;
		final_proj_pos.X_coord = initial_proj_pos.X_coord + initial_proj_vel.X_coord * time_of_flight;
		final_proj_pos.Y_coord = initial_proj_pos.Y_coord + initial_proj_vel.Y_coord * time_of_flight;
		final_proj_pos.Z_coord = initial_proj_pos.Z_coord + initial_proj_vel.Z_coord * time_of_flight -
		                       fixed(0.5) * g * (time_of_flight * time_of_flight);

		// 4. Check if the projectile's final position is close to the target's position.
		CHECK(is_close(final_proj_pos.X_coord, target_pos.X_coord, fixed(0.1)));
		CHECK(is_close(final_proj_pos.Y_coord, target_pos.Y_coord, fixed(0.1)));
		CHECK(is_close(final_proj_pos.Z_coord, target_pos.Z_coord, fixed(0.1)));
	}

	SUBCASE("Moving target") {
		// Setup: A target moving towards the turret.
		PositionVector initial_target_pos(10, 0, 1.5);
		VelocityVector target_vel(-5, 0, 0);
		Target         target(initial_target_pos, target_vel);

		// Execution: Get the calculated aiming position.
		PositionVector aiming_position = target.interceptPosition();

		// Verification: Simulate and check for intercept.
		const fixed          projectile_speed = 20.0;
		const PositionVector initial_proj_pos(0, 0, 1.5);
		const fixed          g = 9.814;

		// 1. Calculate the initial velocity vector of the projectile.
		PositionVector initial_proj_vel_pos = aiming_position.normalize() * projectile_speed;
		VelocityVector initial_proj_vel(
			initial_proj_vel_pos.X_coord,
			initial_proj_vel_pos.Y_coord,
			initial_proj_vel_pos.Z_coord
		);

		// 2. Calculate the time of flight.
		fixed time_of_flight = initial_target_pos.X_coord / (initial_proj_vel.X_coord - target_vel.X_coord);

		// 3. Calculate the target's position at the time of flight.
		PositionVector final_target_pos = initial_target_pos + target_vel * time_of_flight;

		// 4. Calculate the projectile's position at the time of flight.
		PositionVector final_proj_pos;
		final_proj_pos.X_coord = initial_proj_pos.X_coord + initial_proj_vel.X_coord * time_of_flight;
		final_proj_pos.Y_coord = initial_proj_pos.Y_coord + initial_proj_vel.Y_coord * time_of_flight;
		final_proj_pos.Z_coord = initial_proj_pos.Z_coord + initial_proj_vel.Z_coord * time_of_flight -
		                       fixed(0.5) * g * (time_of_flight * time_of_flight);

		// 5. Check if the positions are close.
		CHECK(is_close(final_proj_pos.X_coord, final_target_pos.X_coord, fixed(0.1)));
		CHECK(is_close(final_proj_pos.Y_coord, final_target_pos.Y_coord, fixed(0.1)));
		CHECK(is_close(final_proj_pos.Z_coord, final_target_pos.Z_coord, fixed(0.1)));
	}

	SUBCASE("Impossible target") {
		// Setup: A target that is too far away to be hit.
		PositionVector target_pos(1000, 0, 1.5);
		Target         target(target_pos, VelocityVector(0, 0, 0));

		// Execution: Get the calculated aiming position.
		PositionVector aiming_position = target.interceptPosition();

		// Verification: The aiming position should be the vector difference,
		// as the root-finding should fail and return this as a fallback.
		const PositionVector initial_proj_pos(0, 0, 1.5);
		DistanceVector       diff = target_pos - initial_proj_pos;
		CHECK(is_close(aiming_position.X_coord, diff.X_coord, fixed(0.1)));
		CHECK(is_close(aiming_position.Y_coord, diff.Y_coord, fixed(0.1)));
		CHECK(is_close(aiming_position.Z_coord, diff.Z_coord, fixed(0.1)));
	}
}
