#include "common.h"
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
		CHECK(predicted_pos.x == initial_pos.x);
		CHECK(predicted_pos.y == initial_pos.y);
		CHECK(predicted_pos.z == initial_pos.z);
	}

	SUBCASE("Prediction after time has passed") {
		auto predicted_pos = target.PredictedPositionAtTime(seconds(1));
		CHECK(predicted_pos.x == initial_pos.x + velocity.x);
		CHECK(predicted_pos.y == initial_pos.y + velocity.y);
		CHECK(predicted_pos.z == initial_pos.z + velocity.z);
	}
}

TEST_CASE("Target InterceptAimpoint") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Stationary target") {
		// Setup: A stationary target at a known position.
		PositionVector target_pos(10, 0, 1.5); // 10m away, at the same height as the turret
		Target         target(target_pos, VelocityVector(0, 0, 0));

		// Execution: Get the calculated aiming position from the target.
		PositionVector aiming_position = target.InterceptAimpoint();

		// Verification: Simulate the projectile's flight and check for an intercept.
		const fixed          projectile_speed = 20.0;
		const PositionVector initial_proj_pos(0, 0, 1.5);
		const fixed          g = 9.814;

		// 1. Calculate the initial velocity vector of the projectile.
		PositionVector initial_proj_vel_pos = aiming_position.normalize() * projectile_speed;
		VelocityVector initial_proj_vel(initial_proj_vel_pos.x, initial_proj_vel_pos.y, initial_proj_vel_pos.z);

		// 2. Calculate the time of flight to the target's XZ plane.
		fixed time_of_flight = target_pos.x / initial_proj_vel.x;

		// 3. Calculate the projectile's position at the time of flight.
		PositionVector final_proj_pos;
		final_proj_pos.x = initial_proj_pos.x + initial_proj_vel.x * time_of_flight;
		final_proj_pos.y = initial_proj_pos.y + initial_proj_vel.y * time_of_flight;
		final_proj_pos.z = initial_proj_pos.z + initial_proj_vel.z * time_of_flight -
			fixed(0.5) * g * (time_of_flight * time_of_flight);

		// 4. Check if the projectile's final position is close to the target's position.
		CHECK(final_proj_pos.x == Approx(target_pos.x).epsilon(0.001));
		CHECK(final_proj_pos.y == Approx(target_pos.y).epsilon(0.001));
		CHECK(final_proj_pos.z == Approx(target_pos.z).epsilon(0.001));
	}

	SUBCASE("Moving target") {
		// Setup: A target moving towards the turret.
		PositionVector initial_target_pos(10, 0, 1.5);
		VelocityVector target_vel(-5, 0, 0);
		Target         target(initial_target_pos, target_vel);

		// Execution: Get the calculated aiming position.
		PositionVector aiming_position = target.InterceptAimpoint();
		logger::Log("Aimpoint in test: ", aiming_position);

		// Verification: Simulate and check for intercept.
		const fixed          projectile_speed = 20.0;
		const PositionVector initial_proj_pos(0, 0, 1.5);
		const fixed          g = 9.814;

		// 1. Calculate the initial velocity vector of the projectile.
		PositionVector initial_proj_vel_pos = aiming_position.normalize() * projectile_speed;
		VelocityVector initial_proj_vel(initial_proj_vel_pos.x, initial_proj_vel_pos.y, initial_proj_vel_pos.z);

		// 2. Calculate the time of flight.
		fixed time_of_flight = initial_target_pos.x / (initial_proj_vel.x - target_vel.x);

		// 3. Calculate the target's position at the time of flight.
		PositionVector final_target_pos = initial_target_pos + target_vel * time_of_flight;

		// 4. Calculate the projectile's position at the time of flight.
		PositionVector final_proj_pos;
		final_proj_pos.x = initial_proj_pos.x + initial_proj_vel.x * time_of_flight;
		final_proj_pos.y = initial_proj_pos.y + initial_proj_vel.y * time_of_flight;
		final_proj_pos.z = initial_proj_pos.z + initial_proj_vel.z * time_of_flight -
			fixed(0.5) * g * (time_of_flight * time_of_flight);

		// 5. Check if the positions are close.
		CHECK(final_proj_pos.x == doctest::Approx(double(final_target_pos.x)).epsilon(0.001));
		CHECK(final_proj_pos.y == doctest::Approx(double(final_target_pos.y)));
		CHECK(final_proj_pos.z == doctest::Approx(double(final_target_pos.z)).epsilon(0.01));
	}

	SUBCASE("Impossible target") {
		// Setup: A target that is too far away to be hit.
		PositionVector target_pos(1000, 0, 1.5);
		Target         target(target_pos, VelocityVector(0, 0, 0));

		// Execution: Get the calculated aiming position.
		PositionVector aiming_position = target.InterceptAimpoint();

		// Verification: The aiming position should be the target position,
		// as the root-finding should fail and return this as a fallback.
		CHECK(aiming_position.x == target_pos.x);
		CHECK(aiming_position.y == target_pos.y);
		CHECK(aiming_position.z == target_pos.z);
	}

	SUBCASE("Moving target at an angle") {
		// Setup: A target moving at an angle to the turret.
		PositionVector initial_target_pos(10, 5, 1.5);
		VelocityVector target_vel(-2, -1, 0);
		Target         target(initial_target_pos, target_vel);

		// Execution: Get the calculated aiming position.
		PositionVector aiming_position = target.InterceptAimpoint();
		logger::Log("Aimpoint in test: ", aiming_position);

		// Verification: Simulate and check for intercept.
		const fixed          projectile_speed = 20.0;
		const PositionVector initial_proj_pos(0, 0, 1.5);
		const fixed          g = 9.814;

		// 1. Calculate the initial velocity vector of the projectile.
		PositionVector initial_proj_vel_pos = aiming_position.normalize() * projectile_speed;
		VelocityVector initial_proj_vel(initial_proj_vel_pos.x, initial_proj_vel_pos.y, initial_proj_vel_pos.z);

		// 2. Calculate the time of flight based on the relative speed in the X direction.
		fixed time_of_flight = initial_target_pos.x / (initial_proj_vel.x - target_vel.x);

		// 3. Calculate the target's position at the time of flight.
		PositionVector final_target_pos = initial_target_pos + target_vel * time_of_flight;

		// 4. Calculate the projectile's position at the time of flight.
		PositionVector final_proj_pos;
		final_proj_pos.x = initial_proj_pos.x + initial_proj_vel.x * time_of_flight;
		final_proj_pos.y = initial_proj_pos.y + initial_proj_vel.y * time_of_flight;
		final_proj_pos.z = initial_proj_pos.z + initial_proj_vel.z * time_of_flight -
			fixed(0.5) * g * (time_of_flight * time_of_flight);

		// 5. Check if the positions are close.
		CHECK(final_proj_pos.x == doctest::Approx(double(final_target_pos.x)).epsilon(0.01));
		CHECK(final_proj_pos.y == doctest::Approx(double(final_target_pos.y)).epsilon(0.01));
		CHECK(final_proj_pos.z == doctest::Approx(double(final_target_pos.z)).epsilon(0.01));
	}
}
