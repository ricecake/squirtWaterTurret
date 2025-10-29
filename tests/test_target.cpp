#include "target.h"
#include "doctest/doctest.h"
#include "utilities.h"
#include <iostream>

static TimePoint mock_time;
TimePoint        getMockTime() {
	return mock_time;
}

TEST_CASE("Target class tests") {
	// Set up mock clock
	Clock::setClock(getMockTime);
	mock_time = TimePoint(seconds(0));

	SUBCASE("Target constructor initializes correctly") {
		PositionVector initial_pos(10, 20, 30);
		VelocityVector initial_vel(1, 2, 3);
		Target         target(initial_pos, initial_vel);

		CHECK(target.Position().X_coord == initial_pos.X_coord);
		CHECK(target.Position().Y_coord == initial_pos.Y_coord);
		CHECK(target.Position().Z_coord == initial_pos.Z_coord);
		CHECK(target.Velocity().X_coord == initial_vel.X_coord);
		CHECK(target.Velocity().Y_coord == initial_vel.Y_coord);
		CHECK(target.Velocity().Z_coord == initial_vel.Z_coord);
		CHECK(target.valid == false); // Default validity
	}

	SUBCASE("Target Update method calculates velocity") {
		Target target(PositionVector(0, 0, 0));
		mock_time = TimePoint(seconds(0));
		target.Update(PositionVector(10, 20, 30));

		// First update should set position but not velocity
		CHECK(target.Position().X_coord == 10);
		CHECK(target.Velocity().X_coord == 0); // No velocity yet

		// Second update after 1 second
		mock_time = TimePoint(seconds(1));
		target.Update(PositionVector(20, 30, 40));

		// Check position
		CHECK(target.Position().X_coord == 20);
		CHECK(target.Position().Y_coord == 30);
		CHECK(target.Position().Z_coord == 40);

		// Check velocity ( (20-10)/1, (30-20)/1, (40-30)/1 )
		CHECK(target.Velocity().X_coord == 10);
		CHECK(target.Velocity().Y_coord == 10);
		CHECK(target.Velocity().Z_coord == 10);
	}

	SUBCASE("PredictedPositionAtTime calculates future position") {
		Target target(PositionVector(10, 20, 30), VelocityVector(5, -5, 10));
		mock_time = TimePoint(seconds(0));

		// Predict position 2 seconds in the future
		auto predicted_pos = target.PredictedPositionAtTime(seconds(2));

		CHECK(predicted_pos.X_coord == (10 + 5 * 2));   // 20
		CHECK(predicted_pos.Y_coord == (20 + -5 * 2));  // 10
		CHECK(predicted_pos.Z_coord == (30 + 10 * 2));  // 50
	}
}
