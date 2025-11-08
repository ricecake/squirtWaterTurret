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
