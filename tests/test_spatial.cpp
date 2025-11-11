#include "doctest/doctest.h"
#include "mock_time.h"
#include "spatial.h"

extern TestClock mock_clock;

TEST_CASE("PositionVector") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Constructors") {
		PositionVector v1(1, 2, 3);
		CHECK(v1.X_coord == 1);
		CHECK(v1.Y_coord == 2);
		CHECK(v1.Z_coord == 3);

		PositionVector v2(v1);
		CHECK(v2.X_coord == 1);
		CHECK(v2.Y_coord == 2);
		CHECK(v2.Z_coord == 3);

		DistanceVector d(4, 5, 6);
		PositionVector v3(v1, d);
		CHECK(v3.X_coord == 5);
		CHECK(v3.Y_coord == 7);
		CHECK(v3.Z_coord == 9);

		VelocityVector vel(1, 1, 1);
		PositionVector v4(v1, vel, seconds(1));
		CHECK(v4.X_coord == 2);
		CHECK(v4.Y_coord == 3);
		CHECK(v4.Z_coord == 4);
	}

	SUBCASE("Methods") {
		PositionVector v(3, 4, 5);
		CHECK(v.Distance() == 5);
		CHECK(v.Pitch() == 45.0);
		CHECK(v.Yaw() == 36.8698);
	}
}

TEST_CASE("DistanceVector") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Constructors") {
		DistanceVector v1(1, 2, 3);
		CHECK(v1.X_coord == 1);
		CHECK(v1.Y_coord == 2);
		CHECK(v1.Z_coord == 3);

		VelocityVector vel(1, 2, 3);
		DistanceVector v2(vel, seconds(2));
		CHECK(v2.X_coord == 2);
		CHECK(v2.Y_coord == 4);
		CHECK(v2.Z_coord == 6);
	}
}

TEST_CASE("VelocityVector") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Constructors") {
		VelocityVector v1(1, 2, 3);
		CHECK(v1.X_coord == 1);
		CHECK(v1.Y_coord == 2);
		CHECK(v1.Z_coord == 3);

		DistanceVector d(2, 4, 6);
		VelocityVector v2(d, seconds(2));
		CHECK(v2.X_coord == 1);
		CHECK(v2.Y_coord == 2);
		CHECK(v2.Z_coord == 3);
	}
}

TEST_CASE("Spatial Operator Overloads") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("VelocityVector = DistanceVector / TimeInterval") {
		DistanceVector d(10, 20, 30);
		VelocityVector v = d / seconds(2);
		CHECK(v.X_coord == 5);
		CHECK(v.Y_coord == 10);
		CHECK(v.Z_coord == 15);
	}

	SUBCASE("DistanceVector = VelocityVector * TimeInterval") {
		VelocityVector v(1, 2, 3);
		DistanceVector d = v * seconds(3);
		CHECK(d.X_coord == 3);
		CHECK(d.Y_coord == 6);
		CHECK(d.Z_coord == 9);
	}

	SUBCASE("PositionVector = PositionVector + DistanceVector") {
		PositionVector p1(1, 2, 3);
		DistanceVector d(4, 5, 6);
		PositionVector p2 = p1 + d;
		CHECK(p2.X_coord == 5);
		CHECK(p2.Y_coord == 7);
		CHECK(p2.Z_coord == 9);
	}

	SUBCASE("DistanceVector = PositionVector - PositionVector") {
		PositionVector p1(5, 7, 9);
		PositionVector p2(1, 2, 3);
		DistanceVector d = p1 - p2;
		CHECK(d.X_coord == 4);
		CHECK(d.Y_coord == 5);
		CHECK(d.Z_coord == 6);
	}
}
