#include "common.h"
#include "spatial.h"

extern TestClock mock_clock;

TEST_CASE("SPECIAL OPERATORS") {
	auto root = PositionVector::integer_sqrt(uint64_t(fixed_16_16(16).raw_value()) << 16);
	CHECK(root == 4);
}

TEST_CASE("PositionVector") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Constructors") {
		PositionVector v1(1, 2, 3);
		CHECK(v1.x == 1);
		CHECK(v1.y == 2);
		CHECK(v1.z == 3);

		PositionVector v2(v1);
		CHECK(v2.x == 1);
		CHECK(v2.y == 2);
		CHECK(v2.z == 3);

		DistanceVector d(4, 5, 6);
		PositionVector v3(v1, d);
		CHECK(v3.x == 5);
		CHECK(v3.y == 7);
		CHECK(v3.z == 9);

		VelocityVector vel(1, 1, 1);
		PositionVector v4(v1, vel, seconds(1));
		CHECK(v4.x == 2);
		CHECK(v4.y == 3);
		CHECK(v4.z == 4);
	}

	SUBCASE("Methods") {
		PositionVector v(3, 4, 5);
		CHECK(v.Distance() == 5);
		CHECK(v.Pitch() == Approx(45.0));
		CHECK(v.Yaw() == Approx(36.8698).epsilon(0.01));
	}
}

TEST_CASE("DistanceVector") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Constructors") {
		DistanceVector v1(1, 2, 3);
		CHECK(v1.x == 1);
		CHECK(v1.y == 2);
		CHECK(v1.z == 3);

		VelocityVector vel(1, 2, 3);
		DistanceVector v2(vel, seconds(2));
		CHECK(v2.x == 2);
		CHECK(v2.y == 4);
		CHECK(v2.z == 6);
	}
}

TEST_CASE("VelocityVector") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("Constructors") {
		VelocityVector v1(1, 2, 3);
		CHECK(v1.x == 1);
		CHECK(v1.y == 2);
		CHECK(v1.z == 3);

		DistanceVector d(2, 4, 6);
		VelocityVector v2(d, seconds(2));
		CHECK(v2.x == 1);
		CHECK(v2.y == 2);
		CHECK(v2.z == 3);
	}
}

TEST_CASE("Spatial Operator Overloads") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("VelocityVector = DistanceVector / TimeInterval") {
		DistanceVector d(10, 20, 30);
		VelocityVector v = d / seconds(2);
		CHECK(v.x == 5);
		CHECK(v.y == 10);
		CHECK(v.z == 15);
	}

	SUBCASE("DistanceVector = VelocityVector * TimeInterval") {
		VelocityVector v(1, 2, 3);
		DistanceVector d = v * seconds(3);
		CHECK(d.x == 3);
		CHECK(d.y == 6);
		CHECK(d.z == 9);
	}

	SUBCASE("PositionVector = PositionVector + DistanceVector") {
		PositionVector p1(1, 2, 3);
		DistanceVector d(4, 5, 6);
		PositionVector p2 = p1 + d;
		CHECK(p2.x == 5);
		CHECK(p2.y == 7);
		CHECK(p2.z == 9);
	}

	SUBCASE("DistanceVector = PositionVector - PositionVector") {
		PositionVector p1(5, 7, 9);
		PositionVector p2(1, 2, 3);
		DistanceVector d = p1 - p2;
		CHECK(d.x == 4);
		CHECK(d.y == 5);
		CHECK(d.z == 6);
	}
}
