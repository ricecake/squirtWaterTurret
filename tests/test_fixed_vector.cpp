#include "common.h"
#include "spatial.h"

extern TestClock mock_clock;

TEST_CASE("FixedVector3D") {
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("integer_sqrt") {
		CHECK(PositionVector::integer_sqrt(uint64_t(fixed_16_16(16).raw_value()) << 16).raw_value() == (4 << 16));
		CHECK(PositionVector::integer_sqrt(uint64_t(fixed_16_16(25).raw_value()) << 16).raw_value() == (5 << 16));
		CHECK(PositionVector::integer_sqrt(uint64_t(fixed_16_16(144).raw_value()) << 16).raw_value() == (12 << 16));
	}

	SUBCASE("angleTo") {
		PositionVector v1(1, 0, 0);
		PositionVector v2(0, 1, 0);
		CHECK(v1.angleTo(v2) == doctest::Approx(90.0));
	}

	SUBCASE("dot") {
		PositionVector v1(1, 2, 3);
		PositionVector v2(4, 5, 6);
		CHECK(v1.dot(v2) == 32);
	}

	SUBCASE("cross") {
		PositionVector v1(1, 0, 0);
		PositionVector v2(0, 1, 0);
		PositionVector v3 = v1.cross(v2);
		CHECK(v3.X_coord == 0);
		CHECK(v3.Y_coord == 0);
		CHECK(v3.Z_coord == 1);
	}

	SUBCASE("magnitude") {
		PositionVector v(3, 4, 5);
		CHECK(v.magnitude() == doctest::Approx(7.071).epsilon(0.01));
	}

	SUBCASE("magnitudeXY") {
		PositionVector v(3, 4, 5);
		CHECK(v.magnitudeXY() == 5);
	}

	SUBCASE("normalize") {
		PositionVector v(3, 4, 0);
		PositionVector normalized = v.normalize();
		CHECK(normalized.X_coord == doctest::Approx(0.6));
		CHECK(normalized.Y_coord == doctest::Approx(0.8));
		CHECK(normalized.Z_coord == 0);
	}

	SUBCASE("pitch") {
		PositionVector v(3, 4, 5);
		CHECK(v.pitch() == doctest::Approx(45.0));
	}

	SUBCASE("yaw") {
		PositionVector v(3, 4, 5);
		CHECK(v.yaw() == doctest::Approx(36.8698).epsilon(0.01));
	}
}
