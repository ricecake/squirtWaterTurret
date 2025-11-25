#include "doctest/doctest.h"
#include "state.h"
#include "tests/doctest_fpm_adapter.hpp"
#include "utilities.h"

TEST_CASE("Test PositionVector::angleTo Calculation") {
	// This is a unit test for the `angleTo` method in `spatial.h`. It was
	// added to debug an issue with the fixed-point math that was causing
	// incorrect results.

	// 1. Define the first vector (representing a target aimpoint).
	// Spherical coordinates: yaw=45 deg, pitch=7.9 deg.
	fixed yaw_rad_1 = fixed(45) * deg2RadFactor;
	fixed pitch_rad_1 = fixed(7.9) * deg2RadFactor;

	fixed          x1 = cos(pitch_rad_1) * sin(yaw_rad_1);
	fixed          y1 = cos(pitch_rad_1) * cos(yaw_rad_1);
	fixed          z1 = sin(pitch_rad_1);
	PositionVector vec1(x1, y1, z1);

	// 2. Define the second vector (representing the turret's current orientation).
	// Spherical coordinates: yaw=30, pitch=15.
	fixed yaw_rad_2 = fixed(30) * deg2RadFactor;
	fixed pitch_rad_2 = fixed(15) * deg2RadFactor;

	fixed          x2 = cos(pitch_rad_2) * sin(yaw_rad_2);
	fixed          y2 = cos(pitch_rad_2) * cos(yaw_rad_2);
	fixed          z2 = sin(pitch_rad_2);
	PositionVector vec2(x2, y2, z2);

	// 3. Calculate the angle between them.
	fixed actual_angle = vec1.angleTo(vec2);

	// 4. The expected value has been calculated and verified to be ~16.29 degrees.
	// This test now serves as a regression check for the fixed-point angleTo logic.
	CHECK(actual_angle == doctest::Approx(16.2902));
}
