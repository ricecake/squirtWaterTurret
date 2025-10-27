#include "doctest.h"
#include "fpm_adapter.hpp"
#include "vector.hpp"
#include <cmath>

using fixed = fixed_16_16;

// Helper to check for approximate equality of fixed-point numbers
bool is_close(fixed a, fixed b, fixed tolerance = fixed(0.001)) {
	return fpm::abs(a - b) < tolerance;
}

TEST_CASE("Vector Addition") {
	Vector3D<fixed> v1{fixed(1), fixed(2), fixed(3)};
	Vector3D<fixed> v2{fixed(4), fixed(5), fixed(6)};
	auto            result = v1 + v2;
	REQUIRE(result.X_coord == fixed(5));
	REQUIRE(result.Y_coord == fixed(7));
	REQUIRE(result.Z_coord == fixed(9));
}

TEST_CASE("Vector Subtraction") {
	Vector3D<fixed> v1{fixed(4), fixed(5), fixed(6)};
	Vector3D<fixed> v2{fixed(1), fixed(2), fixed(3)};
	auto            result = v1 - v2;
	REQUIRE(result.X_coord == fixed(3));
	REQUIRE(result.Y_coord == fixed(3));
	REQUIRE(result.Z_coord == fixed(3));
}

TEST_CASE("Vector Scalar Multiplication") {
	Vector3D<fixed> v{fixed(1), fixed(2), fixed(3)};
	auto            result = v * fixed(2);
	REQUIRE(result.X_coord == fixed(2));
	REQUIRE(result.Y_coord == fixed(4));
	REQUIRE(result.Z_coord == fixed(6));
}

TEST_CASE("Vector Scalar Division") {
	Vector3D<fixed> v{fixed(2), fixed(4), fixed(6)};
	auto            result = v / fixed(2);
	REQUIRE(result.X_coord == fixed(1));
	REQUIRE(result.Y_coord == fixed(2));
	REQUIRE(result.Z_coord == fixed(3));
}

TEST_CASE("Vector Dot Product") {
	Vector3D<fixed> v1{fixed(1), fixed(2), fixed(3)};
	Vector3D<fixed> v2{fixed(4), fixed(5), fixed(6)};
	fixed           result = v1.dot(v2); // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
	REQUIRE(result == fixed(32));
}

TEST_CASE("Vector Cross Product") {
	Vector3D<fixed> v1{fixed(1), fixed(0), fixed(0)}; // i
	Vector3D<fixed> v2{fixed(0), fixed(1), fixed(0)}; // j
	auto            result = v1.cross(v2);           // should be k (0, 0, 1)
	REQUIRE(result.X_coord == fixed(0));
	REQUIRE(result.Y_coord == fixed(0));
	REQUIRE(result.Z_coord == fixed(1));
}

TEST_CASE("Vector Magnitude") {
	Vector3D<fixed> v{fixed(3), fixed(4), fixed(0)};
	fixed           result = v.magnitude(); // sqrt(9 + 16) = 5
	REQUIRE(is_close(result, fixed(5)));
}

TEST_CASE("Vector Normalization") {
	Vector3D<fixed> v{fixed(3), fixed(4), fixed(0)};
	auto            result = v.normalize();
	// magnitude should be 1
	REQUIRE(is_close(result.magnitude(), fixed(1)));
	// direction should be the same
	REQUIRE(is_close(result.X_coord, fixed(0.6)));
	REQUIRE(is_close(result.Y_coord, fixed(0.8)));
	REQUIRE(is_close(result.Z_coord, fixed(0)));
}
