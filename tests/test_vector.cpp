#include <iostream>
#include <ostream>

#include "doctest/doctest.h"
#include "fpm_adapter.hpp"
#include "spatial.h"
#include "vector.hpp"

using fixed = fixed_16_16;

TEST_CASE("Vector addition") {
	Vector3D<fixed> v1{1, 2, 3};
	Vector3D<fixed> v2{4, 5, 6};
	auto            result = v1 + v2;
	CHECK(result.X_coord == fixed(5));
	CHECK(result.Y_coord == fixed(7));
	CHECK(result.Z_coord == fixed(9));
}

TEST_CASE("Vector subtraction") {
	Vector3D<fixed> v1{4, 5, 6};
	Vector3D<fixed> v2{1, 2, 3};
	auto            result = v1 - v2;
	CHECK(result.X_coord == fixed(3));
	CHECK(result.Y_coord == fixed(3));
	CHECK(result.Z_coord == fixed(3));
}

TEST_CASE("Vector scalar multiplication") {
	Vector3D<fixed> v{1, 2, 3};
	auto            result = v * fixed(2);
	CHECK(result.X_coord == fixed(2));
	CHECK(result.Y_coord == fixed(4));
	CHECK(result.Z_coord == fixed(6));
}

TEST_CASE("Vector scalar division") {
	Vector3D<fixed> v{2, 4, 6};
	auto            result = v / fixed(2);
	CHECK(result.X_coord == fixed(1));
	CHECK(result.Y_coord == fixed(2));
	CHECK(result.Z_coord == fixed(3));
}

TEST_CASE("Vector dot product") {
	Vector3D<fixed> v1{1, 2, 3};
	Vector3D<fixed> v2{4, 5, 6};
	fixed           result = v1.dot(v2); // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
	CHECK(result == fixed(32));
}

TEST_CASE("Vector cross product") {
	Vector3D<fixed> v1{1, 0, 0};           // i
	Vector3D<fixed> v2{0, 1, 0};           // j
	auto            result = v1.cross(v2); // should be k (0, 0, 1)
	CHECK(result.X_coord == fixed(0));
	CHECK(result.Y_coord == fixed(0));
	CHECK(result.Z_coord == fixed(1));
}

TEST_CASE("Vector magnitude") {
	Vector3D<fixed> v{3, 4, 0};
	fixed           result = v.magnitude(); // sqrt(9 + 16) = 5
	CHECK(result == fixed(5));
}

TEST_CASE("Vector normalization") {
	Vector3D<fixed> v{3, 4, 0};
	auto            result = v.normalize();
	// magnitude should be 1
	CHECK(result.magnitude() == fixed(1));
	// direction should be the same
	CHECK(result.X_coord == fixed(0.6));
	CHECK(result.Y_coord == fixed(0.8));
	CHECK(result.Z_coord == fixed(0));
}
