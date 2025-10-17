#include "tests/test_vector.h"

#include <cassert>
#include <cmath>

#include "fpm_adapter.hpp"
#include "vector.hpp"

using fixed = fixed_16_16;

// Helper to check for approximate equality of fixed-point numbers
bool is_close(fixed a, fixed b, fixed tolerance = fixed(0.001)) {
	return fpm::abs(a - b) < tolerance;
}

void test_vector_addition() {
	Vector3D<fixed> v1{1, 2, 3};
	Vector3D<fixed> v2{4, 5, 6};
	auto            result = v1 + v2;
	assert(result.X_coord == fixed(5));
	assert(result.Y_coord == fixed(7));
	assert(result.Z_coord == fixed(9));
}

void test_vector_subtraction() {
	Vector3D<fixed> v1{4, 5, 6};
	Vector3D<fixed> v2{1, 2, 3};
	auto            result = v1 - v2;
	assert(result.X_coord == fixed(3));
	assert(result.Y_coord == fixed(3));
	assert(result.Z_coord == fixed(3));
}

void test_vector_scalar_multiplication() {
	Vector3D<fixed> v{1, 2, 3};
	auto            result = v * fixed(2);
	assert(result.X_coord == fixed(2));
	assert(result.Y_coord == fixed(4));
	assert(result.Z_coord == fixed(6));
}

void test_vector_scalar_division() {
	Vector3D<fixed> v{2, 4, 6};
	auto            result = v / fixed(2);
	assert(result.X_coord == fixed(1));
	assert(result.Y_coord == fixed(2));
	assert(result.Z_coord == fixed(3));
}

void test_vector_dot_product() {
	Vector3D<fixed> v1{1, 2, 3};
	Vector3D<fixed> v2{4, 5, 6};
	fixed           result = v1.dot(v2); // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
	assert(result == fixed(32));
}

void test_vector_cross_product() {
	Vector3D<fixed> v1{1, 0, 0};           // i
	Vector3D<fixed> v2{0, 1, 0};           // j
	auto            result = v1.cross(v2); // should be k (0, 0, 1)
	assert(result.X_coord == fixed(0));
	assert(result.Y_coord == fixed(0));
	assert(result.Z_coord == fixed(1));
}

void test_vector_magnitude() {
	Vector3D<fixed> v{3, 4, 0};
	fixed           result = v.magnitude(); // sqrt(9 + 16) = 5
	assert(is_close(result, fixed(5)));
}

void test_vector_normalization() {
	Vector3D<fixed> v{3, 4, 0};
	auto            result = v.normalize();
	// magnitude should be 1
	assert(is_close(result.magnitude(), fixed(1)));
	// direction should be the same
	assert(is_close(result.X_coord, fixed(0.6)));
	assert(is_close(result.Y_coord, fixed(0.8)));
	assert(is_close(result.Z_coord, fixed(0)));
}

void run_vector_tests() {
	test_vector_addition();
	test_vector_subtraction();
	test_vector_scalar_multiplication();
	test_vector_scalar_division();
	test_vector_dot_product();
	test_vector_cross_product();
	test_vector_magnitude();
	test_vector_normalization();
}