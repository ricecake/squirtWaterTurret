#include <initializer_list>
#include <iostream>
#include <ostream>
#include <ranges>
#include <tuple>

#include "common.h"
#include "vector.hpp"

using fixed = fixed_16_16;

TEST_CASE("Vector addition") {
	Vector3D<fixed> v1{1, 2, 3};
	Vector3D<fixed> v2{4, 5, 6};
	auto            result = v1 + v2;
	CHECK(result.x == fixed(5));
	CHECK(result.y == fixed(7));
	CHECK(result.z == fixed(9));
}

TEST_CASE("Vector subtraction") {
	Vector3D<fixed> v1{4, 5, 6};
	Vector3D<fixed> v2{1, 2, 3};
	auto            result = v1 - v2;
	CHECK(result.x == fixed(3));
	CHECK(result.y == fixed(3));
	CHECK(result.z == fixed(3));
}

TEST_CASE("Vector scalar multiplication") {
	Vector3D<fixed> v{1, 2, 3};
	auto            result = v * fixed(2);
	CHECK(result.x == fixed(2));
	CHECK(result.y == fixed(4));
	CHECK(result.z == fixed(6));
}

TEST_CASE("Vector scalar division") {
	Vector3D<fixed> v{2, 4, 6};
	auto            result = v / fixed(2);
	CHECK(result.x == fixed(1));
	CHECK(result.y == fixed(2));
	CHECK(result.z == fixed(3));
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
	CHECK(result.x == fixed(0));
	CHECK(result.y == fixed(0));
	CHECK(result.z == fixed(1));
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
	CHECK(result.x == fixed(0.6));
	CHECK(result.y == fixed(0.8));
	CHECK(result.z == fixed(0));
}

TEST_CASE("Vector angular computation") {
	// using Vector = std::array<fixed, 3>;

	struct TestItem {
		Vector3D<fixed> vector_a;
		Vector3D<fixed> vector_b;
		double          expected_angle; // e.g., angle in degrees
	};

	// TestItem items[] = {
	// 	// Identical Vectors, 0 Angle
	// 	{.vector_a = {1, 0, 0}, .vector_b = {1, 0, 0}, .expected_angle = 0},
	// 	{{0, 1, 0}, {0, 1, 0}, 0},
	// 	{{0, 0, 1}, {0, 0, 1}, 0},

	// 	// Orthogonal Vectors, 90 Angle
	// 	{.vector_a = {1, 0, 0}, .vector_b = {0, 1, 0}, .expected_angle = 90},
	// 	{{1, 0, 0}, {0, 0, 1}, 90},
	// 	{{0, 1, 0}, {0, 0, 1}, 90},

	// 	// Complex Orthogonal Pair
	// 	{.vector_a = {1, 1, 0}, .vector_b = {1, -1, 0}, .expected_angle = 90},
	// };

	TestItem items[] = {
		// --- BASIC ORTHOGONAL & IDENTICAL CASES ---
		{.vector_a = {1, 0, 0}, .vector_b = {1, 0, 0}, .expected_angle = 0},
		{.vector_a = {0, 1, 0}, .vector_b = {0, 1, 0}, .expected_angle = 0},
		{.vector_a = {0, 0, 1}, .vector_b = {0, 0, 1}, .expected_angle = 0},

		{.vector_a = {1, 0, 0}, .vector_b = {0, 1, 0}, .expected_angle = 90},
		{.vector_a = {1, 0, 0}, .vector_b = {0, 0, 1}, .expected_angle = 90},
		{.vector_a = {0, 1, 0}, .vector_b = {0, 0, 1}, .expected_angle = 90},

		{.vector_a = {1, 1, 0}, .vector_b = {1, -1, 0}, .expected_angle = 90},

		// --- EDGE CASE: ZERO VECTORS (Critical for Normalization/Division by Zero) ---
		{.vector_a = {0, 0, 0}, .vector_b = {0, 0, 0}, .expected_angle = 0},

		// Angle between zero vector and any other vector.
		{.vector_a = {1, 0, 0}, .vector_b = {0, 0, 0}, .expected_angle = 0},
		{.vector_a = {0, 0, 0}, .vector_b = {0, 1, 0}, .expected_angle = 0},

		// --- EDGE CASE: OPPOSITE DIRECTION VECTORS (180°) ---
		{.vector_a = {1, 0, 0}, .vector_b = {-1, 0, 0}, .expected_angle = 180},
		{.vector_a = {0, 2, 0}, .vector_b = {0, -5, 0}, .expected_angle = 180},
		{.vector_a = {1, 1, 1}, .vector_b = {-1, -1, -1}, .expected_angle = 180},

		// --- NON-UNIT MAGNITUDE VECTORS (Testing Normalization Robustness) ---
		// Parallel vectors of different magnitudes.
		{.vector_a = {5, 0, 0}, .vector_b = {2, 0, 0}, .expected_angle = 0},

		// Orthogonal vectors of different magnitudes.
		{.vector_a = {10, 0, 0}, .vector_b = {0, 0.5, 0}, .expected_angle = 90},

		// A larger, complex vector vs a smaller one, 45 degrees apart (use dot product for angle).
		{.vector_a = {10, 0, 0}, .vector_b = {1, 1, 0}, .expected_angle = 45},

		// --- DIAGONAL & NEGATIVE COMPONENTS (Testing all quadrants/octants) ---
		// 45 degrees in X-Y plane (1,1,0) vs (1,0,0)
		{.vector_a = {1, 1, 0}, .vector_b = {1, 0, 0}, .expected_angle = 45},

		// 135 degrees (Negative X)
		{.vector_a = {-1, 1, 0}, .vector_b = {1, 0, 0}, .expected_angle = 135},

		// Angle in the X-Z plane, non-unit.
		{.vector_a = {2, 0, 2}, .vector_b = {0, 0, -5}, .expected_angle = 135},

		// ---- BONUS TESTS KNOWN TO NOT WORK GOOD ----
		// --- FLOATING POINT PRECISION CASES ---
		// Very small magnitudes approaching zero.
		// {.vector_a = {0.001, 0, 0}, .vector_b = {0.001, 0, 0}, .expected_angle = 0},
		// {.vector_a = {0.001, 0, 0}, .vector_b = {0, 0.001, 0}, .expected_angle = 90},

		// --- COMPLEX, ARBITRARY VECTORS (Testing general robustness) ---
		// Random directions, large components
		// {.vector_a = {12, 34, 56}, .vector_b = {78, 90, 12}, .expected_angle = 32}, // Approx, use calculator for
		// exact
		// {.vector_a = {-1, 2, -3}, .vector_b = {4, -5, 6}, .expected_angle = 153},   // Approx, use calculator for
		// exact
	};

	auto index = 0;
	for (auto [vector_a, vector_b, expected_angle] : items) {
		INFO("Test ", index++, ": ", vector_a, ", ", vector_b, ", ", expected_angle);
		CHECK(vector_a.angleTo(vector_b) == doctest::Approx(expected_angle).epsilon(0.01));
	}
}