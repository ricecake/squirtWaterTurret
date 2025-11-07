#include <cmath>
#include <iostream>
#include <ostream>

#include "aproximate_math.hpp"
#include "doctest/doctest.h"
#include "fpm_adapter.hpp"

// Test for the small_root function
TEST_CASE("small_root") {
	// Test with a simple linear function f(x) = x - 5
	std::function<fixed(const fixed)> linear_func = [](const fixed x) -> fixed { return x - fixed(5); };

	auto result = Approximate::small_root(linear_func);

	CHECK(result.converged);
	// Check if the result is close to 5, allowing for some error
	CHECK(fpm::abs(result.result - fixed(5)) < fixed(0.01));
}

// Test for the n_roots function
TEST_CASE("n_roots") {
	// Test with a sine wave function f(x) = sin(x), which has roots at n*pi
	// Note: The built-in sin function takes a double, so we will use that for this test.
	std::function<double(const double)> sin_func = [](const double x) -> double { return std::sin(x); };

	// Find the first two roots (pi and 2*pi)
	auto result = Approximate::n_roots(sin_func, 2);

	CHECK(result.converged);
	CHECK(result.result.size() == 2);

	// Check if the found roots are close to pi and 2*pi
	const double pi = 3.14159265358979323846;
	CHECK(std::abs(result.result[0] - pi) < 0.01);
	CHECK(std::abs(result.result[1] - (2 * pi)) < 0.01);
}

// Placeholder tests for trigonometric functions
// These will be properly implemented once the function definitions are available.
TEST_CASE("sin") {
	// CHECK(Approximate::sin(fixed(0)) == fixed(0));
}

TEST_CASE("cos") {
	// CHECK(Approximate::cos(fixed(0)) == fixed(1));
}

TEST_CASE("tan") {
	// CHECK(Approximate::tan(fixed(0)) == fixed(0));
}

TEST_CASE("atan") {
	// CHECK(Approximate::atan(fixed(0)) == fixed(0));
}

TEST_CASE("sqrt") {
	// CHECK(Approximate::sqrt(fixed(4)) == fixed(2));
}
