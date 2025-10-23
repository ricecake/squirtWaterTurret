#include "tests/test_approximate_math.h"

#include <cassert>
#include <cmath>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"

// Test for the small_root function
void test_small_root() {
	// Test with a simple linear function f(x) = x - 5
	std::function<const fixed(const fixed)> linear_func = [](const fixed x) -> const fixed { return x - fixed(5); };

	auto result = Approximate::small_root(linear_func);

	assert(result.converged);
	// Check if the result is close to 5, allowing for some error
	assert(fpm::abs(result.result - fixed(5)) < fixed(0.01));
}

// Test for the n_roots function
void test_n_roots() {
	// Test with a sine wave function f(x) = sin(x), which has roots at n*pi
	// Note: The built-in sin function takes a double, so we will use that for this test.
	std::function<const double(const double)> sin_func = [](const double x) -> const double { return std::sin(x); };

	// Find the first two roots (pi and 2*pi)
	auto result = Approximate::n_roots(sin_func, 2);

	assert(result.converged);
	assert(result.result.size() == 2);

	// Check if the found roots are close to pi and 2*pi
	const double pi = 3.14159265358979323846;
	assert(std::abs(result.result[0] - pi) < 0.01);
	assert(std::abs(result.result[1] - (2 * pi)) < 0.01);
}

// Placeholder tests for trigonometric functions
// These will be properly implemented once the function definitions are available.
void test_sin() {
	// assert(Approximate::sin(fixed(0)) == fixed(0));
}

void test_cos() {
	// assert(Approximate::cos(fixed(0)) == fixed(1));
}

void test_tan() {
	// assert(Approximate::tan(fixed(0)) == fixed(0));
}

void test_atan() {
	// assert(Approximate::atan(fixed(0)) == fixed(0));
}

void test_sqrt() {
	// assert(Approximate::sqrt(fixed(4)) == fixed(2));
}

void run_approximate_math_tests() {
	test_small_root();
	test_n_roots();
	test_sin();
	test_cos();
	test_tan();
	test_atan();
	test_sqrt();
}