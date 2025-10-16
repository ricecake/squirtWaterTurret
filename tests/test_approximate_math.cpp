#include "tests/test_approximate_math.h"

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"

#include <cassert>
#include <cmath>

// Test for the small_root function
void test_small_root() {
	// Test with a simple linear function f(x) = x - 5
	std::function<const fixed(const fixed)> linear_func = [](const fixed x) -> const fixed {
		return x - fixed(5);
	};

	auto result = Approximate::small_root(linear_func);

	assert(result.converged);
	// Check if the result is close to 5, allowing for some error
	assert(fpm::abs(result.result - fixed(5)) < fixed(0.01));
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
	test_sin();
	test_cos();
	test_tan();
	test_atan();
	test_sqrt();
}