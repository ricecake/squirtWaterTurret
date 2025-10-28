#include "doctest.h"
#include <cmath>
#include <functional>

#include "aproximate_math.hpp"
#include "fpm_adapter.hpp"

TEST_CASE("Test small_root") {
    // Test with a simple linear function f(x) = x - 5
    std::function<const fixed(const fixed)> linear_func = [](const fixed x) -> const fixed { return x - fixed(5); };

    auto result = Approximate::small_root(linear_func);

    REQUIRE(result.converged);
    // Check if the result is close to 5, allowing for some error
    REQUIRE(fpm::abs(result.result - fixed(5)) < fixed(0.01));
}

TEST_CASE("Test n_roots") {
    // Test with a sine wave function f(x) = sin(x), which has roots at n*pi
    // Note: The built-in sin function takes a double, so we will use that for this test.
    std::function<const double(const double)> sin_func = [](const double x) -> const double { return std::sin(x); };

    // Find the first two roots (pi and 2*pi)
    auto result = Approximate::n_roots(sin_func, 2);

    REQUIRE(result.converged);
    REQUIRE(result.result.size() == 2);

    // Check if the found roots are close to pi and 2*pi
    const double pi = 3.14159265358979323846;
    CHECK(std::abs(result.result[0] - pi) < 0.01);
    CHECK(std::abs(result.result[1] - (2 * pi)) < 0.01);
}

// Placeholder tests for trigonometric functions
// These will be properly implemented once the function definitions are available.
TEST_CASE("Test sin") {
    // REQUIRE(Approximate::sin(fixed(0)) == fixed(0));
}

TEST_CASE("Test cos") {
    // REQUIRE(Approximate::cos(fixed(0)) == fixed(1));
}

TEST_CASE("Test tan") {
    // REQUIRE(Approximate::tan(fixed(0)) == fixed(0));
}

TEST_CASE("Test atan") {
    // REQUIRE(Approximate::atan(fixed(0)) == fixed(0));
}

TEST_CASE("Test sqrt") {
    // REQUIRE(Approximate::sqrt(fixed(4)) == fixed(2));
}
