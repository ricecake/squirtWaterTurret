#include "doctest_fpm_adapter.hpp"

TEST_CASE("doctest_fpm_adapter") {
	SUBCASE("Approximate comparison with floating-point numbers") {
		fixed_16_16 a = 1.234;
		double      b = 1.234;
		double      c = 1.235;

		CHECK(a == b);
		CHECK(b == a);
		CHECK(a != c);
		CHECK(c != a);
	}

	SUBCASE("String conversion for fixed-point numbers") {
		fixed_16_16       a = 3.14159;
		std::stringstream ss;
		ss << a;
		std::string s = ss.str();
		CHECK(s == "3.14159");
	}
}
