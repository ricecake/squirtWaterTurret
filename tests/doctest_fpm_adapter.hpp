#pragma once

#include <sstream> // For StringMaker

#include "doctest/doctest.h"
#include "fpm_adapter.hpp"

// The doctest StringMaker needs to be in the doctest namespace
namespace doctest {
	// This allows doctest to print the value of our fixed-point numbers
	template <typename B, typename I, unsigned int F, bool E>
	struct StringMaker<FixedAdapter<B, I, F, E>> {
		static String convert(const FixedAdapter<B, I, F, E>& value) {
			std::stringstream ss;
			ss << value;
			return ss.str().c_str();
		}
	};
} // namespace doctest

// To avoid ambiguous overload errors with the operators in fpm_adapter.hpp,
// we provide explicit overloads for float and double comparisons for each
// of our fixed-point types. These will be selected over the more generic
// templates in fpm_adapter.hpp when dealing with float/double literals in tests.

#define FIXED_POINT_DOCTEST_ADAPTER_IMPL(FIXED_TYPE)                                                                   \
	inline bool operator==(const FIXED_TYPE& lhs, const double& rhs) {                                                 \
		return static_cast<double>(lhs) == doctest::Approx(rhs);                                                       \
	}                                                                                                                  \
	inline bool operator==(const double& lhs, const FIXED_TYPE& rhs) {                                                 \
		return doctest::Approx(lhs) == static_cast<double>(rhs);                                                       \
	}                                                                                                                  \
	inline bool operator==(const FIXED_TYPE& lhs, const float& rhs) {                                                  \
		return static_cast<double>(lhs) == doctest::Approx(static_cast<double>(rhs));                                  \
	}                                                                                                                  \
	inline bool operator==(const float& lhs, const FIXED_TYPE& rhs) {                                                  \
		return doctest::Approx(static_cast<double>(lhs)) == static_cast<double>(rhs);                                  \
	}                                                                                                                  \
	inline bool operator!=(const FIXED_TYPE& lhs, const double& rhs) {                                                 \
		return static_cast<double>(lhs) != doctest::Approx(rhs);                                                       \
	}                                                                                                                  \
	inline bool operator!=(const double& lhs, const FIXED_TYPE& rhs) {                                                 \
		return doctest::Approx(lhs) != static_cast<double>(rhs);                                                       \
	}                                                                                                                  \
	inline bool operator!=(const FIXED_TYPE& lhs, const float& rhs) {                                                  \
		return static_cast<double>(lhs) != doctest::Approx(static_cast<double>(rhs));                                  \
	}                                                                                                                  \
	inline bool operator!=(const float& lhs, const FIXED_TYPE& rhs) {                                                  \
		return doctest::Approx(static_cast<double>(lhs)) != static_cast<double>(rhs);                                  \
	}

FIXED_POINT_DOCTEST_ADAPTER_IMPL(fixed_16_16)
FIXED_POINT_DOCTEST_ADAPTER_IMPL(fixed_24_8)
FIXED_POINT_DOCTEST_ADAPTER_IMPL(fixed_8_24)

#undef FIXED_POINT_DOCTEST_ADAPTER_IMPL
