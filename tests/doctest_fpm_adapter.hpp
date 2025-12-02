#pragma once

#include <sstream> // For StringMaker

#include "common.h"
// The fpm library has inherent shadowing and sign-conversion issues.
// Suppress these warnings when including fpm headers.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#include "fpm/fixed.hpp"
#pragma GCC diagnostic pop

template <class FixedType>
concept FixedNumber = requires { fpm::is_fixed_v<FixedType> || std::is_floating_point_v<FixedType>; };

// The doctest StringMaker needs to be in the doctest namespace
namespace doctest {
	// This allows doctest to print the value of our fixed-point numbers
	template <>
	struct StringMaker<class FixedType> {
		static String convert(const FixedNumber auto& value) {
			std::stringstream ss;
			ss << value;
			return ss.str().c_str();
		}
	};
} // namespace doctest

template <FixedNumber FixedType>
doctest::Approx Approx(const FixedType& val) {
	return doctest::Approx(double(val));
}

template <FixedNumber FixedType>
inline bool operator==(const FixedType& lhs, const doctest::Approx& rhs) {
	return static_cast<double>(lhs) == rhs;
}

template <FixedNumber FixedType>
inline bool operator==(const doctest::Approx& lhs, const FixedType& rhs) {
	return lhs == static_cast<double>(rhs);
}

template <FixedNumber FixedType>
inline bool operator!=(const FixedType& lhs, const doctest::Approx& rhs) {
	return static_cast<double>(lhs) != rhs;
}

template <FixedNumber FixedType>
inline bool operator!=(const doctest::Approx& lhs, const FixedType& rhs) {
	return lhs != static_cast<double>(rhs);
}
