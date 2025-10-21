/**
 * @file fpm_adapter.hpp
 * @brief An adapter for the fpm fixed-point math library.
 *
 * This file provides a wrapper around the `fpm::fixed` class to enhance its
 * interoperability with standard numeric types. It includes operator overloads
 * and helper functions to make fixed-point arithmetic more seamless.
 */
#pragma once

#include "fpm/fixed.hpp"
#include "fpm/ios.hpp"
#include "fpm/math.hpp"

/**
 * @brief A wrapper class for `fpm::fixed` to provide enhanced functionality.
 *
 * This adapter class inherits from `fpm::fixed` and is intended to simplify
 * the use of fixed-point numbers throughout the codebase.
 */
template <typename B, typename I, unsigned int F, bool E = true>
class FixedAdapter: public fpm::fixed<B, I, F, E> {
	using b = B;
	using i = I;

	static const unsigned int f = F;
	static const bool         e = E;

public:
	inline FixedAdapter() noexcept = default;

	template <typename NonFixedType>
	constexpr inline FixedAdapter(NonFixedType val) noexcept : fpm::fixed<B, I, F, E>(val) {}
};

/**
 * @brief Overloads for standard library functions to support fixed-point types.
 */
namespace std {
	/**
	 * @brief Overload of std::max for comparing a fixed-point type with a non-fixed-point type.
	 * @return The larger of the two values, converted to the fixed-point type.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>* = nullptr>
	constexpr FixedType max(const FixedType& a, const NonFixedType& b) {
		return (a < b) ? FixedType(b) : a;
	}

	/**
	 * @brief Overload of std::min for comparing a fixed-point type with a non-fixed-point type.
	 * @return The smaller of the two values, converted to the fixed-point type.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>* = nullptr>
	constexpr FixedType min(const FixedType& a, const NonFixedType& b) {
		return (a >= b) ? FixedType(b) : a;
	}

	/**
	 * @brief Overload of std::pow for the FixedAdapter type.
	 */
	template <typename T, typename O>
		requires same_as<T, FixedAdapter<typename T::params, typename T::i, T::f, T::e>>
	constexpr FixedAdapter<typename T::params, typename T::i, T::f, T::e> pow(const T& A, const O& B) {
		return FixedAdapter<typename T::params, typename T::i, T::f, T::e>(fpm::pow(A, B));
	}
} // namespace std

/**
 * @brief Overloads and specializations for the fpm library.
 */
namespace fpm {
	/**
	 * @brief Specialization of fpm::is_fixed for the FixedAdapter type.
	 */
	template <typename BaseType, typename IntermediateType, unsigned int FractionBits, bool EnableRounding>
	struct is_fixed<FixedAdapter<BaseType, IntermediateType, FractionBits, EnableRounding>>: std::true_type {};

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator+=(FixedType& x, const NonFixedType& y) noexcept {
		return x += FixedType(y);
	}

	/**
	 * @brief Subtracts a floating-point value from a fixed-point value and assigns the result.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator-=(FixedType& x, const NonFixedType& y) noexcept {
		return x -= FixedType(y);
	}

	/**
	 * @brief Multiplies a fixed-point value by a floating-point value and assigns the result.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator*=(FixedType& x, const NonFixedType& y) noexcept {
		return x *= FixedType(y);
	}

	/**
	 * @brief Divides a fixed-point value by a floating-point value and assigns the result.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator/=(FixedType& x, const NonFixedType& y) noexcept {
		return x /= FixedType(y);
	}

	/**
	 * @brief Adds a fixed-point and a floating-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator+(const FixedType& x, const NonFixedType& y) noexcept {
		return x + FixedType(y);
	}
	/**
	 * @brief Adds a floating-point and a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator+(const NonFixedType& x, const FixedType& y) noexcept {
		return y + FixedType(x);
	}

	/**
	 * @brief Subtracts a floating-point value from a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator-(const FixedType& x, const NonFixedType& y) noexcept {
		return x - FixedType(y);
	}
	/**
	 * @brief Subtracts a fixed-point value from a floating-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator-(const NonFixedType& x, const FixedType& y) noexcept {
		return y - FixedType(x);
	}

	/**
	 * @brief Multiplies a fixed-point value by a floating-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator*(const FixedType& x, const NonFixedType& y) noexcept {
		return x * FixedType(y);
	}
	/**
	 * @brief Multiplies a floating-point value by a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator*(const NonFixedType& x, const FixedType& y) noexcept {
		return y * FixedType(x);
	}

	/**
	 * @brief Divides a fixed-point value by a floating-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator/(const FixedType& x, const NonFixedType& y) noexcept {
		return x / FixedType(y);
	}
	/**
	 * @brief Divides a floating-point value by a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator/(const NonFixedType& x, const FixedType& y) noexcept {
		return y / FixedType(x);
	}

	/**
	 * @brief Compares a fixed-point and a non-fixed-point value for equality.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator==(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() == FixedType(y).raw_value();
	}
	/**
	 * @brief Compares a non-fixed-point and a fixed-point value for equality.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator==(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() == FixedType(y).raw_value();
	}

	/**
	 * @brief Compares a fixed-point and a non-fixed-point value for inequality.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator!=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() != FixedType(y).raw_value();
	}
	/**
	 * @brief Compares a non-fixed-point and a fixed-point value for inequality.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator!=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() != FixedType(y).raw_value();
	}

	/**
	 * @brief Checks if a fixed-point value is less than a non-fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() < FixedType(y).raw_value();
	}
	/**
	 * @brief Checks if a non-fixed-point value is less than a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() < FixedType(y).raw_value();
	}
	/**
	 * @brief Checks if a fixed-point value is less than or equal to a non-fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() <= FixedType(y).raw_value();
	}
	/**
	 * @brief Checks if a non-fixed-point value is less than or equal to a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() <= FixedType(y).raw_value();
	}

	/**
	 * @brief Checks if a fixed-point value is greater than a non-fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() > FixedType(y).raw_value();
	}
	/**
	 * @brief Checks if a non-fixed-point value is greater than a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() > FixedType(y).raw_value();
	}
	/**
	 * @brief Checks if a fixed-point value is greater than or equal to a non-fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() >= FixedType(y).raw_value();
	}
	/**
	 * @brief Checks if a non-fixed-point value is greater than or equal to a fixed-point value.
	 */
	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() >= FixedType(y).raw_value();
	}
} // namespace fpm

using fixed_16_16 = FixedAdapter<std::int32_t, std::int64_t, 16>;
using fixed_24_8 = FixedAdapter<std::int32_t, std::int64_t, 8>;
using fixed_8_24 = FixedAdapter<std::int32_t, std::int64_t, 24>;
