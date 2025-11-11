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
	constexpr inline FixedAdapter(NonFixedType val) noexcept: fpm::fixed<B, I, F, E>(val) {}
};

/**
 * @brief Overloads for standard library functions to support fixed-point types.
 */
namespace std {
	/**
	 * @brief Overload of std::max for fixed-point and non-fixed-point types.
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
	 * @brief Overload of std::min for fixed-point and non-fixed-point types.
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

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator-=(FixedType& x, const NonFixedType& y) noexcept {
		return x -= FixedType(y);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator*=(FixedType& x, const NonFixedType& y) noexcept {
		return x *= FixedType(y);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator/=(FixedType& x, const NonFixedType& y) noexcept {
		return x /= FixedType(y);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator+(const FixedType& x, const NonFixedType& y) noexcept {
		return x + FixedType(y);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator+(const NonFixedType& x, const FixedType& y) noexcept {
		return y + FixedType(x);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator-(const FixedType& x, const NonFixedType& y) noexcept {
		return x - FixedType(y);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator-(const NonFixedType& x, const FixedType& y) noexcept {
		return y - FixedType(x);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator*(const FixedType& x, const NonFixedType& y) noexcept {
		return x * FixedType(y);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator*(const NonFixedType& x, const FixedType& y) noexcept {
		return y * FixedType(x);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator/(const FixedType& x, const NonFixedType& y) noexcept {
		return x / FixedType(y);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type* = nullptr>
	constexpr inline FixedType operator/(const NonFixedType& x, const FixedType& y) noexcept {
		return y / FixedType(x);
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator==(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() == FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator==(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() == FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator!=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() != FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator!=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() != FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() < FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() < FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() <= FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator<=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() <= FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() > FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() > FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() >= FixedType(y).raw_value();
	}

	template <
		typename FixedType,
		typename std::enable_if<fpm::is_fixed<FixedType>::value>::type* = nullptr,
		typename NonFixedType,
		typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type* = nullptr>
	constexpr inline bool operator>=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() >= FixedType(y).raw_value();
	}
} // namespace fpm

// Specializations for customization points
namespace std {

	template <typename B, typename I, unsigned int F, bool R>
	struct hash<FixedAdapter<B, I, F, R>> {
		using argument_type = FixedAdapter<B, I, F, R>;
		using result_type = std::size_t;

		result_type
		operator()(argument_type arg) const noexcept(noexcept(std::declval<std::hash<B>>()(arg.raw_value()))) {
			return m_hash(arg.raw_value());
		}

	private:
		std::hash<B> m_hash;
	};

	template <typename B, typename I, unsigned int F, bool R>
	struct numeric_limits<FixedAdapter<B, I, F, R>> {
		static constexpr bool                    is_specialized = true;
		static constexpr bool                    is_signed = std::numeric_limits<B>::is_signed;
		static constexpr bool                    is_integer = false;
		static constexpr bool                    is_exact = true;
		static constexpr bool                    has_infinity = false;
		static constexpr bool                    has_quiet_NaN = false;
		static constexpr bool                    has_signaling_NaN = false;
		static constexpr std::float_denorm_style has_denorm = std::denorm_absent;
		static constexpr bool                    has_denorm_loss = false;
		static constexpr std::float_round_style  round_style = std::round_to_nearest;
		static constexpr bool                    is_iec559 = false;
		static constexpr bool                    is_bounded = true;
		static constexpr bool                    is_modulo = std::numeric_limits<B>::is_modulo;
		static constexpr int                     digits = std::numeric_limits<B>::digits;

		// Any number with `digits10` significant base-10 digits (that fits in
		// the range of the type) is guaranteed to be convertible from text and
		// back without change. Worst case, this is 0.000...001, so we can only
		// guarantee this case. Nothing more.
		static constexpr int digits10 = 1;

		// This is equal to max_digits10 for the integer and fractional part together.
		static constexpr int max_digits10 = fpm::detail::max_digits10(std::numeric_limits<B>::digits - F) +
			fpm::detail::max_digits10(F);

		static constexpr int  radix = 2;
		static constexpr int  min_exponent = 1 - F;
		static constexpr int  min_exponent10 = -fpm::detail::digits10(F);
		static constexpr int  max_exponent = std::numeric_limits<B>::digits - F;
		static constexpr int  max_exponent10 = fpm::detail::digits10(std::numeric_limits<B>::digits - F);
		static constexpr bool traps = true;
		static constexpr bool tinyness_before = false;

		static constexpr FixedAdapter<B, I, F, R> lowest() noexcept {
			return FixedAdapter<B, I, F, R>::from_raw_value(std::numeric_limits<B>::lowest());
		};

		static constexpr FixedAdapter<B, I, F, R> min() noexcept { return lowest(); }

		static constexpr FixedAdapter<B, I, F, R> max() noexcept {
			return FixedAdapter<B, I, F, R>::from_raw_value(std::numeric_limits<B>::max());
		};

		static constexpr FixedAdapter<B, I, F, R> epsilon() noexcept {
			return FixedAdapter<B, I, F, R>::from_raw_value(1);
		};

		static constexpr FixedAdapter<B, I, F, R> round_error() noexcept { return FixedAdapter<B, I, F, R>(1) / 2; };

		static constexpr FixedAdapter<B, I, F, R> denorm_min() noexcept { return min(); }
	};

	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::is_specialized;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::is_signed;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::is_integer;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::is_exact;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::has_infinity;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::has_quiet_NaN;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::has_signaling_NaN;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr std::float_denorm_style numeric_limits<FixedAdapter<B, I, F, R>>::has_denorm;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::has_denorm_loss;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr std::float_round_style numeric_limits<FixedAdapter<B, I, F, R>>::round_style;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::is_iec559;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::is_bounded;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::is_modulo;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::digits;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::digits10;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::max_digits10;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::radix;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::min_exponent;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::min_exponent10;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::max_exponent;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr int numeric_limits<FixedAdapter<B, I, F, R>>::max_exponent10;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::traps;
	template <typename B, typename I, unsigned int F, bool R>
	constexpr bool numeric_limits<FixedAdapter<B, I, F, R>>::tinyness_before;

} // namespace std

using fixed_16_16 = FixedAdapter<std::int32_t, std::int64_t, 16>;
using fixed_24_8 = FixedAdapter<std::int32_t, std::int64_t, 8>;
using fixed_8_24 = FixedAdapter<std::int32_t, std::int64_t, 24>;
