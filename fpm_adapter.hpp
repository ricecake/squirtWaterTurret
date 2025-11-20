/**
 * @file fpm_adapter.hpp
 * @brief An adapter for the fpm fixed-point math library.
 *
 * This file provides a wrapper around the `fpm::fixed` class to enhance its
 * interoperability with standard numeric types. It includes operator overloads
 * and helper functions to make fixed-point arithmetic more seamless.
 */
#pragma once
#include <compare>     // For std::strong_ordering (operator<=>)
#include <concepts>    // For std::integral
#include <iostream>    // For the std::ostream operator
#include <limits>      // For std::numeric_limits
#include <stdexcept>   // For std::overflow_error, std::underflow_error
#include <type_traits> // For std::is_same_v, std::is_signed_v

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
public:
	using IntermediateType = I;
	static const int FixedBits = F;

	inline FixedAdapter() noexcept = default;

	template <typename NonFixedType>
	constexpr inline FixedAdapter(NonFixedType val) noexcept: fpm::fixed<B, I, F, E>(val) {}

	[[nodiscard]] auto operator<=>(const auto& other) const noexcept {
		return this->raw_value() <=> static_cast<decltype(*this)>(other).raw_value();
	}

	constexpr inline operator bool() const { return bool(this->raw_value()); }
};

template <typename T>
concept IsFixedPoint = fpm::is_fixed_v<T>;

/**
 * @brief Overloads for standard library functions to support fixed-point types.
 */
namespace std {
	/**
	 * @brief Overload of std::max for fixed-point and non-fixed-point types.
	 */
	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr FixedType max(const FixedType& a, const NonFixedType& b) {
		return (a < b) ? FixedType(b) : a;
	}

	/**
	 * @brief Overload of std::min for fixed-point and non-fixed-point types.
	 */
	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr FixedType min(const FixedType& a, const NonFixedType& b) {
		return (a >= b) ? FixedType(b) : a;
	}

	/**
	 * @brief Overload of std::pow for the FixedAdapter type.
	 */
	template <typename T, typename O>
		requires IsFixedPoint<T>
	constexpr T pow(const T& A, const O& B) {
		return T(fpm::pow(A, B));
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

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator+=(FixedType& x, const NonFixedType& y) noexcept {
		return x += FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator-=(FixedType& x, const NonFixedType& y) noexcept {
		return x -= FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator*=(FixedType& x, const NonFixedType& y) noexcept {
		return x *= FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator/=(FixedType& x, const NonFixedType& y) noexcept {
		return x /= FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator+(const FixedType& x, const NonFixedType& y) noexcept {
		return x + FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator+(const NonFixedType& x, const FixedType& y) noexcept {
		return FixedType(x) + y;
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator-(const FixedType& x, const NonFixedType& y) noexcept {
		return x - FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator-(const NonFixedType& x, const FixedType& y) noexcept {
		return FixedType(x) - y;
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator*(const FixedType& x, const NonFixedType& y) noexcept {
		return x * FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator*(const NonFixedType& x, const FixedType& y) noexcept {
		return FixedType(x) * y;
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator/(const FixedType& x, const NonFixedType& y) noexcept {
		return x / FixedType(y);
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && std::is_floating_point_v<NonFixedType>
	constexpr inline FixedType operator/(const NonFixedType& x, const FixedType& y) noexcept {
		return FixedType(x) / y;
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator==(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() == FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator==(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() == FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator!=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() != FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator!=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() != FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator<(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() < FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator<(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() < FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator<=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() <= FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator<=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() <= FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator>(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() > FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator>(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() > FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator>=(const FixedType& x, const NonFixedType& y) noexcept {
		return x.raw_value() >= FixedType(y).raw_value();
	}

	template <typename FixedType, typename NonFixedType>
		requires IsFixedPoint<FixedType> && (!IsFixedPoint<NonFixedType>)
	constexpr inline bool operator>=(const NonFixedType& y, const FixedType& x) noexcept {
		return x.raw_value() >= FixedType(y).raw_value();
	}
} // namespace fpm

// Specializations for customization points
// Note: The following templates are specialized for the `FixedAdapter` class.
// Due to C++ template specialization rules, these specializations will not
// automatically apply to classes that inherit from `FixedAdapter`. To extend this
// functionality to a subclass of `FixedAdapter`, you can inherit from the
// `detail::HashBase` and `detail::NumericLimitsBase` helper structs.
//
// For example:
//
//   class MyFixed: public FixedAdapter<int32_t, int64_t, 16> {};
//
//   namespace std {
//     template <>
//     struct hash<MyFixed>: public detail::HashBase<MyFixed> {};
//     template <>
//     struct numeric_limits<MyFixed>: public detail::NumericLimitsBase<MyFixed> {};
//   }
//

namespace detail {
	template <IsFixedPoint FixedType>
	struct NumericLimitsBase {
		using B = typename FixedType::base_type;
		using I = typename FixedType::intermediate_type;
		static constexpr unsigned int F = FixedType::fraction_bits;

		inline static constexpr bool                    is_specialized = true;
		inline static constexpr bool                    is_signed = std::numeric_limits<B>::is_signed;
		inline static constexpr bool                    is_integer = false;
		inline static constexpr bool                    is_exact = true;
		inline static constexpr bool                    has_infinity = false;
		inline static constexpr bool                    has_quiet_NaN = false;
		inline static constexpr bool                    has_signaling_NaN = false;
		inline static constexpr std::float_denorm_style has_denorm = std::denorm_absent;
		inline static constexpr bool                    has_denorm_loss = false;
		inline static constexpr std::float_round_style  round_style = std::round_to_nearest;
		inline static constexpr bool                    is_iec559 = false;
		inline static constexpr bool                    is_bounded = true;
		inline static constexpr bool                    is_modulo = std::numeric_limits<B>::is_modulo;
		inline static constexpr int                     digits = std::numeric_limits<B>::digits;
		inline static constexpr int                     digits10 = 1;
		inline static constexpr int max_digits10 = fpm::detail::max_digits10(std::numeric_limits<B>::digits - F) +
			fpm::detail::max_digits10(F);
		inline static constexpr int  radix = 2;
		inline static constexpr int  min_exponent = 1 - F;
		inline static constexpr int  min_exponent10 = -fpm::detail::digits10(F);
		inline static constexpr int  max_exponent = std::numeric_limits<B>::digits - F;
		inline static constexpr int  max_exponent10 = fpm::detail::digits10(std::numeric_limits<B>::digits - F);
		inline static constexpr bool traps = true;
		inline static constexpr bool tinyness_before = false;

		static constexpr FixedType lowest() noexcept {
			return FixedType::from_raw_value(std::numeric_limits<B>::lowest());
		};

		static constexpr FixedType min() noexcept { return lowest(); }

		static constexpr FixedType max() noexcept { return FixedType::from_raw_value(std::numeric_limits<B>::max()); };

		static constexpr FixedType epsilon() noexcept { return FixedType::from_raw_value(1); };

		static constexpr FixedType round_error() noexcept { return FixedType(1) / 2; };

		static constexpr FixedType denorm_min() noexcept { return min(); }
	};

	template <IsFixedPoint FixedType>
	struct HashBase {
		using argument_type = FixedType;
		using result_type = std::size_t;
		using B = typename FixedType::base_type;

		result_type
		operator()(argument_type arg) const noexcept(noexcept(std::declval<std::hash<B>>()(arg.raw_value()))) {
			return m_hash(arg.raw_value());
		}

	private:
		std::hash<B> m_hash;
	};
} // namespace detail

namespace std {

	template <typename B, typename I, unsigned int F, bool R>
	struct hash<FixedAdapter<B, I, F, R>>: public detail::HashBase<FixedAdapter<B, I, F, R>> {};

	template <typename B, typename I, unsigned int F, bool R>
	struct numeric_limits<FixedAdapter<B, I, F, R>>: public detail::NumericLimitsBase<FixedAdapter<B, I, F, R>> {};

} // namespace std

using fixed_16_16 = FixedAdapter<std::int32_t, std::int64_t, 16>;
using fixed_24_8 = FixedAdapter<std::int32_t, std::int64_t, 8>;
using fixed_8_24 = FixedAdapter<std::int32_t, std::int64_t, 24>;
