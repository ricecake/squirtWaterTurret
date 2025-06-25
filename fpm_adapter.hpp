#pragma once

#include "fpm/fixed.hpp"
#include "fpm/math.hpp"

template <
	typename B, typename I, unsigned int F, bool E = true>
class FixedAdapter : public fpm::fixed<B, I, F, E>
{
public:
	template <
		typename NonFixedType>
	constexpr inline FixedAdapter(NonFixedType val) noexcept
		: fpm::fixed<B, I, F, E>(val)
	{
	}
};

namespace std
{
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value> * = nullptr>
	constexpr FixedType max(const FixedType &a, const NonFixedType &b)
	{
		return (a < b) ? FixedType(b) : a;
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value> * = nullptr>
	constexpr FixedType min(const FixedType &a, const NonFixedType &b)
	{
		return (a >= b) ? FixedType(b) : a;
	}
}

namespace fpm
{
	template <typename BaseType, typename IntermediateType, unsigned int FractionBits, bool EnableRounding>
	struct is_fixed<FixedAdapter<BaseType, IntermediateType, FractionBits, EnableRounding>> : std::true_type
	{
	};

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator+=(FixedType &x, const NonFixedType &y) noexcept
	{
		return x += FixedType(y);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator-=(FixedType &x, const NonFixedType &y) noexcept
	{
		return x -= FixedType(y);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator*=(FixedType &x, const NonFixedType &y) noexcept
	{
		return x *= FixedType(y);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator/=(FixedType &x, const NonFixedType &y) noexcept
	{
		return x /= FixedType(y);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator+(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x + FixedType(y);
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator+(const NonFixedType &x, const FixedType &y) noexcept
	{
		return y + FixedType(x);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator-(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x - FixedType(y);
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator-(const NonFixedType &x, const FixedType &y) noexcept
	{
		return y - FixedType(x);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator*(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x * FixedType(y);
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator*(const NonFixedType &x, const FixedType &y) noexcept
	{
		return y * FixedType(x);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator/(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x / FixedType(y);
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<std::is_floating_point<NonFixedType>::value>::type * = nullptr>
	constexpr inline FixedType operator/(const NonFixedType &x, const FixedType &y) noexcept
	{
		return y / FixedType(x);
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator==(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x.raw_value() == FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator==(const NonFixedType &y, const FixedType &x) noexcept
	{
		return x.raw_value() == FixedType(y).raw_value();
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator!=(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x.raw_value() != FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator!=(const NonFixedType &y, const FixedType &x) noexcept
	{
		return x.raw_value() != FixedType(y).raw_value();
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator<(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x.raw_value() < FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator<(const NonFixedType &y, const FixedType &x) noexcept
	{
		return x.raw_value() < FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator<=(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x.raw_value() <= FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator<=(const NonFixedType &y, const FixedType &x) noexcept
	{
		return x.raw_value() <= FixedType(y).raw_value();
	}

	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator>(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x.raw_value() > FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator>(const NonFixedType &y, const FixedType &x) noexcept
	{
		return x.raw_value() > FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator>=(const FixedType &x, const NonFixedType &y) noexcept
	{
		return x.raw_value() >= FixedType(y).raw_value();
	}
	template <
		typename FixedType, typename std::enable_if<fpm::is_fixed<FixedType>::value>::type * = nullptr,
		typename NonFixedType, typename std::enable_if<!fpm::is_fixed<NonFixedType>::value>::type * = nullptr>
	constexpr inline bool operator>=(const NonFixedType &y, const FixedType &x) noexcept
	{
		return x.raw_value() >= FixedType(y).raw_value();
	}
}

using fixed_16_16 = FixedAdapter<std::int32_t, std::int64_t, 16>;
using fixed_24_8 = FixedAdapter<std::int32_t, std::int64_t, 8>;
using fixed_8_24 = FixedAdapter<std::int32_t, std::int64_t, 24>;
