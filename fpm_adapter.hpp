#include "fpm/fixed.hpp"
#include "fpm/math.hpp"

template <typename B, typename I, unsigned int F, bool E = true>
class fixed : public fpm::fixed<B, I, F, E>
{
public:
	template <typename T>
	constexpr inline fixed(T val) noexcept
		: fpm::fixed<B, I, F, E>(val)
	{
	}
};

namespace fpm
{
	template <typename B, typename I, unsigned int F, bool R,
			  typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator+=(fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x += fixed<B, I, F, R>(y);
	}
	template <typename B, typename I, unsigned int F, bool R,
			  typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator-=(fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x -= fixed<B, I, F, R>(y);
	}
	template <typename B, typename I, unsigned int F, bool R,
			  typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator*=(fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x *= fixed<B, I, F, R>(y);
	}
	template <typename B, typename I, unsigned int F, bool R,
			  typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator/=(fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x /= fixed<B, I, F, R>(y);
	}

	template <typename B, typename I, unsigned int F, bool R, typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator+(const fixed<B, I, F, R> &x, T y) noexcept
	{
		return x + fixed<B, I, F, R>(y);
	}

	template <typename B, typename I, unsigned int F, bool R, typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator+(T x, const fixed<B, I, F, R> &y) noexcept
	{
		return y + fixed<B, I, F, R>(x);
	}

	template <typename B, typename I, unsigned int F, bool R, typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator-(const fixed<B, I, F, R> &x, T y) noexcept
	{
		return x - fixed<B, I, F, R>(y);
	}

	template <typename B, typename I, unsigned int F, bool R, typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator-(T x, const fixed<B, I, F, R> &y) noexcept
	{
		return y - fixed<B, I, F, R>(x);
	}

	template <typename B, typename I, unsigned int F, bool R, typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator*(const fixed<B, I, F, R> &x, T y) noexcept
	{
		return x * fixed<B, I, F, R>(y);
	}

	template <typename B, typename I, unsigned int F, bool R, typename T, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator*(T x, const fixed<B, I, F, R> &y) noexcept
	{
		return y * fixed<B, I, F, R>(x);
	}

	template <typename B, typename I, unsigned int F, typename T, bool R, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator/(const fixed<B, I, F, R> &x, T y) noexcept
	{
		return x / fixed<B, I, F, R>(y);
	}

	template <typename B, typename I, unsigned int F, typename T, bool R, typename std::enable_if<std::is_floating_point<T>::value>::type * = nullptr>
	constexpr inline fixed<B, I, F, R> operator/(T x, const fixed<B, I, F, R> &y) noexcept
	{
		return y / fixed<B, I, F, R>(x);
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator==(const fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x.raw_value() == fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator!=(const fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x.raw_value() != fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator<(const fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x.raw_value() < fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator>(const fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x.raw_value() > fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator<=(const fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x.raw_value() <= fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator>=(const fixed<B, I, F, R> &x, const T &y) noexcept
	{
		return x.raw_value() >= fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator==(const T &y, const fixed<B, I, F, R> &x) noexcept
	{
		return x.raw_value() == fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator!=(const T &y, const fixed<B, I, F, R> &x) noexcept
	{
		return x.raw_value() != fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator<(const T &y, const fixed<B, I, F, R> &x) noexcept
	{
		return x.raw_value() < fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator>(const T &y, const fixed<B, I, F, R> &x) noexcept
	{
		return x.raw_value() > fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator<=(const T &y, const fixed<B, I, F, R> &x) noexcept
	{
		return x.raw_value() <= fixed<B, I, F, R>(y).raw_value();
	}

	template <typename B, typename I, typename T, unsigned int F, bool R>
	constexpr inline bool operator>=(const T &y, const fixed<B, I, F, R> &x) noexcept
	{
		return x.raw_value() >= fixed<B, I, F, R>(y).raw_value();
	}
}

using fixed_16_16 = fixed<std::int32_t, std::int64_t, 16>;
using fixed_24_8 = fixed<std::int32_t, std::int64_t, 8>;
using fixed_8_24 = fixed<std::int32_t, std::int64_t, 24>;
