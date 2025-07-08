#pragma once

#include "fpm_adapter.hpp"
#include <chrono>

// struct FixedReprHighResClock : public std::chrono::high_resolution_clock {
// 	using rep = fixed;
// 	using duration = std::chrono::duration<rep, std::nano>;
// 	using time_point = std::chrono::time_point<FixedReprHighResClock, duration>;
// };

using Clock = std::chrono::high_resolution_clock;
// using Clock = FixedReprHighResClock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

// class VDur : public std::chrono::duration<fixed>
// {
// 	public:
// 	VDur(fixed t): std::chrono::duration<fixed>(t){};
// };

template <typename Rep, typename Period = std::ratio<1>>
class DynamicTimeInterval
{
public:
	using DurationType = std::chrono::duration<Rep, Period>;
	using rep = DurationType::rep;
	using period = DurationType::period;

	DynamicTimeInterval(const DynamicTimeInterval &other) = default;

	// Constructor to initialize with a std::chrono::duration
	constexpr DynamicTimeInterval(DurationType d) : m_duration(d) {}

	// Constructor to initialize with a raw count
	constexpr DynamicTimeInterval(Rep count) : m_duration(count) {}

	template <typename Orep, typename Operiod>
	constexpr DynamicTimeInterval(std::chrono::duration<Orep, Operiod> d) : m_duration(std::chrono::duration_cast<DurationType>(d))
	{
	}

	// Get the underlying duration
	constexpr DurationType get_duration() const
	{
		return m_duration;
	}

	// Get the count of ticks
	constexpr Rep count() const
	{
		return m_duration.count();
	}

	constexpr uint64_t microseconds() const {
		return std::chrono::duration_cast<
			std::chrono::duration<uint64_t, std::micro>
		>(m_duration).count();
	}

	// Example of a conversion utility
	template <typename TargetRep, typename TargetPeriod>
	constexpr DynamicTimeInterval<TargetRep, TargetPeriod> as() const
	{
		return DynamicTimeInterval<TargetRep, TargetPeriod>(
			std::chrono::duration_cast<std::chrono::duration<TargetRep, TargetPeriod>>(m_duration));
	}

	template <typename T, typename P>
	constexpr DynamicTimeInterval operator+(const DynamicTimeInterval<T, P> &other) {
		return DynamicTimeInterval(m_duration + std::chrono::duration_cast<std::chrono::duration<Rep, Period>>(other.m_duration));
	}

	constexpr bool operator>(const DynamicTimeInterval &other) {
		return m_duration > other.m_duration;
	}

private:
	const DurationType m_duration;
};

using TimeInterval = DynamicTimeInterval<fixed_16_16, std::ratio<1>>;

template <typename T>
concept ChronoDuration = requires(T obj) {
	// typename TimeInterval<typename T::rep, typename T::period>;
	// std::is_base_of_v<TimeInterval<typename T::rep, typename T::period>, T>;
	requires std::same_as<DynamicTimeInterval<typename T::rep, typename T::period>, T>;
};

template <typename T>
concept ChronoPoint = std::is_base_of_v<std::chrono::time_point<typename T::Clock, typename T::Duration>, T>;

constexpr auto AsSeconds = [](const ChronoDuration auto &d)
{ return std::chrono::duration_cast<std::chrono::duration<fixed_16_16>>(d); };

// Define motor interface type
const int motorInterfaceType = 1;
const int maxSpeed = 1000;	   // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
const int acceleration = 3000; // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
constexpr static fixed_16_16 rad2DegFactor = fixed_16_16(57.2957795131);
const fixed_16_16 Gz = -9.80665;
const int altitude = 1320;
const fixed_16_16 projectileSpeed = 20;

template <typename T, typename P>
constexpr DynamicTimeInterval<fixed_16_16, std::milli> milliseconds(const fixed_16_16 millis, const DynamicTimeInterval<T, P> offset)
{
	return DynamicTimeInterval<fixed_16_16, std::milli>(millis) + DynamicTimeInterval<fixed_16_16, std::milli>(offset);
}

constexpr DynamicTimeInterval<fixed_16_16, std::milli> milliseconds(const fixed_16_16 millis)
{
	return std::chrono::duration<fixed_16_16, std::milli>(millis);
}


template <typename T, typename P>
constexpr DynamicTimeInterval<fixed_16_16> seconds(fixed_16_16 seconds, DynamicTimeInterval<T, P> offset)
{
	return DynamicTimeInterval<fixed_16_16>(seconds) + DynamicTimeInterval<fixed_16_16, std::ratio<1>>(offset);
}

constexpr DynamicTimeInterval<fixed_16_16> seconds(fixed_16_16 seconds)
{
	return std::chrono::duration<fixed_16_16, std::ratio<1>>(seconds);
}
