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

template <typename Rep, typename Period>
class DynamicTimeInterval;

template <typename T>
concept ChronoDuration = requires(T obj) {
	// typename TimeInterval<typename T::rep, typename T::period>;
	// std::is_base_of_v<TimeInterval<typename T::rep, typename T::period>, T>;
	requires std::same_as<DynamicTimeInterval<typename T::rep, typename T::period>, T>;
};

template <typename T>
concept ChronoPoint = std::is_base_of_v<std::chrono::time_point<typename T::Clock, typename T::Duration>, T>;

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

	template <typename ORep, typename OPeriod>
	friend class DynamicTimeInterval;

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

	constexpr uint64_t microseconds() const
	{
		return std::chrono::duration_cast<
				   std::chrono::duration<uint64_t, std::micro>>(m_duration)
			.count();
	}

	// Example of a conversion utility
	template <typename TargetRep, typename TargetPeriod>
	constexpr DynamicTimeInterval<TargetRep, TargetPeriod> as() const
	{
		return DynamicTimeInterval<TargetRep, TargetPeriod>(
			std::chrono::duration_cast<std::chrono::duration<TargetRep, TargetPeriod>>(m_duration));
	}

	template <typename T, typename P>
	constexpr DynamicTimeInterval operator+(const DynamicTimeInterval<T, P> &other)
	{
		return DynamicTimeInterval(m_duration + std::chrono::duration_cast<std::chrono::duration<Rep, Period>>(other.m_duration));
	}

	template <typename T, typename P>
	constexpr bool operator>(const DynamicTimeInterval<T, P> &other)
	{
		return m_duration > other.m_duration;
	}

	// constexpr bool operator>(const ChronoDuration auto &other) {
	// 	return m_duration > other.get_duration();
	// }

private:
	const DurationType m_duration;
};

using TimeInterval = DynamicTimeInterval<uint64_t, std::ratio<1>>;

constexpr auto AsSeconds = [](const ChronoDuration auto &d)
{
	return TimeInterval(d.get_duration());
	// DynamicTimeInterval<fixed_16_16, std::ratio<1>>(std::chrono::duration_cast<std::chrono::duration<fixed_16_16, std::ratio<1>> >(d.get_duration()));
	// return d.as<fixed_16_16, std::ratio<1>>();

	// return std::chrono::duration_cast<std::chrono::duration<fixed_16_16>>(d);
};

// Define motor interface type
const int motorInterfaceType = 1;
const int maxSpeed = 1000;	   // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
const int acceleration = 3000; // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
constexpr static fixed_16_16 rad2DegFactor = fixed_16_16(57.2957795131);
const fixed_16_16 Gz = -9.80665;
const int altitude = 1320;
const fixed_16_16 projectileSpeed = 20;

template <typename T, typename P>
constexpr DynamicTimeInterval<uint64_t, std::milli> milliseconds(const uint64_t millis, const DynamicTimeInterval<T, P> offset)
{
	return DynamicTimeInterval<uint64_t, std::milli>(millis) + DynamicTimeInterval<uint64_t, std::milli>(offset.get_duration());
}

constexpr DynamicTimeInterval<uint64_t, std::milli> milliseconds(const uint64_t millis)
{
	return std::chrono::duration<uint64_t, std::milli>(millis);
}

template <typename T, typename P>
constexpr DynamicTimeInterval<uint64_t> seconds(uint64_t seconds, DynamicTimeInterval<T, P> offset)
{
	return DynamicTimeInterval<uint64_t>(seconds) + DynamicTimeInterval<uint64_t>(offset.get_duration());
}

constexpr DynamicTimeInterval<uint64_t> seconds(uint64_t seconds)
{
	return std::chrono::duration<uint64_t, std::ratio<1>>(seconds);
}

const auto fireActionInterval = seconds(3);

inline uint64_t milliSinceEpoch() {
	auto now = std::chrono::system_clock::now();
	auto duration_since_epoch = now.time_since_epoch();
	return std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_epoch).count();
}