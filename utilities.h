#pragma once

#include <array> // std::array
#include <chrono>
#include <functional>
#include <string>
#include <string_view>
#include <utility> // std::index_sequence

#include "fpm_adapter.hpp"

// Clocks and clock management

template <typename Rep, typename Period>
class DynamicTimeInterval;

using DefaultClock = std::chrono::high_resolution_clock;
using TimePoint = DefaultClock::time_point;
using Duration = DefaultClock::duration;

using TimeInterval = DynamicTimeInterval<uint64_t, std::ratio<1>>;

template <typename T>
concept ChronoDuration = requires(T obj) {
	requires std::same_as<DynamicTimeInterval<typename T::rep, typename T::period>, T>;
};

template <typename T>
concept ChronoPoint = std::is_base_of_v<std::chrono::time_point<typename T::Clock, typename T::Duration>, T>;

template <class C>
	requires requires(C obj) {
		{ obj() } -> std::convertible_to<TimePoint>;
	}
class Clock_t {
private:
	inline static C s_clock = DefaultClock::now;

public:
	static void setClock(C clock) { s_clock = clock; }

	static C getClock() { return s_clock; }

	static TimePoint now() { return s_clock(); }
};

using Clock = Clock_t<std::function<TimePoint(void)>>;

/**
 * @brief A flexible time interval class that wraps std::chrono::duration.
 *
 * This class provides a more dynamic way to handle time intervals, allowing for
 * easier conversions and manipulations.
 *
 * @tparam Rep The representation type for the duration (e.g., int, double).
 * @tparam Period The period of the duration (e.g., std::milli, std::ratio<1>).
 */
template <typename Rep, typename Period = std::ratio<1>>
class DynamicTimeInterval {
public:
	using DurationType = std::chrono::duration<Rep, Period>;
	using rep = DurationType::rep;
	using period = DurationType::period;

	template <typename ORep, typename OPeriod>
	friend class DynamicTimeInterval;

	DynamicTimeInterval(const DynamicTimeInterval& other) = default;

	/**
	 * @brief Constructs a DynamicTimeInterval from a std::chrono::duration.
	 */
	constexpr DynamicTimeInterval(DurationType d): m_duration(d) {}

	/**
	 * @brief Constructs a DynamicTimeInterval from a raw count of ticks.
	 */
	constexpr DynamicTimeInterval(Rep count): m_duration(count) {}

	template <typename Orep, typename Operiod>
	constexpr DynamicTimeInterval(std::chrono::duration<Orep, Operiod> d):
		m_duration(std::chrono::duration_cast<DurationType>(d)) {}

	/**
	 * @brief Gets the underlying std::chrono::duration object.
	 */
	constexpr DurationType get_duration() const { return m_duration; }

	/**
	 * @brief Gets the count of ticks for this interval.
	 */
	constexpr Rep count() const { return m_duration.count(); }

	/**
	 * @brief Gets the duration in microseconds.
	 */
	constexpr uint64_t microseconds() const {
		return std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(m_duration).count();
	}

	/**
	 * @brief Converts the interval to another representation and period.
	 */
	template <typename TargetRep, typename TargetPeriod>
	constexpr DynamicTimeInterval<TargetRep, TargetPeriod> as() const {
		return DynamicTimeInterval<TargetRep, TargetPeriod>(
			std::chrono::duration_cast<std::chrono::duration<TargetRep, TargetPeriod>>(m_duration)
		);
	}

	template <typename T, typename P>
	constexpr DynamicTimeInterval operator+(const DynamicTimeInterval<T, P>& other) {
		return DynamicTimeInterval(
			m_duration + std::chrono::duration_cast<std::chrono::duration<Rep, Period>>(other.m_duration)
		);
	}

	template <typename T, typename P>
	constexpr bool operator>(const DynamicTimeInterval<T, P>& other) {
		return m_duration > other.m_duration;
	}

private:
	const DurationType m_duration;
};

/**
 * @brief Creates a time interval in milliseconds with an optional offset.
 */
template <typename T, typename P>
constexpr DynamicTimeInterval<uint64_t, std::milli>
milliseconds(const uint64_t millis, const DynamicTimeInterval<T, P> offset) {
	return DynamicTimeInterval<uint64_t, std::milli>(millis) +
		DynamicTimeInterval<uint64_t, std::milli>(offset.get_duration());
}

/**
 * @brief Creates a time interval in milliseconds.
 */
constexpr DynamicTimeInterval<uint64_t, std::milli> milliseconds(const uint64_t millis) {
	return std::chrono::duration<uint64_t, std::milli>(millis);
}

/**
 * @brief Creates a time interval in microseconds.
 */
constexpr DynamicTimeInterval<uint64_t, std::micro> microseconds(const uint64_t micros) {
	return std::chrono::duration<uint64_t, std::micro>(micros);
}

/**
 * @brief Creates a time interval in seconds with an optional offset.
 */
template <typename T, typename P>
constexpr DynamicTimeInterval<uint64_t> seconds(uint64_t seconds, DynamicTimeInterval<T, P> offset) {
	return DynamicTimeInterval<uint64_t>(seconds) + DynamicTimeInterval<uint64_t>(offset.get_duration());
}

/**
 * @brief Creates a time interval in seconds.
 */
constexpr DynamicTimeInterval<uint64_t> seconds(uint64_t seconds) {
	return std::chrono::duration<uint64_t, std::ratio<1>>(seconds);
}

/**
 * @brief Gets the number of std::chrono Units since the Unix epoch.
 */
template <typename T>
inline T unitSinceEpoch() {
	auto now = std::chrono::time_point_cast<T>(Clock::now());
	auto duration_since_epoch = now.time_since_epoch();
	return std::chrono::duration_cast<T>(duration_since_epoch);
}

/**
 * @brief Gets the number of milliseconds since the Unix epoch.
 */
inline uint64_t milliSinceEpoch() {
	auto duration = unitSinceEpoch<std::chrono::milliseconds>();
	return static_cast<uint64_t>(duration.count());
}

/**
 * @brief Gets the number of microseconds since the Unix epoch.
 */
inline uint64_t microSinceEpoch() {
	auto duration = unitSinceEpoch<std::chrono::microseconds>();
	return static_cast<uint64_t>(duration.count());
}

/**
 * @brief Gets the number of nanoseconds since the Unix epoch.
 */
inline uint64_t nanoSinceEpoch() {
	auto duration = unitSinceEpoch<std::chrono::nanoseconds>();
	return static_cast<uint64_t>(duration.count());
}

// System-wide constants
const int                    motorInterfaceType = 1;                     ///< Stepper motor driver interface type.
const int                    maxSpeed = 1000;                            ///< Maximum speed for the motors.
const int                    acceleration = 3000;                        ///< Acceleration for the motors.
constexpr static fixed_16_16 rad2DegFactor = fixed_16_16(57.2957795131); ///< Conversion factor from radians to degrees.
constexpr static fixed_16_16 deg2RadFactor = fixed_16_16(0.01745329251); ///< Conversion factor from degrees to radians.
const fixed_16_16            Gz = -9.80665;                              ///< Acceleration due to gravity.
const int                    altitude = 1320;                            ///< Default altitude for calculations.
const fixed_16_16            projectileSpeed = 20;                       ///< Speed of the projectile.

/// @brief The interval after which a fire action can be repeated.
const auto fireActionInterval = seconds(3);

template <typename T>
consteval std::string_view type_name() {
	using namespace std::literals; // required for ""sv
// Format is usually: "constexpr std::string_view type_name() [with T = Name]"
// We need to slice it. This logic is compiler specific.
#if defined(__clang__) || defined(__GNUC__) || defined(_MSC_VER)
	#if defined(__clang__) || defined(__GNUC__)

	std::string_view name = __PRETTY_FUNCTION__;
	std::string_view prefix = "T = ";
	std::string_view suffix = "; ";

	#elif defined(_MSC_VER)

	std::string_view prefix = "type_name<";
	std::string_view suffix = ", ";
	std::string_view name = __FUNCSIG__;

	#endif

	auto start = name.find(prefix);
	auto end = name.find(suffix, start);
	if (end == std::string_view::npos) {
		end = name.rfind(">(void)"sv, start);
	}
	if (start != std::string_view::npos && end != std::string_view::npos) {
		start += prefix.size();
		return name.substr(start, end - start);
	}
	return name;

#else
	return "UnknownType";
#endif
}