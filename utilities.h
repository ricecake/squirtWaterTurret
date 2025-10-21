/**
 * @file utilities.h
 * @brief Provides time-related utilities, constants, and a flexible time interval class.
 */
#pragma once

#include <chrono>

#include "fpm_adapter.hpp"

// using aliases for std::chrono types for convenience and consistency.
using Clock = std::chrono::high_resolution_clock; ///< The high-resolution clock used for timing throughout the system.
using TimePoint = Clock::time_point;              ///< A specific point in time, as measured by the system `Clock`.
using Duration = Clock::duration;                 ///< A time duration, as measured by the system `Clock`.

template <typename Rep, typename Period>
class DynamicTimeInterval;

/**
 * @brief A concept to identify types that behave like a `DynamicTimeInterval`.
 * This is used for template constraints to ensure type safety.
 */
template <typename T>
concept ChronoDuration = requires(T obj) {
	requires std::same_as<DynamicTimeInterval<typename T::rep, typename T::period>, T>;
};

/**
 * @brief A concept to identify types that behave like a `std::chrono::time_point`.
 */
template <typename T>
concept ChronoPoint = std::is_base_of_v<std::chrono::time_point<typename T::Clock, typename T::Duration>, T>;

/**
 * @brief A flexible time interval class that wraps `std::chrono::duration`.
 *
 * This class provides a more dynamic and user-friendly way to handle time intervals.
 * It simplifies creation, conversion, and arithmetic operations between different
 * time units (e.g., milliseconds, seconds) and representation types.
 *
 * @tparam Rep The underlying representation type for the duration's tick count (e.g., `uint64_t`, `double`).
 * @tparam Period The period of the duration, representing the time of one tick (e.g., `std::milli`, `std::ratio<1>`).
 */
template <typename Rep, typename Period = std::ratio<1>>
class DynamicTimeInterval {
public:
	using DurationType = std::chrono::duration<Rep, Period>;
	using rep = typename DurationType::rep;
	using period = typename DurationType::period;

	template <typename ORep, typename OPeriod>
	friend class DynamicTimeInterval;

	DynamicTimeInterval(const DynamicTimeInterval& other) = default;

	/**
	 * @brief Constructs a `DynamicTimeInterval` from a `std::chrono::duration`.
	 * @param d The `std::chrono::duration` to wrap.
	 */
	constexpr DynamicTimeInterval(DurationType d) : m_duration(d) {}

	/**
	 * @brief Constructs a `DynamicTimeInterval` from a raw count of ticks.
	 * @param count The number of ticks for this interval's period.
	 */
	constexpr DynamicTimeInterval(Rep count) : m_duration(count) {}

	/**
	 * @brief Constructs a `DynamicTimeInterval` by converting from another `std::chrono::duration`.
	 * @tparam Orep The representation type of the other duration.
	 * @tparam Operiod The period of the other duration.
	 * @param d The `std::chrono::duration` to convert from.
	 */
	template <typename Orep, typename Operiod>
	constexpr DynamicTimeInterval(std::chrono::duration<Orep, Operiod> d) :
		m_duration(std::chrono::duration_cast<DurationType>(d)) {}

	/**
	 * @brief Gets the underlying `std::chrono::duration` object.
	 * @return The wrapped `std::chrono::duration`.
	 */
	constexpr DurationType get_duration() const { return m_duration; }

	/**
	 * @brief Gets the count of ticks for this interval.
	 * @return The number of ticks.
	 */
	constexpr Rep count() const { return m_duration.count(); }

	/**
	 * @brief Gets the duration in microseconds.
	 * @return The total number of microseconds.
	 */
	constexpr uint64_t microseconds() const {
		return std::chrono::duration_cast<std::chrono::duration<uint64_t, std::micro>>(m_duration).count();
	}

	/**
	 * @brief Converts this interval to another `DynamicTimeInterval` with a different representation and period.
	 * @tparam TargetRep The target representation type.
	 * @tparam TargetPeriod The target period.
	 * @return A new `DynamicTimeInterval` with the converted value.
	 */
	template <typename TargetRep, typename TargetPeriod>
	constexpr DynamicTimeInterval<TargetRep, TargetPeriod> as() const {
		return DynamicTimeInterval<TargetRep, TargetPeriod>(
			std::chrono::duration_cast<std::chrono::duration<TargetRep, TargetPeriod>>(m_duration)
		);
	}

	/**
	 * @brief Adds two `DynamicTimeInterval` objects.
	 * @param other The interval to add.
	 * @return A new `DynamicTimeInterval` representing the sum.
	 */
	template <typename T, typename P>
	constexpr DynamicTimeInterval operator+(const DynamicTimeInterval<T, P>& other) {
		return DynamicTimeInterval(
			m_duration + std::chrono::duration_cast<std::chrono::duration<Rep, Period>>(other.m_duration)
		);
	}

	/**
	 * @brief Compares two `DynamicTimeInterval` objects.
	 * @param other The interval to compare against.
	 * @return `true` if this interval is greater than the other.
	 */
	template <typename T, typename P>
	constexpr bool operator>(const DynamicTimeInterval<T, P>& other) {
		return m_duration > other.m_duration;
	}

private:
	const DurationType m_duration;
};

/// @brief A type alias for a time interval with a `uint64_t` representation and a period of 1 second.
using TimeInterval = DynamicTimeInterval<uint64_t, std::ratio<1>>;

/**
 * DEPRECATE
 * This function is not used anywhere in the codebase.
 */
constexpr auto AsSeconds = [](const ChronoDuration auto& d) { return TimeInterval(d.get_duration()); };

// ======================================================================================
// --- System-wide constants ---
// ======================================================================================
const int                    motorInterfaceType = 1;                     ///< Stepper motor driver interface type (e.g., DRV8825).
const int                    maxSpeed = 1000;                            ///< Maximum speed for the motors in steps/second.
const int                    acceleration = 3000;                        ///< Acceleration for the motors in steps/second^2.
constexpr static fixed_16_16 rad2DegFactor = fixed_16_16(57.2957795131); ///< Conversion factor from radians to degrees.
const fixed_16_16            Gz = -9.80665;                              ///< Acceleration due to gravity in m/s^2.
const int                    altitude = 1320;                            ///< Default altitude of the system in meters.
const fixed_16_16            projectileSpeed = 20;                       ///< Initial speed of the projectile in m/s.

// ======================================================================================
// --- Time Interval Helper Functions ---
// ======================================================================================

/**
 * @brief Creates a time interval in milliseconds with an optional offset.
 * @param millis The number of milliseconds.
 * @param offset An existing `DynamicTimeInterval` to add to the new interval.
 * @return A `DynamicTimeInterval` in milliseconds.
 */
template <typename T, typename P>
constexpr DynamicTimeInterval<uint64_t, std::milli>
milliseconds(const uint64_t millis, const DynamicTimeInterval<T, P> offset) {
	return DynamicTimeInterval<uint64_t, std::milli>(millis) +
		DynamicTimeInterval<uint64_t, std::milli>(offset.get_duration());
}

/**
 * @brief Creates a time interval in milliseconds.
 * @param millis The number of milliseconds.
 * @return A `DynamicTimeInterval` in milliseconds.
 */
constexpr DynamicTimeInterval<uint64_t, std::milli> milliseconds(const uint64_t millis) {
	return std::chrono::duration<uint64_t, std::milli>(millis);
}

/**
 * @brief Creates a time interval in seconds with an optional offset.
 * @param seconds The number of seconds.
 * @param offset An existing `DynamicTimeInterval` to add to the new interval.
 * @return A `DynamicTimeInterval` in seconds.
 */
template <typename T, typename P>
constexpr DynamicTimeInterval<uint64_t> seconds(uint64_t seconds, DynamicTimeInterval<T, P> offset) {
	return DynamicTimeInterval<uint64_t>(seconds) + DynamicTimeInterval<uint64_t>(offset.get_duration());
}

/**
 * @brief Creates a time interval in seconds.
 * @param seconds The number of seconds.
 * @return A `DynamicTimeInterval` in seconds.
 */
constexpr DynamicTimeInterval<uint64_t> seconds(uint64_t seconds) {
	return std::chrono::duration<uint64_t, std::ratio<1>>(seconds);
}

/// @brief The minimum interval after which a fire action can be repeated on the same target.
const auto fireActionInterval = seconds(3);

/**
 * @brief Gets the number of milliseconds since the Unix epoch.
 * @return The current time as a `uint64_t` millisecond count.
 */
inline uint64_t milliSinceEpoch() {
	auto now = std::chrono::system_clock::now();
	auto duration_since_epoch = now.time_since_epoch();
	return std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_epoch).count();
}