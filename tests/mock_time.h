#pragma once

#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>

#include "utilities.h" // Includes TimePoint, DefaultClock, etc.

using Microseconds = std::chrono::microseconds;
using Seconds = std::chrono::seconds;
using Minutes = std::chrono::minutes;

/**
 * @brief A mock clock for testing that increments time by a random microsecond
 * amount on each call.
 * * This simulates real-world timing jitter and allows controlled testing of
 * time-dependent logic.
 */
struct TestClock {
private:
	inline const static TimePoint                         start = TimePoint(Duration(0));
	inline static TimePoint                               m_current_time = start;
	inline static std::mt19937                            m_engine{std::random_device{}()};
	inline static std::uniform_int_distribution<long int> m_distribution{1, 50};

public:
	TestClock() { Clock::setClock(*this); }

	/**
	 * @brief Resets the clock to the real-world current time.
	 */
	void reset() { m_current_time = start; }

	void set(uint64_t us) { m_current_time = TimePoint(Microseconds(us)); }

	/**
	 * @brief The callable operator that returns the current time and advances it.
	 * @return The TimePoint of the current mock time.
	 */
	TimePoint operator()() {
		// Capture the time point *before* advancing
		const TimePoint time_to_return = m_current_time;

		const long delay_us = m_distribution(m_engine);
		m_current_time += Microseconds(delay_us);
		return std::chrono::time_point_cast<Microseconds>(time_to_return);
	}

	TestClock& operator+=(const Duration& o) {
		m_current_time += o;
		return *this;
	}
};
