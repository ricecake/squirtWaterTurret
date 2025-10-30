#pragma once

#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>

#include "utilities.h" // Includes TimePoint, DefaultClock, etc.

using Microseconds = std::chrono::microseconds;
using Milliseconds = std::chrono::milliseconds;
using Seconds = std::chrono::seconds;
using Minutes = std::chrono::minutes;

/**
 * @brief A mock clock for testing that can be advanced manually or automatically.
 *
 * By default, it increments time by a random microsecond amount on each call to
 * simulate real-world timing jitter. This can be disabled for deterministic tests.
 */
struct TestClock {
private:
	inline const static TimePoint                         start = TimePoint(Duration(0));
	inline static TimePoint                               m_current_time = start;
	inline static std::mt19937                            m_engine{std::random_device{}()};
	inline static std::uniform_int_distribution<long int> m_distribution{1, 50};
	inline static bool                                    s_advance_randomly = true;

public:
	TestClock() { Clock::setClock(*this); }

	/**
	 * @brief Resets the clock to the epoch.
	 */
	void reset() {
		m_current_time = start;
		s_advance_randomly = true;
	}

	void set(uint64_t us) { m_current_time = TimePoint(Microseconds(us)); }

	/**
	 * @brief A RAII helper to disable random clock advancement for a scope.
	 */
	struct ScopedDeterministicClock {
		ScopedDeterministicClock() { s_advance_randomly = false; }
		~ScopedDeterministicClock() { s_advance_randomly = true; }
	};

	/**
	 * @brief The callable operator that returns the current time and advances it.
	 * @return The TimePoint of the current mock time.
	 */
	TimePoint operator()() {
		// Capture the time point *before* advancing
		const TimePoint time_to_return = m_current_time;
		if (s_advance_randomly) {
			const long delay_us = m_distribution(m_engine);
			m_current_time += Microseconds(delay_us);
		}
		return std::chrono::time_point_cast<Microseconds>(time_to_return);
	}

	TestClock& operator+=(const Duration& o) {
		m_current_time += o;
		return *this;
	}
};
