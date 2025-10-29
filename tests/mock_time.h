#pragma once

#include <algorithm>
#include <chrono>
#include <random>

#include "utilities.h" // Includes TimePoint, DefaultClock, etc.

/**
 * @brief A mock clock for testing that increments time by a random microsecond
 * amount on each call.
 * * This simulates real-world timing jitter and allows controlled testing of
 * time-dependent logic.
 */
struct TestClock {
private:
	TimePoint                               m_current_time;
	std::mt19937                            m_engine;
	std::uniform_int_distribution<long int> m_distribution{100, 1000};

public:
	TestClock(): m_current_time(DefaultClock::now()), m_engine(std::random_device{}()) {}

	/**
	 * @brief Resets the clock to the real-world current time.
	 */
	void reset() { m_current_time = DefaultClock::now(); }

	/**
	 * @brief The callable operator that returns the current time and advances it.
	 * @return The TimePoint of the current mock time.
	 */
	TimePoint operator()() {
		// Capture the time point *before* advancing
		const TimePoint time_to_return = m_current_time;

		const long delay_us = m_distribution(m_engine);

		using Microseconds = std::chrono::microseconds;
		m_current_time += Microseconds(delay_us);
		return time_to_return;
	}

	template <typename T, typename P>
	TestClock& operator+=(const DynamicTimeInterval<T, P>& o) {
		m_current_time += o.get_duration();
		return *this;
	}
};

/**
 * @brief Example usage of the TestClock to switch the global clock for testing.
 */
// void run_tests() {
//     // 1. Create the mock clock instance
//     TestClock mock_clock;

//     // 2. Wrap the callable operator in a std::function
//     // Note: std::bind is used here because TestClock::operator() is const
//     // and we need to capture the state of the specific 'mock_clock' instance.
//     std::function<TimePoint(void)> mock_func = std::bind(&TestClock::operator(), &mock_clock);

//     // 3. Set the global clock (assuming Clock::setClock is available)
//     // Clock::setClock(mock_func);

//     // RUN YOUR TESTS HERE
//     // ...

//     // Check the time jumps:
//     // TimePoint t1 = Clock::now();
//     // TimePoint t2 = Clock::now();
//     // assert(t2 > t1);
// }
