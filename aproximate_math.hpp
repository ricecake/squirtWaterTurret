#pragma once

#include <cmath> // For std::signbit
#include <functional>
#include <vector>

#include "fpm_adapter.hpp"
#include <stdint.h>

using fixed = fixed_16_16;

/**
 * @brief Provides functions for numerical approximations.
 *
 * This namespace contains a collection of mathematical functions that use
 * approximation algorithms to compute their results, suitable for fixed-point arithmetic.
 */
namespace Approximate {
	/**
	 * @brief Represents the result of an approximation calculation.
	 * @tparam T The type of the result.
	 */
	template <typename T>
	struct ApproximateResult {
		bool converged = false; ///< Indicates whether the approximation converged to a solution.
		T    result;            ///< The result of the approximation.
	};

	fixed sin(fixed);
	fixed cos(fixed);
	fixed tan(fixed);
	fixed atan(fixed);
	fixed sqrt(fixed);

	/**
	 * @brief Finds a small root of a function using a combination of the secant and bisection methods.
	 *
	 * This function attempts to find a root of the given function `func` within a specified number
	 * of iterations. It starts by searching for an interval where a root is likely to exist and
	 * then refines the search using a numerical method.
	 *
	 * @tparam T The numeric type for the calculation.
	 * @param func The function for which to find a root.
	 * @param error The desired proportional error for convergence.
	 * @param rounds The maximum number of iterations to perform.
	 * @return An ApproximateResult containing the outcome of the root-finding process.
	 */
	template <typename T>
	constexpr ApproximateResult<T>
	small_root(const std::function<T(const T)> func, const T error = T(0.001), const uint8_t rounds = 16) {
		T leftInput = 0;
		T rightInput = 0.01;
		T midInput;

		T leftValue = func(leftInput);
		T rightValue = func(rightInput);
		T midValue;

		uint8_t round = 0;

		auto sign_bit = [](auto val) {
			if constexpr (fpm::is_fixed<T>::value) {
				return fpm::signbit(val);
			} else {
				return std::signbit(val);
			}
		};

		// Find an interval containing the first root by expanding the search window.
		while ((sign_bit(leftValue) == sign_bit(rightValue)) && (round < rounds)) {
			leftInput = rightInput;
			leftValue = rightValue;
			rightInput *= 4;
			rightValue = func(rightInput);
			// round++; -- TODO: evaluate the impact of this check.
		}

		round = 0; // Reset round counter for the refinement loop
		do {
			// Secant method - zero of secant of function at best guess
			midInput = rightInput - rightValue * ((rightInput - leftInput) / (rightValue - leftValue));
			// If secant intersects outside our boundary, pick next best guess to be midpoint between edges instead.
			if (midInput <= leftInput || midInput >= rightInput) {
				midInput = leftInput + (rightInput - leftInput) / 2;
			}
			midValue = func(midInput);

			// Narrow the search interval based on the sign of the function value.
			if (sign_bit(leftValue) == sign_bit(midValue)) {
				leftInput = midInput;
				leftValue = midValue;
			} else {
				rightInput = midInput;
				rightValue = midValue;
			}

			// This should check if proportional error is less than the threshold
			if ((rightInput - leftInput) / rightInput <= error) {
				return ApproximateResult<T>(true, midInput);
			}
			// TODO: Add a check for convergence rate to exit early if progress stalls.

		} while (round++ < rounds);
		return ApproximateResult<T>(false, midInput);
	}

	/**
	 * @brief Finds N roots of a function using a combination of the secant and bisection methods.
	 *
	 * This function attempts to find N roots of the given function `func` within a specified
	 * number of iterations for each root. It repeatedly searches for roots, beginning each
	 * new search just after the previously found root.
	 *
	 * @tparam T The numeric type for the calculation.
	 * @param func The function for which to find roots.
	 * @param n_roots The number of roots to find.
	 * @param error The desired proportional error for convergence for each root.
	 * @brief Finds N smallest positive roots of a function using a combination of the secant and bisection methods.
	 *
	 * This function attempts to find N roots of the given function `func` within a specified
	 * number of iterations for each root. It repeatedly searches for roots, beginning each
	 * new search just after the previously found root. This implementation finds the N smallest
	 * positive roots.
	 *
	 * @tparam T The numeric type for the calculation.
	 * @param func The function for which to find roots.
	 * @param n_roots The number of roots to find.
	 * @param error The desired proportional error for convergence for each root.
	 * @param rounds The maximum number of iterations to perform for each root.
	 * @return An ApproximateResult containing a std::vector of the found roots.
	 */
	template <typename T>
	ApproximateResult<std::vector<T>> n_roots(
		const std::function<T(const T)> func,
		const uint8_t                         n_roots,
		const T                               error = T(0.001),
		const uint8_t                         rounds = 16
	) {
		std::vector<T> roots;
		T              last_root = 0;

		for (int i = 0; i < n_roots; i++) {
			// Start searching slightly after the last root to avoid finding it again.
			const T search_start = last_root + error;

			std::function<T(const T)> shifted_func = [&](const T x) { return func(x + search_start); };
			auto                            result = small_root(shifted_func, error, rounds);
			if (result.converged) {
				// The new root is relative to the search start.
				last_root = search_start + result.result;
				roots.push_back(last_root);
			} else {
				// If we can't find another root, we're done.
				return ApproximateResult<std::vector<T>>{false, roots};
			}
		}

		return ApproximateResult<std::vector<T>>{true, roots};
	}
} // namespace Approximate
