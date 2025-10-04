#pragma once

#include <functional>
#include <stdint.h>
#include <cmath> // For std::signbit
#include "fpm_adapter.hpp"

using fixed = fixed_16_16;

/**
 * @brief Provides functions for numerical approximations.
 *
 * This namespace contains a collection of mathematical functions that use
 * approximation algorithms to compute their results, suitable for fixed-point arithmetic.
 */
namespace Approximate
{
	/**
	 * @brief Represents the result of an approximation calculation.
	 * @tparam T The type of the result.
	 */
	template <typename T>
	struct ApproximateResult
	{
		bool converged = false; ///< Indicates whether the approximation converged to a solution.
		T result;               ///< The result of the approximation.
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
	constexpr ApproximateResult<T> small_root(const std::function<const T(const T)> func, const T error = T(0.001), const uint8_t rounds = 32)
	{
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

		auto abs_val = [](auto val) {
			if constexpr (fpm::is_fixed<T>::value) {
				return fpm::abs(val);
			} else {
				return std::abs(val);
			}
		};

		// Find an interval containing the first root by expanding the search window.
		while ((sign_bit(leftValue) == sign_bit(rightValue)) && (round < rounds))
		{
			leftInput = rightInput;
			leftValue = rightValue;
			rightInput *= 4;
			rightValue = func(rightInput);
			round++;
		}

		round = 0; // Reset round counter for the refinement loop
		do
		{
			// Use the bisection method to find the midpoint of the interval.
			midInput = leftInput + (rightInput - leftInput) / 2;

			// If the midpoint is the same as an endpoint, we've reached the precision limit.
			if (midInput == leftInput || midInput == rightInput)
			{
				return ApproximateResult<T>(true, midInput);
			}

			midValue = func(midInput);

			// Narrow the search interval based on the sign of the function value.
			if (sign_bit(leftValue) == sign_bit(midValue))
			{
				leftInput = midInput;
				leftValue = midValue;
			}
			else
			{
				rightInput = midInput;
				rightValue = midValue;
			}

			// Check for convergence based on the absolute width of the interval.
			if (abs_val(rightInput - leftInput) <= error)
			{
				return ApproximateResult<T>(true, midInput);
			}
			// TODO: Add a check for convergence rate to exit early if progress stalls.

		} while (round++ < rounds);
		return ApproximateResult<T>(false, midInput);
	}
}
