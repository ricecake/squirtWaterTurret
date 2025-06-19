#include <sys/_types.h>
#include <functional>
#include <stdint.h>
#include "fpm/math.hpp"

using fixed = fpm::fixed_16_16;

namespace Approximate
{

	fixed sin(fixed);
	fixed cos(fixed);
	fixed tan(fixed);
	fixed atan(fixed);
	fixed sqrt(fixed);

	template <typename T>
	bool small_root(T &result, std::function<T(T)> func, T error = T(0.001), uint8_t rounds = 16)
	{
		T leftInput = 0;
		T rightInput = 1;
		T midInput;

		T leftValue = func(leftInput);
		T rightValue = func(rightInput);
		T midValue;

		uint8_t round = 0;

		// Find the interval containing first root
		while ((fpm::signbit(leftValue) == fpm::signbit(rightValue)) && (round < rounds))
		{
			leftInput = rightInput;
			leftValue = rightValue;
			rightInput *= 2;
			rightValue = func(rightInput);
			round++;
		}

		do
		{
			midInput = rightInput - rightValue * ((rightInput - leftInput) / (rightValue - leftValue));
			if (midInput <= leftInput || midInput >= rightInput)
			{
				midInput = leftInput + (rightInput - leftInput) / 2;
			}
			midValue = func(midInput);

			if (fpm::signbit(leftValue) == fpm::signbit(midValue))
			{
				leftInput = midInput;
				leftValue = midValue;
			}
			else
			{
				rightInput = midInput;
				rightValue = midValue;
			}

			// This should check if proportional error is less than the threshold
			if ((rightInput - leftInput) / rightInput < error)
			{
				result = midInput;
				return true;
			}
			// need a block to see if the convergence rate has dropped below some threshold, and return if we're not making progress, but we're close enough.

		} while (round++ < rounds);
		result = midInput;
		return false;
	}
}
