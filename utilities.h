#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define motor interface type
const int motorInterfaceType = 1;
const int maxSpeed = 1000;	   // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
const int acceleration = 3000; // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.


template <typename T>
int64_t milliseconds(T millis, int64_t offset = 0)
{
	return offset + int64_t(1000 * millis);
}

template <typename T>
int64_t seconds(T seconds, int64_t offset = 0)
{
	return offset + int64_t(1000 * 1000 * seconds);
}
