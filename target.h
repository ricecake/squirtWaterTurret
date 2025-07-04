#pragma once

// #include <functional>
#include <stdint.h>
// #include <queue>
// #include <AccelStepper.h>
// #include <MultiStepper.h>
#include "vector.hpp"
#include "fpm_adapter.hpp"
#include <chrono>

using fixed = fixed_16_16;

// struct FixedReprHighResClock : public std::chrono::high_resolution_clock {
// 	using rep = fixed;
// 	using duration = std::chrono::duration<rep, std::nano>;
// 	using time_point = std::chrono::time_point<FixedReprHighResClock, duration>;
// };

using Clock = std::chrono::high_resolution_clock;
// using Clock = FixedReprHighResClock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

// class VDur : public std::chrono::duration<fixed>
// {
// 	public:
// 	VDur(fixed t): std::chrono::duration<fixed>(t){};
// };

template <typename Rep = fixed, typename Period = std::ratio<1>>
class TimeInterval
{
public:
	using DurationType = std::chrono::duration<Rep, Period>;

	// Constructor to initialize with a std::chrono::duration
	constexpr TimeInterval(DurationType d) : m_duration(d) {}

	// Constructor to initialize with a raw count
	constexpr TimeInterval(Rep count) : m_duration(count) {}

	template <typename Orep, typename Operiod>
	constexpr TimeInterval(std::chrono::duration<Orep, Operiod> d) : m_duration(std::chrono::duration_cast<DurationType>(d))
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

	// Example of a conversion utility
	template <typename TargetRep, typename TargetPeriod>
	constexpr TimeInterval<TargetRep, TargetPeriod> as() const
	{
		return TimeInterval<TargetRep, TargetPeriod>(
			std::chrono::duration_cast<std::chrono::duration<TargetRep, TargetPeriod>>(m_duration));
	}

private:
	const DurationType m_duration;
};

template <typename T>
concept ChronoDuration = requires(T obj) {
	typename TimeInterval<typename T::rep, typename T::period>;
	// std::is_base_of_v<TimeInterval<typename T::rep, typename T::period>, T>;
};

template <typename T>
concept ChronoPoint = std::is_base_of_v<std::chrono::time_point<typename T::Clock, typename T::Duration>, T>;

constexpr auto AsSeconds = [](const ChronoDuration auto &d)
{ return std::chrono::duration_cast<std::chrono::duration<fixed>>(d); };

class Target;
class PositionVector;
class DistanceVector;
class VelocityVector;

class DistanceVector : public Vector3D<fixed, DistanceVector>
{
	using Vec = Vector3D<fixed, DistanceVector>;

public:
	DistanceVector() = default;
	DistanceVector(const DistanceVector &other) = default;
	constexpr DistanceVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}
	DistanceVector(VelocityVector, ChronoDuration auto interval);
};

class PositionVector : public Vector3D<fixed, PositionVector>
{
	using Vec = Vector3D<fixed, PositionVector>;
	fixed _distance = 0;
	fixed _pitch = 0;
	fixed _yaw = 0;

public:
	PositionVector() = default;
	PositionVector(const PositionVector &other) = default;
	PositionVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}
	PositionVector(PositionVector, DistanceVector);
	PositionVector(PositionVector, VelocityVector, ChronoDuration auto interval);

public:
	fixed Pitch();
	fixed Yaw();
	fixed Distance();
};

class VelocityVector : public Vector3D<fixed, VelocityVector>
{
	using Vec = Vector3D<fixed, VelocityVector>;

public:
	VelocityVector() = default;
	VelocityVector(const VelocityVector &other) = default;
	VelocityVector(fixed x, fixed y, fixed z) : Vec(x, y, z) {}
	VelocityVector(DistanceVector, ChronoDuration auto interval);
	VelocityVector(PositionVector, PositionVector, ChronoDuration auto interval);

	// DistanceVector operator*(const ChronoDuration auto& interval);
};

class Target
{
public:
	Target() = default;
	Target(const Target &other) = default;
	Target(PositionVector P, VelocityVector V = VelocityVector(0, 0, 0));
	Target(uint8_t index, bool valid = false, PositionVector P = PositionVector(0, 0, 0), VelocityVector V = VelocityVector(0, 0, 0));

public:
	void Update(PositionVector P);

public:
	fixed Pitch();
	fixed Yaw();
	fixed Distance();

	const VelocityVector Velocity() const;
	const PositionVector Position() const;

	Duration timeSinceLastAction();
	bool actionIdleExceeds(ChronoDuration auto limit);
	void IncrementAction();
	PositionVector PredictedPositionAtTime(ChronoDuration auto interval);

public:
	bool valid = false;
	uint8_t index;
	TimePoint seen;
	TimePoint last_action;

private:
	TimePoint last_seen;
	PositionVector position;
	PositionVector last_position;
	VelocityVector velocity;
};


constexpr VelocityVector const operator/(const DistanceVector &, const ChronoDuration auto &interval);
constexpr const DistanceVector operator*(const VelocityVector &, const TimeInterval<> &interval);

constexpr PositionVector const operator+(const PositionVector &, const DistanceVector &);
constexpr DistanceVector const operator-(const PositionVector &, const PositionVector &);
