#pragma once

#include <functional>
#include <stdint.h>
#include <queue>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "vector.hpp"
#include "command.h"
#include "target.h"
#include "fpm_adapter.hpp"

using fixed = fixed_16_16;

class Command;

class SystemState
{
private:
	const int motorInterfaceType = 1;

	// Define pin connections
	// const int stepPinB = 32;
	// const int dirPinB = 33;
	// const int stepPinA = 25;
	// const int dirPinA = 26;
	const int stepPinA = 32;
	const int dirPinA = 33;
	const int stepPinB = 25;
	const int dirPinB = 26;

	const int firePin = 2;

public:
	// Define motor limits
	// const int maxSpeed = maxSpeed; // This should be made more internal, and things should use proportional values.  Half speed, full speed, etc.
	// const int acceleration = 120;

	// Define other constants
	const int stepFraction = 16; // The microstep fraction

	const int h_max = 500;
	const int v_max = 1000;
	const int h_min = -500;
	const int v_min = -1000;

	const fixed angleToStep{0.1125}; //(360 / 200) / 1 / 16; // circle / steps per circle / gear ratio / step division

private:

public:
	AccelStepper stepperA;
	AccelStepper stepperB;
	MultiStepper steppers;

	SemaphoreHandle_t xMutex;

private:
	bool moveState = true;
	bool fireState = false;
	bool needTrackingUpdate = false;
	uint8_t trackingSpeed = 255;
	uint8_t selectedTarget = 0;

private:
	PositionVector targetAimpoint();
	std::array<Target, 32> target;
	std::array<Target, 3> radarTarget; // Separate the two -- by default populate the radar target and prefer the target list if possible
	std::priority_queue<Command *, std::vector<Command *>, decltype([](auto left, auto right)
																	{ return left->run_after >= right->run_after; })>
		commandQueue;

public:
	SystemState();
	Target &currentTarget();
	constexpr size_t size() {
		return target.size();
	}
	void updateTarget(const uint8_t idx, const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin = 0);
	inline void updateTargetById(const uint8_t id, const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin = 0) {
		auto pred = [&](Target& item) {
			return item.id == id;
		};
		auto found = std::ranges::find_if(target, pred);

		if (found == target.end()) {
			auto pred = [&](Target& item) {
				return item.valid == false;
			};
			found = std::ranges::find_if(target, pred);
		}

		if (found == target.end()) {
			auto pred = [&](Target& item) {
				return item.seen;
			};
			found = std::ranges::min_element(target, std::ranges::less{}, pred);
		}

		updateTarget(found->index, valid, newPosition, indifferenceMargin);
		target[found->index].id = id;
	};
	void updateNearestTarget(const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin = 0);
	void updateNearestTarget2d(const bool valid, PositionVector &newPosition, const uint16_t indifferenceMargin = 0);
	void setTarget(uint8_t index, uint8_t speed = 0xFF);
	void setFire(bool active);
	bool getFireState();
	void queueSelectTarget(uint8_t index, uint16_t milliseconds);
	void queueFire(uint16_t milliseconds);
	void queueLinger(uint8_t milliseconds);
	void processCommandQueue();
	void actualizeState();
	inline Target &fetchTarget(const uint8_t idx) {
		return target[idx];
	}
	inline uint8_t fetchNearestTargetIdx(const PositionVector& point) {
		auto distance = [&](Target& item){
			auto pos = item.Position();
			return pow(pos.X_coord - point.X_coord, 2)+pow(pos.Y_coord - point.Y_coord, 2)+pow(pos.Z_coord - point.Z_coord, 2);
		};
		auto res = std::ranges::min_element(target, std::ranges::less{}, distance);
		return std::ranges::distance(target.begin(), res);
	}

	inline uint8_t fetchNearestTarget2dIdx(const PositionVector& point) {
		auto distance = [&](Target& item){
			auto pos = item.Position();
			return pow(pos.X_coord - point.X_coord, 2)+pow(pos.Y_coord - point.Y_coord, 2);
		};
		auto res = std::ranges::min_element(target, std::ranges::less{}, distance);
		return std::ranges::distance(target.begin(), res);
	}

	fixed targetTravelDistance();

private:
	void actualizePosition();
	void actualizeFiring();
};
