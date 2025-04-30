#pragma once

#include <functional>
#include <stdint.h>
#include <queue>

// Define pin connections
const int stepPinB = 32;
const int dirPinB = 33;
const int stepPinA = 25;
const int dirPinA = 26;

// Define motor limits
const int maxSpeed = 400;
const int acceleration = 120;

// Define other constants
const int stepFraction = 16; // The microstep fraction

const int h_max = 500;
const int v_max = 1000;
const int h_min = -500;
const int v_min = -1000;

const float angleToStep = 0.1125; //(360 / 200) / 1 / 16; // circle / steps per circle / gear ratio / step division

// Define motor interface type
#define motorInterfaceType 1

class SystemState;

class Target {
	uint8_t index;
	long X_coord = 0;
	long Y_coord = 0;
	long distance = 0;
	long pitch = 0;
	long yaw = 0;
	bool valid = false;
	long speed = 0;
};

class Command
{
public:
	uint32_t id = 0;
	int64_t run_after = 0;
	bool operator<(const Command &other) const;

	virtual void Execute(SystemState &state) = 0;
	Command(int64_t run_after);
};

class MoveCommand : Command
{
	long pitch = 0;
	long yaw = 0;
	int speed = maxSpeed;
	void Execute(SystemState &state);
	MoveCommand(long, long, int, int64_t);

	// This should actually just be speed and target index.
	// The radar will set the targets as it finds them, and then we schedule which one we're interested in.
};

class SystemState
{
private:
	// 	motor a
	// 	motor b
	// motor collector;
	bool moveState = true;
	bool fireState = false;
	uint32_t lastMoveId = 0;
	Target target[3];
	uint8_t selectedTarget = 0;
	std::priority_queue<Command, std::vector<Command>, std::greater<Command>> commandQueue;

public:
	void queueFire(uint8_t milliseconds);
	void queueLinger(uint8_t milliseconds);
};

struct MoveCmd
{
	int H;
	int V;
	int S;
	int D;
	MoveCmd(int h = 0, int v = 0, int s = maxSpeed, int d = 50)
		: H(h), V(v), S(s), D(d) {}
};
