#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include "command.h"
#include "utilities.h"
#ifndef ARDUINO
	#include "tests/mocks.h"
#endif

class SystemState;
class Command;
const inline auto CommandPointerComparator = [](const std::shared_ptr<Command>& left,
                                                const std::shared_ptr<Command>& right) -> bool {
	// A null command is "greater" than any valid command, so it will sink to the
	// bottom of the priority queue.
	if (!left) {
		return true;
	}
	if (!right) {
		return false;
	}
	return left->run_after > right->run_after;
};

class CommandQueue {
public:
	CommandQueue();
	void process(SystemState* state);

	template <typename T, typename... Args>
	void addCommand(Args&&... args) {
		auto newCommand = std::make_shared<T>(std::forward<Args>(args)...);
		max_run_after = std::max(max_run_after, newCommand->run_after);
		std::lock_guard<std::mutex> lock(xMutex);
		commandQueue.push(std::move(newCommand));
	}

	template <typename T, typename... Args>
	void addCommandAfter(Args&&... args) {
		auto run_time = max_run_after + 1;
		auto newCommand = std::make_shared<T>(std::forward<Args>(args)..., 0);
		newCommand->run_after = run_time;
		std::lock_guard<std::mutex> lock(xMutex);
		max_run_after = run_time;
		commandQueue.push(std::move(newCommand));
	}

	template <typename T, typename... Args>
	void runCommandIn(uint64_t duration, Args&&... args) {
		addCommand<T>(std::forward<Args>(args)..., duration);
	}

	std::string serialize();

private:
	uint64_t max_run_after = 0;
	std::priority_queue<
		std::shared_ptr<Command>,
		std::vector<std::shared_ptr<Command>>,
		decltype(CommandPointerComparator)>
					   commandQueue;
	mutable std::mutex xMutex;
};
