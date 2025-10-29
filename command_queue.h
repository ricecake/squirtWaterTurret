#pragma once

#include <queue>
#include <string>
#include <vector>

#ifdef ARDUINO
	#include "freertos/FreeRTOS.h"
	#include "freertos/semphr.h"
#else
	#include "tests/mocks.h"
#endif

#include "command.h"
#include "utilities.h"

class SystemState;
class Command;
const inline auto CommandPointerComparator = [](const auto& left, const auto& right) {
	return left->run_after >= right->run_after;
};

class CommandQueue {
public:
	CommandQueue();
	void process(SystemState* state);

	template <typename T, typename... Args>
	void addCommand(Args&&... args) {
		auto newCommand = new T(std::forward<Args>(args)...);
		max_run_after = std::max(max_run_after, newCommand->run_after);
		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
			commandQueue.push(newCommand);
			xSemaphoreGive(xMutex);
		}
	}

	template <typename T, typename... Args>
	void addCommandAfter(Args&&... args) {
		auto last_run_after = max_run_after + 1;
		commandQueue.push(new T(std::forward<Args>(args)..., last_run_after));
	}

	template <typename T, typename... Args>
	void runCommandIn(int64_t duration, Args&&... args) {
		int64_t run_after = microSinceEpoch() + duration;
		addCommand<T>(std::forward<Args>(args)..., run_after);
	}

	std::string serialize() const;

private:
	uint64_t                                                                                 max_run_after = 0;
	std::priority_queue<Command*, std::vector<Command*>, decltype(CommandPointerComparator)> commandQueue;
	SemaphoreHandle_t                                                                        xMutex;
};
