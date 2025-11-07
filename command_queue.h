#pragma once

#include <memory>
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
const inline auto CommandPointerComparator = [](const std::shared_ptr<Command>& left,
                                                const std::shared_ptr<Command>& right) -> bool {
	if (left && right) {
		return left->run_after >= right->run_after;
	}
	return bool(left);
};

class CommandQueue {
public:
	CommandQueue();
	void process(SystemState* state);

	template <typename T, typename... Args>
	void addCommand(Args&&... args) {
		auto newCommand = std::make_shared<T>(std::forward<Args>(args)...);
		max_run_after = std::max(max_run_after, newCommand->run_after);
		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
			commandQueue.push(std::move(newCommand));
			xSemaphoreGive(xMutex);
		}
	}

	template <typename T, typename... Args>
	void addCommandAfter(Args&&... args) {
		auto now = microSinceEpoch();
		auto last_run_after = (max_run_after - now) + 1;
		auto newCommand = std::make_shared<T>(std::forward<Args>(args)..., last_run_after);
		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
			commandQueue.push(std::move(newCommand));
			xSemaphoreGive(xMutex);
		}
	}

	template <typename T, typename... Args>
	void runCommandIn(uint64_t duration, Args&&... args) {
		addCommand<T>(std::forward<Args>(args)..., duration);
	}

	std::string serialize() const;

private:
	uint64_t max_run_after = 0;
	std::priority_queue<
		std::shared_ptr<Command>,
		std::vector<std::shared_ptr<Command>>,
		decltype(CommandPointerComparator)>
					  commandQueue;
	SemaphoreHandle_t xMutex;
};
