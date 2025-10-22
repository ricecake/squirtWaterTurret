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

class SystemState;
class Command;
const inline auto CommandPointerComparator = [](const auto& left, const auto& right) {
	return left->run_after >= right->run_after;
};

class CommandQueue {
public:
	CommandQueue();
	void process(SystemState* state);

	template<typename T, typename... Args>
	void addCommand(Args&&... args) {
		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
			commandQueue.push(new T(std::forward<Args>(args)...));
			xSemaphoreGive(xMutex);
		}
	}

	template<typename T, typename... Args>
	void addCommandAfter(Args&&... args) {
		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
			int64_t last_run_after = esp_timer_get_time();
			if (!commandQueue.empty()) {
				auto tempQueue = commandQueue;
				Command* cmd = nullptr;
				while (!tempQueue.empty()) {
					cmd = tempQueue.top();
					tempQueue.pop();
				}
				last_run_after = cmd->run_after;
			}

			commandQueue.push(new T(std::forward<Args>(args)..., last_run_after));
			xSemaphoreGive(xMutex);
		}
	}

	template<typename T, typename... Args>
	void runCommandIn(int64_t duration, Args&&... args) {
		int64_t run_after = esp_timer_get_time() + duration;
		addCommand<T>(std::forward<Args>(args)..., run_after);
	}

	std::string serialize() const;

private:
	std::priority_queue<Command*, std::vector<Command*>, decltype(CommandPointerComparator)> commandQueue;
	SemaphoreHandle_t                                                                      xMutex;
};
