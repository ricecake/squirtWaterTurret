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

	std::string serialize() const;

private:
	std::priority_queue<Command*, std::vector<Command*>, decltype(CommandPointerComparator)> commandQueue;
	SemaphoreHandle_t                                                                      xMutex;
};
