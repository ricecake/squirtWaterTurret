#include "command_queue.h"
#include "state.h"
#include <sstream>

CommandQueue::CommandQueue() {
	xMutex = xSemaphoreCreateMutex();
}

void CommandQueue::process(SystemState* state) {
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		auto now = esp_timer_get_time();
		while (!commandQueue.empty()) {
			auto comm = commandQueue.top();
			if (now < comm->run_after) {
				break;
			}
			commandQueue.pop();
			comm->Execute(state);
			delete comm;
		}
		xSemaphoreGive(xMutex);
	}
}

std::string CommandQueue::serialize() const {
	std::stringstream ss;
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		auto tempQueue = commandQueue;
		xSemaphoreGive(xMutex);
		while (!tempQueue.empty()) {
			auto comm = tempQueue.top();
			ss << "Command ID: " << comm->id << " Run After: " << comm->run_after << "\n";
			tempQueue.pop();
		}
	}
	return ss.str();
}
