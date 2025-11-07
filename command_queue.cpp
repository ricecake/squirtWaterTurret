#include "command_queue.h"

#include <iostream>
#include <sstream>

#include "state.h"
#include "utilities.h"

CommandQueue::CommandQueue() {
	xMutex = xSemaphoreCreateMutex();
}

void CommandQueue::process(SystemState* state) {
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		uint64_t now = microSinceEpoch();
		while (!commandQueue.empty()) {
			auto comm = commandQueue.top();
			if (comm) {
				if (now < comm->run_after) {
					break;
				}
				comm->Execute(state);
			}
			commandQueue.pop();
		}
		xSemaphoreGive(xMutex);
	}
}

std::string CommandQueue::serialize() const {
	std::stringstream ss;
	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
		auto tempQueue = commandQueue;
		xSemaphoreGive(xMutex);

		auto now = microSinceEpoch();
		ss << "Time: " << now << std::endl;
		bool flagged = false;
		while (!tempQueue.empty()) {
			auto comm = tempQueue.top();
			if (comm) {
				if (now < comm->run_after && !flagged) {
					ss << "===== WOULD STOP HERE =========" << std::endl;
					flagged = true;
				}

				ss << "Command ID: " << comm->id << " Run After: " << comm->run_after << "\n";
			}
			tempQueue.pop();
		}
	}
	return ss.str();
}
