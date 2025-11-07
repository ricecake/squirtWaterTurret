#include "command_queue.h"

#include <iostream>
#include <sstream>

#include "state.h"
#include "utilities.h"

CommandQueue::CommandQueue() = default;

void CommandQueue::process(SystemState* state) {
	std::vector<std::shared_ptr<Command>> runnable_commands;
	{
		std::lock_guard<std::mutex> lock(xMutex);
		uint64_t now = microSinceEpoch();
		while (!commandQueue.empty()) {
			// Defensively check for null pointers, which seem to be the cause
			// of the crash. We don't know why they are in the queue, but we
			// can at least handle them gracefully.
			if (!commandQueue.top()) {
				commandQueue.pop();
				continue;
			}

			// If the command is ready to run, move it to a temporary vector.
			if (now >= commandQueue.top()->run_after) {
				runnable_commands.push_back(commandQueue.top());
				commandQueue.pop();
			} else {
				// The queue is sorted by time, so we can stop here.
				break;
			}
		}
	}

	// Execute the commands outside of the mutex lock to allow for re-entrant
	// command scheduling.
	for (const auto& comm : runnable_commands) {
		if (comm) {
			comm->Execute(state);
		}
	}
}

std::string CommandQueue::serialize() {
	std::stringstream ss;
	{
		std::lock_guard<std::mutex> lock(xMutex);
		auto tempQueue = commandQueue;

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
