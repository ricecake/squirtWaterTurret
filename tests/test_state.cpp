#include "state.h"
#include "doctest/doctest.h"
#include "command.h"
#include "utilities.h"

static TimePoint mock_time;
TimePoint        getMockTime() {
	return mock_time;
}

// Mock command for testing state changes
class StateUpdateCommand : public Command {
public:
	using Command::Command;

	void execute(SystemState& state) override {
		state.test_value = 1;
	}
};


TEST_CASE("SystemState class tests") {
	SystemState state;
	Clock::setClock(getMockTime);
	mock_time = TimePoint(seconds(0));

	SUBCASE("setTarget selects the correct target") {
		state.cvTarget[5].Update(PositionVector(100, 200, 300));

		// Select a CV target
		state.setTarget(cerializer::TargetSource::CV, 5);
		CHECK(state.currentTarget().Position().X_coord == 100);

		// Select the static target
		state.setTarget(cerializer::TargetSource::STATIC, 0);
		CHECK(state.currentTarget().Position().X_coord != 100); // Should not be the CV target
	}

	SUBCASE("queueSelectTarget adds a command to the queue") {
		state.queueSelectTarget(cerializer::TargetSource::RADAR, 1, 1000);

		// Process the queue after 1 second
		mock_time = TimePoint(milliseconds(1000));
		state.processCommandQueue();

		// Check that the target source was changed
		CHECK(state.target_source == cerializer::TargetSource::RADAR);
	}

	SUBCASE("queueFire adds a FireCommand to the queue") {
		state.queueFire(500); // Fire for 500 milliseconds

		// Before processing, fire state should be off
		CHECK_FALSE(state.getFireState());

		// Process at 250ms, should be on
		mock_time = TimePoint(milliseconds(250));
		state.processCommandQueue();
		state.actualizeState();
		CHECK(state.getFireState());

		// Process at 501ms, should be off
		mock_time = TimePoint(milliseconds(501));
		state.processCommandQueue();
		state.actualizeState();
		CHECK_FALSE(state.getFireState());
	}
}
