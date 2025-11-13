#include "doctest/doctest.h"
#include "firecontrol.h"
#include "state.h"
#include "mock_time.h"

TEST_CASE("FireControl") {
	SystemState state;
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("activate") {
		state.config.fireActionInterval = 0;
		FireControl cmd(true, 100);
		cmd.Execute(&state);
		CHECK(state.getFireState() == true);
	}

	SUBCASE("deactivate") {
		state.setFire(true);
		FireControl cmd(false, 100);
		cmd.Execute(&state);
		CHECK(state.getFireState() == false);
	}

	SUBCASE("activation too soon") {
		state.config.fireActionInterval = 1000000;
		state.currentTarget().IncrementAction();
		FireControl cmd(true, 100);
		cmd.Execute(&state);
		CHECK(state.getFireState() == false);
	}
}
