#include "state.h"
#include "doctest/doctest.h"
#include "mock_time.h"
#include "fpm_adapter.hpp"
#include "serializer.hpp"

TEST_CASE("SystemState") {
	SystemState state;
	mock_clock.reset();
	TestClock::ScopedDeterministicClock det_clock;

	SUBCASE("setMove and getMoveState") {
		state.setMove(true);
		CHECK(state.getMoveState() == true);
		state.setMove(false);
		CHECK(state.getMoveState() == false);
	}

	SUBCASE("setStrategy and getStrategy") {
		state.setStrategy(TurretStrategy::CLOSEST);
		CHECK(state.getStrategy() == TurretStrategy::CLOSEST);
		state.setStrategy(TurretStrategy::FURTHEST);
		CHECK(state.getStrategy() == TurretStrategy::FURTHEST);
	}

	SUBCASE("setStance and getStance") {
		state.setStance(TurretStance::PASSIVE);
		CHECK(state.getStance() == TurretStance::PASSIVE);
		state.setStance(TurretStance::AGGRESSIVE);
		CHECK(state.getStance() == TurretStance::AGGRESSIVE);
	}

	SUBCASE("updateConfig") {
		cerializer::Config config(10.0f, 1.5f, 1000, 500);
		state.updateConfig(&config);

		CHECK(static_cast<double>(state.getConfig().projectile_speed) == doctest::Approx(10.0));
		CHECK(static_cast<double>(state.getConfig().turret_height) == doctest::Approx(1.5));
	}

	SUBCASE("Target source handling") {
		// Default is STATIC
		CHECK(state.currentTargetArray().size() == 1);

		state.target_source = TargetSource::RADAR;
		CHECK(state.currentTargetArray().size() == 3); // RADAR target buffer is 3
		state.setTarget(TargetSource::RADAR, 0, 0);
		CHECK(&state.currentTarget() == &state.radarTarget[0]);

		state.target_source = TargetSource::CV;
		CHECK(state.currentTargetArray().size() == 32); // CV target buffer is 32
		state.setTarget(TargetSource::CV, 0, 0);
		CHECK(&state.currentTarget() == &state.cvTarget[0]);

		state.target_source = TargetSource::STATIC;
		CHECK(state.currentTargetArray().size() == 1);
		state.setTarget(TargetSource::STATIC, 0, 0);
		CHECK(&state.currentTarget() == &state.staticTarget);
	}

	SUBCASE("queueFire") {
		state.config.fireActionInterval = 0;
		state.queueFire(100);
		mock_clock.set(1);
		state.processCommandQueue(); // Fire command should be added
		CHECK(state.getFireState() == true);
	}
}
