#include "doctest/doctest.h"
#include "state.h"
#include "command.h"
#include "command_queue.h"
#include "target.h"
#include "tests/mock_time.h"
#include <iostream>
#include <ostream>

// Test fixture for integration tests
struct IntegrationTestFixture {
    SystemState state;

    // Helper function to check motor positions
    void check_motor_positions(long expected_a, long expected_b) {
        CHECK(state.stepperA.currentPosition() == expected_a);
        CHECK(state.stepperB.currentPosition() == expected_b);
    }

    // Test method for direct state manipulation
    void test_direct_manipulation() {
        mock_clock.reset();
        TestClock::ScopedDeterministicClock scoped_clock;

        // Set the static target to a known position
        state.target_source = TargetSource::STATIC;
        state.staticTarget.Update(PositionVector(1, 0, 1.5));

        // Trigger the state update
        state.triggerTrackingUpdate();

        // Run the state update until the motors reach their destination
        int i = 0;
        do {
            state.actualizeState();
            i++;
        } while (i < 1000 && (state.stepperA.distanceToGo() != 0 || state.stepperB.distanceToGo() != 0));

        check_motor_positions(89, -1155);
    }

    // Test method for command-based manipulation
    void test_command_based_manipulation() {
        mock_clock.reset();
        TestClock::ScopedDeterministicClock scoped_clock;

        // Set up a CV target
        state.target_source = TargetSource::CV;
        state.cvTarget[0].Update(PositionVector(1, 0, 1.5));
        state.cvTarget[0].valid = true;

        // Queue a command to select the CV target
        state.queueSelectTarget(TargetSource::CV, 0, 0);

        // Advance the clock and process the queue
        mock_clock += Microseconds(1);
        state.processCommandQueue();

        // Run the state update until the motors reach their destination
        int i = 0;
        do {
            state.actualizeState();
            i++;
        } while (i < 1000 && (state.stepperA.distanceToGo() != 0 || state.stepperB.distanceToGo() != 0));

        check_motor_positions(89, -1155);
    }
};

TEST_CASE("Integration Tests") {
    IntegrationTestFixture fixture;

    SUBCASE("Updating a target and actualizing state moves the motors") {
        fixture.test_direct_manipulation();
    }

    SUBCASE("Queueing a command to select a target moves the motors") {
        fixture.test_command_based_manipulation();
    }
}
