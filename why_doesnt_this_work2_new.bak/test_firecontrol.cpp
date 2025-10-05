#include <cassert>
#include <chrono>
#include <thread>
#include "firecontrol.h"
#include "state.h"
#include "why_doesnt_this_work2_new.bak/mocks.h"
#include "why_doesnt_this_work2_new.bak/test_firecontrol.h"
#include "utilities.h"

// Test case for activating the firing mechanism
void test_FireControl_activate()
{
    SystemState state;
    state.setFire(false); // Ensure initial state is off
    // Target& target = state.currentTarget(); // Unused but kept for clarity

    // To make actionIdleExceeds true, we can just not set last_action
    // Or set it to a time in the past. The default constructor of Target sets it to epoch.
    // We need to wait a bit to make sure the time difference is large enough.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    FireControl cmd(true, 10, 0);
    cmd.Execute(&state);

    assert(state.getFireState() == true);
}

// Test case for deactivating the firing mechanism
void test_FireControl_deactivate()
{
    SystemState state;
    state.setFire(true); // Ensure initial state is on
    // Target& target = state.currentTarget(); // Unused.
    // By not calling IncrementAction(), the target's last_action remains in the
    // distant past, allowing the deactivation check to pass.

    // Wait for a duration longer than the command's duration
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    FireControl cmd(false, 10, 0);
    cmd.Execute(&state);

    assert(state.getFireState() == false);
}

// Test case to ensure firing doesn't happen if already active
void test_FireControl_already_active()
{
    SystemState state;
    state.setFire(true);

    FireControl cmd(true, 10, 0);
    cmd.Execute(&state);

    assert(state.getFireState() == true);
}

// Test case to ensure no change if already inactive
void test_FireControl_already_inactive()
{
    SystemState state;
    state.setFire(false);

    FireControl cmd(false, 10, 0);
    cmd.Execute(&state);

    assert(state.getFireState() == false);
}

// Test case for when action idle time has not been exceeded
void test_FireControl_activation_too_soon()
{
    SystemState state;
    state.setFire(false);
    Target& target = state.currentTarget();
    target.IncrementAction(); // Set last action time to now

    FireControl cmd(true, 10, 0);
    cmd.Execute(&state);

    assert(state.getFireState() == false);
}


// Test runner for the firecontrol module
void run_firecontrol_tests()
{
    test_FireControl_activate();
    test_FireControl_deactivate();
    test_FireControl_already_active();
    test_FireControl_already_inactive();
    test_FireControl_activation_too_soon();
}