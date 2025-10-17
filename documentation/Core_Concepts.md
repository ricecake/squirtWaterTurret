# Core Firmware Concepts

This document details the core concepts and design patterns that govern the C++ firmware. Understanding these concepts is key to modifying or extending the firmware's behavior.

## 1. Time-Sequenced Command Queue

The heart of the firmware's multitasking capability is a non-blocking, time-sequenced command queue. The system needs to perform multiple tasks concurrently: listen for serial commands, update motor positions, process sensor data, and control the firing mechanism. A simple, linear execution model would be unresponsive.

### The Problem: Blocking Operations

Many operations, like waiting for a motor to reach its destination or waiting for a specific time to fire, would typically "block" the processor, preventing it from doing other work. For example, a `delay(1000)` call would halt all other activity for one second.

### The Solution: The Command Pattern

The firmware implements the [Command Design Pattern](https://en.wikipedia.org/wiki/Command_pattern) to solve this.

-   **`Command` Class:** A `Command` is an object that represents an action to be taken. Key examples include `TargetSelection`, `FireCommand`, and `LingerCommand`.
-   **`run_after` Timestamp:** Each command has a `run_after` property, which is a timestamp (in microseconds) indicating the earliest time it should be executed.
-   **Priority Queue:** All pending commands are stored in a `std::priority_queue`. This queue is special because it sorts commands not by importance, but by their `run_after` timestamp. The command with the *earliest* timestamp is always at the front of the queue.

### The Main Loop (`SystemState::processCommandQueue`)

The main control loop operates as follows:
1.  Get the current time (`now`).
2.  Look at the command at the front of the queue.
3.  If `now >= command.run_after`, it's time to execute this command.
4.  The command is popped from the queue and its `Execute()` method is called.
5.  The loop repeats.

This architecture ensures the firmware is always responsive. A command to "fire in 5 seconds" doesn't block the system for 5 seconds. Instead, a `FireCommand` with a `run_after` timestamp of `now + 5,000,000` is added to the queue. The system can continue processing other commands (like tracking a moving target) in the meantime. When the 5-second mark is passed, the `FireCommand` will naturally rise to the top of the queue and be executed.

## 2. State Management (`SystemState`)

The `SystemState` class is the central hub of the firmware. It is a singleton-like object that owns and manages all other components of the system:
-   The stepper motor instances (`AccelStepper`).
-   The command queue.
-   The target arrays (`cvTarget`, `radarTarget`, `staticTarget`).
-   The current system configuration (`ConfigParameters`).
-   The current targeting mode (`target_source`).

This centralized approach ensures that there is a single source of truth for the state of the system. Commands operate on the `SystemState` object to effect change.

## 3. Targeting Modes (`TargetSource`)

The firmware can acquire targets from three different sources, defined by the `TargetSource` enum. The active source determines which array of `Target` objects the system uses for aiming.

-   **`TargetSource::STATIC`:**
    -   The turret aims at a fixed point in space.
    -   This mode is used for calibration or as a default "home" position.
    -   The coordinates are stored in the `staticTarget` object.

-   **`TargetSource::CV` (Computer Vision):**
    -   The turret receives target coordinates from the host Python script.
    -   This is the primary operating mode.
    -   The host sends `Target` messages over serial, which the firmware uses to update its list of `cvTarget` objects. The system then aims at the currently selected target from this list.

-   **`TargetSource::RADAR`:**
    -   The turret uses the LD2450 radar sensor for autonomous target acquisition.
    -   The radar provides basic motion detection and position information.
    -   This is a fallback mode for when the host CV system is not connected. The firmware updates the `radarTarget` array based on sensor readings.

Switching between these modes is handled by a `SetTargetSourceMessage` sent from the host, which allows the CV system to enable or disable its own control over the turret.