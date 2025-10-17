# Firmware Class Reference

This document provides a high-level guide to the key C++ classes in the firmware. It focuses on their roles and interactions, providing context that goes beyond simple inline documentation.

## Core Classes

### `SystemState`
**File:** `state.h`, `state.cpp`

The `SystemState` class is the most important class in the firmware. It acts as the central nervous system, owning and managing all other components.

**Responsibilities:**
-   **State Monolith:** It holds the instances of the stepper motors, the command queue, and all target data arrays. There is a single, authoritative `SystemState` object.
-   **Hardware Abstraction:** It provides high-level methods like `setFire(bool)` and `setMove(bool)` that abstract the direct hardware control.
-   **Command Processing:** Its `processCommandQueue()` method is the heart of the main loop, responsible for executing commands at the correct time.
-   **State Actualization:** Its `actualizeState()` method translates the desired state (e.g., "aim at target X") into physical actions (e.g., "move stepper A 500 steps").
-   **Target Management:** It contains the logic for updating target information, whether it comes from the CV system or the radar.

Think of `SystemState` as the "god object" of the firmware, but in a way that's necessary for this kind of embedded system to maintain a coherent state.

### `Command` (and its subclasses)
**Files:** `command.h`, `command.cpp`, `firecontrol.h`, `target_selection.h`

`Command` is an abstract base class that defines the Command Design Pattern. It represents a single, deferred action.

**Key Subclasses:**
-   **`TargetSelection`:** Enqueues a command to change the currently active target. It contains the target's ID and the desired tracking speed.
-   **`FireControl`:** Enqueues a command to activate or deactivate the firing mechanism. It includes a duration, allowing for timed bursts.
-   **`LingerCommand`:** A "no-op" command that simply does nothing. Its purpose is to create a programmable delay in the command queue without blocking execution.

When a function like `SystemState::queueFire(duration)` is called, it doesn't immediately fire. Instead, it creates a `FireControl` object with the appropriate `run_after` timestamp and pushes it onto the command queue for later execution.

## Data and Targeting Classes

### `Target`
**File:** `target.h`, `target.cpp`

A `Target` object represents a single entity that the turret can aim at. It's more than just coordinates; it's a stateful object.

**Key Properties:**
-   `position`: The current 3D position of the target.
-   `velocity`: The calculated velocity of the target.
-   `last_seen`: A timestamp of when its data was last updated. This is crucial for expiring stale targets.
-   `valid`: A boolean flag indicating if the target data is current and should be considered for engagement.
-   `id`: A unique identifier, used by the CV system to track specific individuals across frames.

**Key Methods:**
-   `Update(PositionVector)`: Updates the target's position and recalculates its velocity based on the change from its previous position.
-   `PredictedPositionAtTime(...)`: Extrapolates the target's future position based on its current velocity.
-   `interceptPosition()`: The most complex method, which calculates the exact point in space to aim at to hit a moving target, accounting for projectile speed and gravity.

### Vector Classes (`PositionVector`, `VelocityVector`, `DistanceVector`)
**File:** `vector.hpp`, `target.h`

These classes provide a type-safe way to handle 3D vector math using fixed-point arithmetic (`fpm::fixed_16_16`), which is essential for performance on a microcontroller without a floating-point unit.

-   `PositionVector`: Represents an absolute point in 3D space (e.g., `(x, y, z)`).
-   `VelocityVector`: Represents a rate of change in position (e.g., `(vx, vy, vz)`).
-   `DistanceVector`: Represents a displacement or difference between two `PositionVector`s.

The C++ type system is used to prevent invalid operations, for example, you cannot add two `PositionVector`s together. You can, however, add a `DistanceVector` to a `PositionVector` to get a new `PositionVector`.

## Hardware Interface Classes

### `LD2450`
**File:** `LD2450.h`, `LD2450.cpp`

This class is a driver for the Hi-Link LD2450 radar sensor. It is a third-party library integrated into the project.

**Responsibilities:**
-   **Serial Communication:** It handles the low-level serial communication with the radar module, parsing its custom binary protocol.
-   **Data Extraction:** It reads the data stream from the sensor and populates a list of `RadarTarget_t` structs, which contain the x, y, speed, and resolution for up to three detected targets.
-   **Hardware Abstraction:** It provides a simple `read()` method that the main firmware can call to get the latest target data from the sensor without needing to know the details of the serial protocol.

The main firmware loop periodically calls `LD2450::read()` and uses the results to update the `radarTarget` array within `SystemState` when the system is in `TargetSource::RADAR` mode.