# Communication Protocol

The CV System ("Brains") and the Firmware ("Brawn") communicate over a standard USB serial connection. To ensure reliable and structured data exchange, a custom binary protocol is used. This protocol is defined entirely in the `serializer.hpp` header file.

## Core Design

The protocol is built around the concept of strongly-typed, fixed-size binary messages. This approach is efficient for microcontrollers, as it avoids the overhead of parsing text-based formats like JSON or XML.

Every message follows the same basic structure:

| Field       | Size (bytes) | Type          | Description                               |
|-------------|--------------|---------------|-------------------------------------------|
| **Magic Head**  | 2            | `uint16_t`    | A constant start-of-message marker: `0xCAFE`. |
| **Type Code**   | 1            | `uint8_t`     | A unique code identifying the message type. |
| **Payload**     | Variable     | `(various)`   | The actual data of the message.           |
| **Magic Foot**  | 2            | `uint16_t`    | A constant end-of-message marker: `0xFACE`.   |

The use of "magic numbers" for the header and footer allows the receiving system (`Deserializer` class) to easily find message boundaries within a continuous stream of serial data.

## Message Definition (`cerializer::Message`)

The `serializer.hpp` library provides a template class, `cerializer::Message`, that uses the Curiously Recurring Template Pattern (CRTP) to simplify the creation of new message types.

To define a new message, you create a class that inherits from `cerializer::Message` and provide three template arguments:
1.  The name of the derived class itself.
2.  A unique `uint8_t` type code.
3.  A variadic list of the data types that make up the message's payload.

**Example:**
```cpp
class Target : public Message<Target, 0, uint32_t, bool, uint16_t, uint16_t, uint16_t> {
    // ... field definitions ...
};
```
This automatically handles:
-   Registering the message type with a factory (`MessageMaker`).
-   Calculating the total size of the message.
-   Providing `ToBinary()` and `LoadBinary()` methods for serialization and deserialization.

## Defined Message Types

These are the messages currently implemented in the system.

---

### 1. `Target`
-   **Type Code:** `0`
-   **Direction:** CV System -> Firmware
-   **Purpose:** Transmits the primary target information from the vision system to the fire control system. This is the most frequently sent message during operation.
-   **Payload:**
    | Field   | Type          | Description                                       |
    |---------|---------------|---------------------------------------------------|
    | `id`    | `uint32_t`    | The unique ID of the person, from the CV database.  |
    | `valid` | `bool`        | `true` if the target is confirmed and engageable. |
    | `x`     | `uint16_t`    | The x-coordinate in mm.                           |
    | `y`     | `uint16_t`    | The y-coordinate in mm (distance forward).        |
    | `z`     | `uint16_t`    | The z-coordinate in mm (height).                  |

---

### 2. `Config`
-   **Type Code:** `1`
-   **Direction:** CV System -> Firmware
-   **Purpose:** Sends runtime configuration parameters to the firmware, allowing for on-the-fly tuning without recompiling.
-   **Payload:**
    | Field              | Type       | Description                                  |
    |--------------------|------------|----------------------------------------------|
    | `projectile_speed` | `float`    | The initial speed of the water stream in m/s.  |
    | `turret_height`    | `float`    | The height of the turret from the ground in m. |
    | `max_speed`        | `uint16_t` | The maximum speed for the stepper motors.      |
    | `acceleration`     | `uint16_t` | The acceleration for the stepper motors.     |

---

### 3. `SetTargetSourceMessage`
-   **Type Code:** `2`
-   **Direction:** CV System -> Firmware
-   **Purpose:** Commands the firmware to switch its active targeting system.
-   **Payload:**
    | Field    | Type       | Description                                                          |
    |----------|------------|----------------------------------------------------------------------|
    | `source` | `uint8_t`  | The target source to use. See `TargetSource` enum (`0`=STATIC, `1`=RADAR, `2`=CV). |

    This is used by the CV system to take control of the turret on startup (`CV`) and release control on shutdown (`STATIC`).

---

### 4. `StaticTargetMessage`
-   **Type Code:** `3`
-   **Direction:** CV System -> Firmware
-   **Purpose:** Provides explicit coordinates for the turret to aim at. This is used to set the `STATIC` target position.
-   **Payload:**
    | Field | Type       | Description                        |
    |-------|------------|------------------------------------|
    | `x`   | `uint16_t` | The x-coordinate in mm.            |
    | `y`   | `uint16_t` | The y-coordinate in mm (forward).  |
    | `z`   | `uint16_t` | The z-coordinate in mm (height).   |

This is also used by the CV system to send target coordinates when it is in control. While the name is `StaticTargetMessage`, it is sent repeatedly to update the aimpoint for a moving target. The name reflects that it's setting the single target coordinate that the firmware's `STATIC` mode tracks. The `TargetSource` determines how this message is interpreted.