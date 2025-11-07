# Project: Automated Sentry Squirt Gun

An exercise in applying computer vision, real-time control, and a bit of math to the important task of squirting things with water.

## How It Works

The system is comprised of two primary components: a computer vision module for target identification and a microcontroller for aiming and firing.

### The "Brains": Computer Vision & Target Management

The high-level logic runs as a Python script, leveraging a Luxonis OAK-D camera for its perception capabilities.

*   **`depthai`:** Used for its powerful, on-device neural processing. A YOLO-based pose estimation model identifies and tracks potential targets in real-time.
*   **`sqlite-vec`:** Provides on-device vector similarity search. This allows the system to store embeddings for identified individuals and recognize them later, because it's important to remember who you've squirted.
*   **Target Serialization:** Once a valid target is acquired, its 3D coordinates are serialized and transmitted to the control system over a serial connection.

### The "Brawn": Firmware & Real-Time Control

The low-level control system is a C++ application running on an ESP32 microcontroller.

*   **Motion System:** It controls a pan/tilt turret built with two stepper motors, using the `AccelStepper` library for smooth, coordinated movement.
*   **Dual-Mode Input:** The firmware can operate in two modes. It can either receive target coordinates from the main CV system or function independently using an LD2450 radar module for basic motion detection.
*   **Command Queue:** To manage operations, the firmware uses a time-sequenced command queue. Actions like "move to coordinates," "select target," and "fire" are scheduled and executed in order. This allows for sequencing complex behaviors without blocking the main control loop.

### The Math: Ballistic Compensation

To hit a target at a distance, you can't just point and shoot. You have to account for gravity.

*   **The Problem:** Calculating the correct launch angle to hit a target at a given distance and height involves solving a rather nasty quartic equation, which is computationally expensive for a microcontroller.
*   **The Solution:** Instead of solving it directly, we use a numerical approximation method. This approach finds a "good enough" solution in a fixed, predictable amount of time, making it suitable for real-time applications. The result is a system that can arc a stream of water with surprising accuracy.

## Frequently Asked Questions

### Isn't this massively too complicated for a squirt gun?
Yes. Next question.

### Why the custom serialization protocol?
Because inventing a marginally adequate solution from scratch is an important part of the learning process. It also avoids adding another dependency.

### What was the thinking behind the library design?
The C++ code is structured to be as modular as possible within the constraints of the Arduino environment. State, targeting logic, and hardware control are separated into their own classes to improve testability and allow for easier modification. For instance, the stepper control logic is abstracted away from the main state machine, which only cares about high-level commands.

### What kind of targets?
Unsuspecting.

## Technology Stack

### Hardware
*   Luxonis OAK-D
*   ESP32 Microcontroller
*   Dual NEMA-17 Stepper Motors
*   LD2450 Radar Sensor
*   A water pump and nozzle assembly
*   A 3D-Printed Mount

### Software & Libraries
*   Python 3
*   C++ (Arduino Framework)
*   `depthai`
*   `sqlite-vec`
*   `AccelStepper`

## Getting Started

1.  Assemble the hardware. You're on your own here.
2.  Flash the firmware in `dualStepperDPTStartWithRadar.ino` to the ESP32 using your preferred tool (e.g., Arduino IDE, PlatformIO).
3.  Run the `cvTargetAssist.py` script on a machine connected to the OAK-D camera.
4.  Ensure the Python script can communicate with the ESP32 over a serial port.
5.  Add water. Power on.
6.  Try not to make a mess.

## Disclaimer
This is a hobby project. It is provided as-is, without warranty of any kind. The author is not responsible for any water damage, startled pets, or annoyed housemates that may result from its use.