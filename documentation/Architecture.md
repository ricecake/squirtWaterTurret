# System Architecture

This document provides a high-level overview of the automated sentry turret's architecture. The system is best understood as a classic "Brains" vs. "Brawn" model, where a powerful host computer handles complex processing and a dedicated microcontroller manages real-time hardware control.

## Core Components

The system is composed of three main parts:

1.  **The Host System (The "Brains"):** A Python application responsible for detecting, identifying, and tracking targets. It runs on a standard computer connected to a Luxonis OAK-D camera.
2.  **The Control System (The "Brawn"):** A C++ application running on an ESP32 microcontroller. It is responsible for the physical movement of the turret, firing the water pump, and processing sensor data.
3.  **The Physical Turret:** The 3D-printed pan/tilt mechanism, stepper motors, water pump, and nozzle.

## System Diagram

```
+------------------------+          (Serial)          +-----------------------+
|    Host System (PC)    |                            |  Control System (ESP32) |
|------------------------|                            |-----------------------|
| - Python 3             |<-------------------------->| - C++ Firmware        |
| - `depthai` for CV     |                            | - Stepper Motor Control |
| - `sqlite-vec` for ID  |       (Target Coords)      | - Water Pump Control  |
| - Target Selection     |                            | - Radar Sensor Input  |
+------------------------+                            +-----------------------+
             |                                                     |
             | (Camera Feed)                                       | (PWM/Step/Dir Signals)
             |                                                     |
+------------------------+                            +-----------------------+
| Luxonis OAK-D Camera   |                            |   Physical Turret     |
+------------------------+                            +-----------------------+

```

## Workflow: From Detection to Firing

1.  **Perception:** The `cvTargetAssist.py` script on the host system initializes the OAK-D camera and its `depthai` pipeline. The pipeline's neural network processes the video feed to detect human poses.

2.  **Identification & Tracking:** When a person is detected, the system extracts a feature vector (embedding) from the image. This vector is compared against a `sqlite-vec` database to determine if the person has been seen before. The system maintains a list of current targets and their 3D positions relative to the camera.

3.  **Target Selection:** The host system selects a primary target from the list of candidates.

4.  **Communication:** The 3D coordinates of the selected target are serialized into a custom binary format and sent to the ESP32 over a USB serial connection.

5.  **Command Reception:** The ESP32 firmware continuously listens for incoming serial data. When it receives a valid target coordinate message, it enqueues a command to aim the turret.

6.  **Ballistic Calculation:** The firmware takes the target's (x, y, z) coordinates and calculates the required pan and tilt angles to hit it. This involves a numerical approximation to account for the ballistic trajectory (gravity's effect on the water stream).

7.  **Execution:** The firmware's control loop executes the aiming command, driving the stepper motors to the calculated angles. Once aimed, it can execute a `FIRE` command to activate the water pump for a set duration.

8.  **Independent Operation:** If no commands are received from the host system, the ESP32 can operate in a fallback mode using an LD2450 radar sensor to detect nearby motion and engage targets.

This separation of concerns allows each component to do what it does best. The host system handles computationally intensive vision processing, while the microcontroller provides the reliable, low-latency, real-time control required for precise physical movement.