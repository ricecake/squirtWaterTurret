# Project To-Do List

This document outlines the remaining work for the automated sentry squirt gun project.

## Testing and Validation

- **Comprehensive Test Suite:**
  - [ ] Write unit tests for all major C++ classes (`SystemState`, `CommandQueue`, `Target`, `FireControl`).
  - [ ] Write unit tests for Python helper functions in `cvTargetAssist.py`.
  - [ ] Ensure all serial message types can be serialized and deserialized correctly.
- **End-to-End Testing:**
  - [ ] Develop an automated test script that simulates the full workflow: CV system detects a target, sends a message, firmware aims and fires.
  - [ ] Create a mock serial port for testing the Python script's communication logic without needing hardware.
  - [ ] Implement integration tests to verify the interaction between the CV system and the firmware.
- **Hardware-in-the-Loop (HIL) Validation:**
  - [ ] Create a test procedure to verify the physical accuracy of the turret's aiming.
  - [ ] Test the system's performance with real-world targets under various lighting conditions.
  - [ ] Validate the effective range and accuracy of the water stream.

## Documentation

- **Doxygen C++ API Documentation:**
  - [ ] Add Doxygen-style comments to all C++ headers and source files.
  - [ ] Generate and publish a searchable HTML version of the C++ API documentation.
- **Python Script Documentation:**
  - [ ] Add docstrings to all functions and classes in `cvTargetAssist.py`.
  - [ ] Document the serial communication protocol in detail.
- **User & Developer Guides:**
  - [ ] Write a comprehensive guide on how to set up the hardware and software.
  - [ ] Create a developer guide that explains the project architecture and how to contribute.
  - [ ] Document the mathematical models used for ballistic calculations.

## Web Interface

- **Backend Development:**
  - [ ] Create a web server (e.g., using Flask or FastAPI) that runs alongside `cvTargetAssist.py`.
  - [ ] Implement a WebSocket interface to stream real-time data (camera feed, turret status, target info) to the frontend.
  - [ ] Create API endpoints to receive commands from the user (e.g., manual turret control, change target source).
- **Frontend Development:**
  - [ ] Design a simple, intuitive user interface using a modern web framework (e.g., React, Vue).
  - [ ] Display the annotated camera feed in real-time.
  - [ ] Add controls for manual aiming, firing, and mode selection.
  - [ ] Create a view to display the list of known targets from the `sqlite-vec` database.
  - [ ] Implement an interface for users to re-classify or label recognized individuals.
- **Integration:**
  - [ ] Connect the frontend to the backend WebSocket and API endpoints.
  - [ ] Ensure the web interface can reliably send commands to the firmware via the Python script.

## Target Selection and Prioritization

- **Implement `select_target` function:**
  - [ ] Implement the logic in the currently stubbed-out `select_target` function in the C++ firmware.
  - [ ] The function should consider the current `TargetSource` (STATIC, RADAR, CV).
  - [ ] It should factor in the time since a target was last fired upon to avoid repeatedly targeting the same object.
  - [ ] Add a proximity factor, prioritizing closer targets when multiple are available.
- **Dynamic Target Source Switching:**
  - [ ] Ensure the firmware can switch between target sources smoothly via the `SetTargetSourceMessage`.
  - [ ] Test the transitions between CV-provided targets and the radar-detected target.

## Code Simplification and Refactoring

- **Decouple Messages from Serializer:**
  - [ ] Move the `Message` class definitions out of `serializer.hpp` into their own dedicated header file (e.g., `messages.h`).
  - [ ] This will improve modularity and make the serializer a more generic tool.
- **Organize Command Structure:**
  - [ ] Create a `commands/` directory to hold different command types if the number of commands grows.
  - [ ] Evaluate if the current `Command` class can be simplified.
- **Improve Code Style and Consistency:**
  - [ ] Run `make format` to ensure all C++ code conforms to the `.clang-format` style.
  - [ ] Review the Python script for clarity, consistency, and adherence to PEP 8.
- **General Cleanup:**
  - [ ] Remove any commented-out or unused code.
  - [ ] Identify and refactor any inefficient patterns or duplicated logic.
  - [ ] Ensure all variable and function names are clear and descriptive.
