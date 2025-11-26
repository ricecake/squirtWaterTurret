# AI Coding Agent Instructions: Dual Stepper Sentry Turret

## Project Overview

An automated water turret system combining computer vision (Python/OAK-D) with real-time firmware control (C++/ESP32). The system detects targets, calculates ballistic trajectories, and aims dual stepper motors for precise targeting. Architecture splits "Brains" (CV detection) from "Brawn" (motor control).

## Critical Architecture Patterns

### Dual-Mode Binary Communication (Custom `serializer.hpp`)
- **Custom protocol**: Binary messages with `0xCAFE` (header) and `0xFACE` (footer) magic numbers, NOT JSON/XML
- **Type-safe via CRTP**: Messages inherit from `cerializer::Message<DerivedClass, uint8_t_TypeCode, PayloadTypes...>`
- **Key message types**: `Target` (0x00, CV→FW), `Config` (0x01), `SetTargetSourceMessage` (0x02), `StaticTargetMessage` (0x03)
- **Location**: `serializer.hpp` defines the entire protocol; new messages rarely needed
- **When modifying**: Always preserve magic numbers and sync Python/C++ serialization logic

### Command Pattern with Time-Sequenced Queue
- **Time-based execution**: Commands run at specified timestamps, not immediately (`run_after` field in `Command`)
- **Thread-safe priority queue**: `CommandQueue` uses mutex, priority ordered by `run_after` value
- **Derived command classes**: Inherit from `AutoCommand<YourCommand>` for automatic type naming via `type_name<T>()`
- **State pattern integration**: Commands call `SystemState::Execute()` to modify turret state
- **Example flow**: CV system queues `SetStrategyCommand` → triggers `TargetSelectionCommand` → enqueues fire sequence
- **Key file**: `command_queue.h` (process loop), `command.h` (base class)

### Ballistic Targeting with Numerical Approximation
- **No analytical solution**: Solving the quartic ballistic equation is too expensive for ESP32
- **Hybrid root-finder**: `Approximate::small_root()` (`aproximate_math.hpp`) uses Secant method + Bisection fallback
- **Guarantees**: Converges in fixed rounds (~16) with predictable CPU time
- **Moving target tracking**: `Target::interceptPosition()` predicts future target position using velocity vector
- **Gravity model**: Accounts for lateral motion, assumes constant target velocity
- **Integration point**: `FireControl` class calls the approximation; results drive motor angles

### Differential Wrist Kinematics
- **Pan/Tilt via two fixed motors**: Inverse kinematics convert desired (pan, tilt) angles to motor step counts
- **Motor relationship**: `α_pan = k * (θ₁ + θ₂) / 2`; `β_tilt = k * (θ₁ - θ₂) / 2`
- **Step calculation**: `PositionVector::Pitch()` and `PositionVector::Yaw()` compute angles from 3D target
- **AccelStepper integration**: Motor movement abstracted; `state.h` manages motor objects
- **Key file**: `vector.hpp` (angle math), `firecontrol.cpp` (motor commands)

### Conditional Arduino Compilation
- **Dual targets**: Code compiles for both ESP32 (with Arduino framework) and Linux (for testing)
- **Preprocessor guards**: `#ifdef ARDUINO` wraps hardware deps (AccelStepper, FreeRTOS, HardwareSerial)
- **Mock layer**: `tests/mocks.h` provides stubs for SerialHardware, AccelStepper, GPIO; enables PC-based unit tests
- **Clock injection**: `Clock::setClock()` allows test control of time; default uses `std::chrono::high_resolution_clock`
- **Testing consequence**: Always use `Clock::now()` instead of calling hardware directly

## Developer Workflows

### Building & Testing
```bash
make testbin   # Compile C++ unit tests (Linux target)
make sketch    # Compile firmware mock binary
make test      # Run unit test suite
make smoke     # Run firmware integration smoke test
make clean     # Remove build artifacts
make format    # Apply clang-format to all C++ code
```
- **Compiler flags**: `-std=gnu++23`, sanitizers enabled (ASan, UBSan) for leak/overflow detection
- **Test framework**: doctest; see `tests/main.cpp` for entry point
- **No external test runners**: Tests link directly against core .cpp files (not mock binary)

### Key Test Files by Component
- `test_command_queue.cpp`: Time-sequenced execution, priority ordering
- `test_firecontrol.cpp`: Ballistic calculation integration
- `test_serializer.cpp`: Binary message round-tripping
- `test_state.cpp`: State machine transitions, motor control logic
- `test_vector.cpp`: 3D vector math, angle calculations
- `test_approximate_math.cpp`: Root-finding convergence

### Serial Protocol Testing
- Python script `serial_protocol.py` simulates CV system messages
- Firmware reads/parses via `cerializer::StreamHandler` in `.ino` file
- Test message format: always wrap payloads with CAFE/FACE envelope
- Verify via `hexify()` utility to inspect raw bytes

## Code Organization & Conventions

### Header-Only Libraries
- **`serializer.hpp`**: 900-line header; defines message factory, serialization, deserialization
- **`fpm_adapter.hpp`**: Fixed-point math wrapper (16.16 format `fixed_16_16`), avoids floats on ESP32
- **`vector.hpp`**: 3D vector operations using fixed-point math, angle extraction
- **`utilities.h`**: Clock abstraction, type name extraction (`type_name<T>()` via `std::source_location`)

### Separation of Concerns
- **`state.h`**: Overall system state, command queue, target management
- **`command.h/.cpp`**: Command definitions and execution logic
- **`firecontrol.h/.cpp`**: Ballistic computation, motor target calculation
- **`target_selection.h/.cpp`**: Strategy selection (CLOSEST, LEAST_HIT, RANDOM, etc.)
- **`target.h/.cpp`**: Single target state tracking, position updates

### Method Organization by Category
Classes organize methods by logical grouping to improve readability and maintainability:

#### Target Class (`target.h`)
- **Update/Mutation**: `Update()`, `IncrementAction()`, `PredictedPositionAtTime()`
- **Getters**: `Position()`, `Velocity()`, `Distance()` (grouped together)
- **Time & State Queries**: `timeSinceLastAction()`, `timeSinceLastSeen()`, `actionable()`, `idleExceeds()`, `actionIdleExceeds()`
- **Complex Logic**: `InterceptAimpoint()` (ballistic intercept calculation)
- **Time Predicates use operator overloading**: `Clock::now() - last_action > milliseconds(100)` instead of named methods

#### SystemState Class (`state.h`)
- **Query Methods**: `currentTarget()`, `currentTargetArray()`, `size()`
- **Target Updates** (grouped): `updateTarget()`, `updateTargetById()`, `updateNearestTarget()`, `updateNearestTarget2d()`
- **Getter/Setter Pairs** (grouped by domain):
  - Fire control: `setFire()`/`clearFire()` + `getFireState()`
  - Movement: `setMove()` + `getMoveState()`
  - Strategy: `setStrategy()` + `currentStrategy()`
  - Target selection: `setTarget()`, `setStance()`
- **State Validation**: `shouldCheckTargetValidity()`, `shouldCheckFiringConditions()`, `targetIsPotentiallyValid()`
- **Command Queue**: `queueSelectTarget()`, `queueFire()`, `queueCeaseFire()`, `processCommandQueue()`
- **Configuration**: `updateConfig()`
- **State Actualization**: `actualizeState()` (complex logic grouped in state.cpp)
- **Physics Calculations**: `targetTravelDistance()`, `currentYaw()`, `currentPitch()` (complex math grouped)
- **Target Search**: `fetchTarget()`, `fetchNearestTargetIdx()`, `fetchNearestTarget2dIdx()`

**Benefit**: Related getter/setter pairs are side-by-side, making it easy to see what state can be queried and modified. Complex calculations are grouped at the end, separate from simple accessors.

### Integer-Only Fixed-Point Math
- **No floats in firmware**: Use `fixed` type (16.16 fixed-point from `fpm/fixed.hpp`)
- **Speed constants**: `projectileSpeed = 25.0` in shared_types.h (in meters/sec)
- **Altitude default**: `altitude = 1.2` (turret height in meters)
- **Config updates**: Runtime tuning via `SetStrategyCommand`; never hardcode physics values

## Integration Points & Data Flow

### CV System → Firmware Pipeline
1. **Detection**: `cvTargetAssist.py` runs OAK-D neural inference
2. **Serialization**: Detected position → `Target` message (type 0x00) with (x, y, z) coordinates
3. **Serial send**: Python writes binary envelope (CAFE + payload + FACE)
4. **Firmware reception**: `IOWrapper::readsome()` → `cerializer::StreamHandler` → deserialize → update target
5. **State update**: `SystemState::updateTarget()` refreshes active target position
6. **Command enqueue**: Automatic motion commands triggered by state machine in `actualizeState()`

### Radar Fallback Mode
- **LD2450 sensor integration**: Firmware can switch to `TargetSource::RADAR` if CV system is unavailable
- **Motion detection only**: Radar provides distance/angle, not identity; uses STATIC or fallback strategies
- **Controlled via**: `SetTargetSourceMessage` (type 0x02) sent from CV system on startup/shutdown

### Target Selection Strategies
- **Enum**: `TurretStrategy` in `shared_types.h` (CLOSEST, LEAST_HIT, RANDOM, SMALLEST_TRAVEL, etc.)
- **Decision logic**: `target_selection.cpp` implements each strategy
- **Runtime switch**: `SetStrategyCommand` changes active strategy without recompile
- **Default**: CLOSEST on boot

## Testing Patterns

### Unit Test Structure
```cpp
#include "common.h"  // Brings in doctest, mocks, clock

TEST_CASE("Description") {
    Clock::setClock(MockClock::now);  // Inject test clock
    SystemState state;
    // ... test logic ...
    CHECK(condition);
}
```

### Mock Time for Command Queue Testing
- **MockClock**: Allows manual time advancement in tests
- **Usage**: `Clock::setClock(MockClock::now)` at test start
- **Advance time**: Call `mock_clock.advance()` to trigger scheduled commands
- **Verification**: Commands execute in order of `run_after` timestamp

### Serial Message Testing
- **Round-trip validation**: Create message → `ToBinary()` → parse bytes → `LoadBinary()` → verify fields match
- **Boundary conditions**: Test with min/max coordinate values (0–65535mm for uint16_t fields)
- **Deserializer recovery**: Verify `Deserializer::State` transitions correctly across magic numbers

## Project-Specific Gotchas

### Fixed-Point Precision
- **16.16 format**: Minimum increment is 1/65536 of a unit; watch for rounding in ballistic iteration
- **Overflow risk**: Temporary multiplications can exceed int32 range; cast carefully
- **Velocity precision**: Target velocity updates may lose significance; test with large motions
- **Adapter layer**: `fpm_adapter.hpp` wraps `fpm::fixed` for operator overloads; use typedef `fixed` (=`fixed_16_16`)

### Arduino Compilation Quirks
- **No direct float usage**: Even `1.0` literals get evaluated; use fixed-point constants
- **Memory constraints**: ESP32 has 520KB SRAM; avoid large allocations
- **String type naming**: `std::source_location` for `type_name<T>()` requires C++20; Makefile enforces `-std=gnu++23`

### Command Timing Assumptions
- **Zero-time scheduling**: Commands with `run_after=0` execute immediately in same loop iteration
- **Order matters**: If multiple commands have same `run_after`, priority queue order is stable but not guaranteed
- **Manual timing**: For precise sequencing, use `addCommandAfter()` to auto-increment timestamps

### Stepper Motor Microsteps
- **Step fraction**: `SystemState::stepFraction = 16` (from `state.h`)
- **Motor abstraction**: AccelStepper handles microstep interpolation
- **Speed/acceleration**: Tuned via `Config` message; default values in firmware

## ESP32 Optimization Strategies

### Memory Efficiency
- **Avoid std::string in hot paths**: Use `std::string_view` for message parsing (see `serializer.hpp`)
- **Stack vs heap**: Command queue uses `std::shared_ptr<Command>` (heap) to avoid giant stack frames; mutex guards prevent race conditions
- **Target array bounds**: `cvTarget` and `radarTarget` arrays are fixed-size (not dynamic); use span for iteration
- **SRAM budget**: ~80KB typical footprint for state + motors + target array; prioritize test validation before deployment

### CPU Efficiency
- **Ballistic iteration budget**: `Approximate::small_root()` runs in ~16 fixed rounds; guarantees predictable timing even at 10 FPS camera input
- **Avoided expensive operations**: No `sin()`, `cos()`, `sqrt()` in main loop; use precomputed LUTs or fixed-point approximations where possible
- **Task prioritization**: Use FreeRTOS (`#ifdef ARDUINO`) semaphores for serial reads; main loop processes commands queued from ISR
- **Serial I/O**: Non-blocking via `IOWrapper::readsome()`; never call blocking `Serial.read()` in main control loop

### Numerical Stability
- **Intermediate casts**: When multiplying fixed-point values, cast to `int64_t` to prevent overflow: `fixed result = fixed(int64_t(a.raw_value()) * b.raw_value() / (1 << 16))`
- **Ballistic stability**: Root-finder uses both Secant (fast) and Bisection (robust) methods; if either method diverges, bisection ensures convergence
- **Velocity damping**: Target velocity updates use exponential smoothing to filter sensor noise; large motion deltas are clamped to realistic max values
- **Motor saturation**: `AccelStepper::moveTo()` queues target position; never interrupt mid-move; allow full travel before queuing next target

### Power & Thermal
- **FPS throttling**: CV system defaults to 10 FPS (tunable via `--fps` arg); lower FPS = reduced USB bandwidth and ESP32 interrupt load
- **Idle strategies**: When no valid target, firmware uses RADAR fallback or static idle point (not continuous motion)
- **Watchdog timers**: FreeRTOS task watchdog active; commands that timeout are logged; add timeout parameters to long-running ops

## cvTargetAssist.py Patterns & Integration

### Detection Pipeline Architecture
- **Multi-stage processing**: YOLO pose detection → keypoint extraction → torso ROI cropping → embedding generation → database lookup
- **Settings injection**: `ScriptSettings` class populated at module load by `parse_and_setup_settings()`; all detection params stored here
- **Mode selection**: `--mode POSE` (body targeting) vs. `--mode FACE` (face recognition); each mode has separate ROI logic
- **FPS control**: `--fps` parameter controls camera pipeline rate; default 10 FPS balances latency vs. computational load

### Target Calculation & Serial Transmission
- **Keypoint mapping**: 17-point YOLOv8 model; indices 5-6 (shoulders), 11-12 (hips) define torso center for targeting (more robust than nose/single joint)
- **3D localization**: Stereo depth from OAK-D camera; (x, y, z) coordinates mapped to target position in mm
- **Validity state**: Target validity tracked separately from presence; invalid targets excluded from aiming but retained in history
- **Serialization format**: Target message contains (id, valid, x, y, z); ID links to `sqlite-vec` database for person identification
- **Transmission**: Binary message wrapped with `0xCAFE` header + type code (0x00) + payload + `0xFACE` footer; sent via `serial.write()`

### Database & Person Identification
- **SQLite with vec extension**: Embeddings stored as vectors; lookup via `sqlite_vec` L2 distance queries
- **Confidence thresholds**: Three-tier matching: `CLOSE_MATCH_THRESHOLD` (0.16, high confidence), `MEDIUM_MATCH_THRESHOLD` (0.22), `FAR_MATCH_THRESHOLD` (0.30)
- **New person enrollment**: If no match above far threshold, new database entry created with embedding
- **Person state tracking**: `sqlite-vec` database retains historical targets; re-identification improves confidence over multiple frames
- **Embedding freshness**: For each new detection, embedding recalculated; averaging or denoising can improve stability (not yet implemented)

### Robustness & Error Handling
- **Detection fallback**: If pose detection fails, firmware switches to `TargetSource::RADAR` via `SetTargetSourceMessage` (type 0x02)
- **Serial reliability**: CV system continuously streams targets; firmware updates only on valid new detections (ignores duplicates within indifference margin)
- **Startup/shutdown handshake**: CV sends `SetTargetSourceMessage` (CV mode) on init, sends STATIC mode on exit; allows graceful fallback
- **Timeout detection**: If no CV messages for N seconds, firmware auto-switches to radar; configurable via config parameter
- **Frame skipping**: If camera drops frames, embedded timestamps allow firmware to detect stale data and ignore

### Configuration & Tuning
- **Runtime config updates**: `Config` message (type 0x01) allows CV system to send `projectile_speed`, `turret_height`, motor speed/acceleration without recompile
- **Threshold tuning**: Match confidence thresholds tunable via `--close-match-threshold`, `--medium-match-threshold`, `--far-match-threshold` CLI args
- **Database state**: `--state` parameter creates separate database directories; useful for multi-user or multi-target scenarios
- **Visualizer mode**: `--enable-visualizer` streams frames to remote UI for debugging; has minimal overhead (~5% CPU)

## Documentation References
- **Architecture overview**: `documentation/Architecture.md`
- **Communication protocol**: `documentation/Communication_Protocol.md`
- **Ballistic math**: `documentation/Algorithms_and_Math.md`
- **Core concepts**: `documentation/Core_Concepts.md`

## Key Files for Common Tasks

| Task | Primary File | Secondary Files |
|------|--------------|-----------------|
| Add new message type | `serializer.hpp` | `shared_types.h`, `dualStepperDPTStartWithRadar.ino` |
| Fix ballistic accuracy | `aproximate_math.hpp`, `firecontrol.cpp` | `test_firecontrol.cpp`, `test_approximate_math.cpp` |
| Change targeting strategy | `target_selection.cpp`, `command.cpp` | `shared_types.h`, `tests/test_target_selection.cpp` |
| Adjust motor response | `firecontrol.cpp`, `state.cpp` | `vector.hpp`, `Makefile` (flags) |
| Debug motor timing | `tests/mock_time.h`, `command_queue.h` | `tests/test_command_queue.cpp` |
| Extend Python integration | `cvTargetAssist.py`, `serial_protocol.py` | `serializer.hpp` (ensure sync) |
| Optimize SRAM usage | `state.h` (reduce array sizes) | `command.h` (reduce message payload) |
| Add new detection mode | `cvTargetAssist.py` (YOLO pipeline) | `dualStepperDPTStartWithRadar.ino` (message handler) |
| Implement person re-ID | `cvTargetAssist.py` (embedding lookup) | `sqlite-vec` queries in database module |

## Common Integration Workflows

### Deploying a CV System Update
1. **Modify detection logic** in `cvTargetAssist.py` (e.g., new keypoint mapping, ROI cropping)
2. **Test serialization** with `serial_protocol.py`; verify hex output matches expected message format
3. **Test firmware reception** by running `./build/mock_binary < test_data.bin`; confirm `SystemState::updateTarget()` is called
4. **Validate on hardware**: Compile firmware with `make sketch`, flash to ESP32, run CV system with `python3 cvTargetAssist.py --serial-output`
5. **Monitor**: Check firmware logs for target updates; compare actual vs. expected coordinates

### Tuning Ballistic Accuracy
1. **Identify miss pattern**: Firmware logs estimated intercept time; compare vs. observed water stream trajectory
2. **Adjust speed constant** in `shared_types.h`: `const fixed projectileSpeed = 25.0;` (in m/sec)
3. **Recalibrate turret height** in same file: `const fixed altitude = 1.2;` (in meters above ground/nozzle)
4. **Send `Config` message** from CV system to update on-the-fly: `Config(speed=26.0, height=1.3, ...)`
5. **Re-test with `test_firecontrol.cpp`**: Modify expected intercept angle, run `make test`, verify pass/fail

### Adding a New Command Type
1. **Define enum** in `shared_types.h` if the command affects strategy/stance
2. **Create class** inheriting from `AutoCommand<YourCommand>` in `command.h`/`.cpp`
3. **Implement `Execute(SystemState*)`**: Calls `state->method()` to modify system behavior
4. **Queue from CV system**: Send appropriate message type or have firmware trigger via state machine
5. **Test timing** with `test_command_queue.cpp`: Verify command runs at expected `run_after` timestamp

### Switching Between CV and Radar
1. **CV → Radar**: Python script exits cleanly, sends `SetTargetSourceMessage(TargetSource::RADAR)`; firmware auto-switches
2. **Radar → CV**: Python script starts, sends `SetTargetSourceMessage(TargetSource::CV)` on init; firmware starts accepting `Target` messages
3. **Static fallback**: Both systems can set static target via `StaticTargetMessage`; useful for demo or safe mode
4. **Verification**: Firmware logs active `target_source`; check with serial monitor or test harness

### Debugging Serial Communication
1. **Enable hex output**: Add `cout << hexify(payload)` in `serializer.hpp` deserializer or Python `serial_protocol.py`
2. **Capture raw bytes**: `python3 cvTargetAssist.py --log-level DEBUG 2>&1 | tee /tmp/cv.log`
3. **Compare message format**:
   - Expected: `CAFE [TYPE:1byte] [PAYLOAD] FACE`
   - Example Target: `CAFE 00 12345678 01 00000100 00000200 000004B0 FACE`
4. **Verify deserializer state machine**: In `serializer.hpp`, `Deserializer::State` transitions through SYNC_HEAD → READING_TYPE → READING_PAYLOAD → SYNC_FOOT
5. **Test with `test_serializer.cpp`**: Round-trip a message, verify fields after deserialization

## Questions to Iterate

When creating or modifying code, verify:
1. **Is this Arduino-targeted or test-targeted?** Use `#ifdef ARDUINO` if needed; ensure mocks exist.
2. **Does this modify serialization?** Update both Python and C++ sides; test with `test_serializer.cpp`.
3. **Does this affect motor timing?** Run `make test` and `make smoke` to validate command queue order.
4. **Should this use fixed-point math?** Check against physics constants in `shared_types.h`; avoid inline floats.
5. **Is new functionality testable on Linux?** If not, provide mocks in `tests/mocks.h`.
