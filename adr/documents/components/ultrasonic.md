# Ultrasonic_Sensor

**Status:** Draft \
**Subsystem:** Ultrasonic_Sensor actuation \
**Platform:** ESP32 / ESP-IDF

## 1. Purpose
`Ultrasonic_Sensor` represents and controls a single ultrasonic sensor. It provides a low-level interface that allows higher-level components, such as the robot controller, to command the sensor and retrieve information from it without directly interacting with its GPIO configurations.

## 2. Responsibilities
`Ultrasonic_Sensor` is responsible for:
1. Configuring the GPIO inputs used by the sensor.
2. Configuring and registering the GPIO interrupt.
3. Calculating valid echo signals.
4. Maintaining the echo signals and distances.
5. Calculating the distance when explicitly requested.
6. Recording the start and end times of echo signals.
7. Maintaining a safe initial sensor state.

## 3. Not Responsible For
`Ultrasonic_Sensor` is not responsible for:
1. Determining where the robot should move.
2. Deciding whether or not the movement is safe to perform.
3. Avoiding objects near the robot.
4. Calculating odometry.
5. Performing PID control.
6. Coordinating other motors.
7. Deciding when a motion command is completed.
8. Autonomous navigation.

## 4. Inputs
This component current accepts an ultrasonic sensor configuration that contains:
1. Ultrasonic sensor name
2. Trigger signal GPIO pin
3. Echo signal GPIO pin

## 5. Outputs
`Ultrasonic_Sensor` produces distance calculations through:
1. Interrupts triggered from triggering electrical echo signals.

`Ultrasonic_Sensor` exposes the calculation through `get_distance()` as well as calculating the distance the signal traveled through `measure_distance()`.

## 6. Initial State
A newly created `Ultrasonic_Sensor` must initialize its sensor to a safe state during its construction and before performing normal operation.

The expected initial sensor state is:
1. Distance = 0.

## 7. Valid Inputs
`Ultrasonic_Sensor::isr_handler(void *arg)` gets triggered when a hardware interrupt occurs. A valid hardware interrupt is:
- The sensor detecting a rising or falling edge.

The `Ultrasonic_Sensor` interface only represents the valid distance the object is from the robot. Higher-level components are the ones responsible for determining what the robot should do based on this distance.

## 8. Invariants
The following conditions should always hold:

### A. Valid Sensor Input
`Ultrasonic_Sensor` accepts a rising edge or falling edge hardware interrupt.

### B. Distance Calculation
A movement command must turn on the corresponding sensor to apply the distance calculation.

### C. Stop State
A stopped command must not use any ultrasonic sensor.

### D. New Movement
A new movement must turn on the corresponding sensor and use its distance calculation.

### E. Safe Initialization
`Ultrasonic_Sensor` may report itself initialized only after its GPIO configuration, sensor configuration, and ISR registration have all succeeded and its mathematical configuration is valid.

### F. Distance Calculation
Distance = (signal_duration x speed_of_sound) / 2

## 9. Failure Modes
Potential failure modes include:
- Invalid GPIO configuration.
- Hardware configuration API failure.

Each failure mode should eventually have a defined behavior.

## 10. Error Handling
The component currently calls the ESP-IDF configuration and sensor APIs directly.

The design needs to determine:
- Which hardware API failures must be checked.
- Which failures are recoverable.
- Which failures should prevent initialization.
- How initialization failure is communicated.
- How runtime sensor failure is communicated.

## 11. Timing Requirements
The component must eventually document whether its public operations are:
- Blocking or non-blocking
- Safe to call from an RTOS task
- Safe to call from an ISR
- Expected to complete within a bounded time

Current implementation does not have these formally specified.

## 12. Resource Constraints
The implementation should account for:
- RAM usage
- Flash/code size
- Stack usage
- Dynamic memory allocation
- CPU overhead
- Logging overhead

No optimization decision should be made without first identifying a relevant constraint or measurement.

## 13. Test Requirements
`Ultrasonic_Sensor` should be testable independently of the physical robot. At minimum, tests should cover:

### A. Initialization
- Successful initialization
- GPIO reset failure
- GPIO direction failure
- GPIO configuration failure
- ISR registration failure
- Initialization stops at first failure
- Failed sensor reports non-operational state

### B. Distance Measurements
- Initial distance is zero
- ISR measures object's distance from the robot
- Multiple distances are measured correctly

### C. Lockout/Invalid State
- Failed initialization prevents invalid sensor operation.

## 14. Design Goals
`Ultrasonic_Senso` should prioritize:
1. Predictable behavior
2. Safe sensor operation
3. Clear ownership of responsibilities
4. Testability
5. Minimal unnecessary complexity
6. Efficient use of embedded resources
7. A small and understandable public interface

The design should prefer simplicity unless additional abstraction provides a concrete engineering benefit.

## 15. Current Implementation Review
The current implementation provides:
- `Ultrasonic_Config` struct for GPIO configuration.
- GPIO pin initialization during construction.
- A stored `uint32_t` distance measurement.
- Interrupt Service Routine.
- A `const` distance measurement getter.
- ESP-IDF logging.
- Hardware API error handling.

Known design areas still required:
- Global GPIO/LEDC resource conflict detection.
- Configuration immutability.
- Logging metadata ownership.
- Timing guarantees.