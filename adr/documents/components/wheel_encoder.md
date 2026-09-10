# Wheel_Encoder

**Status:** Draft \
**Subsystem:** Wheel_Encoder actuation \
**Platform:** ESP32 / ESP-IDF

## 1. Purpose

`Wheel_Encoder` represents and controls a single wheel encoder sensor. It provides a low-level interface that allows higher-level components, such as the drive train, to command the sensor and retrieve information from it without directly interacting with its GPIO configurations.

## 2. Responsibilities

`Wheel_Encoder` is responsible for:
1. Configuring the GPIO input used by the encoder.
2. Configuring and registering the GPIO interrupt.
3. Counting valid encoder pulses.
4. Maintaining the pulse count.
5. Resetting the pulse count when explicitly requested.
6. Calculating wheel travel distance from pulse count, wheel diameter, and encoder resolution.
7. Maintaining a safe initial sensor state.

## 3. Not responsible for

`Wheel_Encoder` is not responsible for:
1. Determining where the robot should move.
2. Deciding whether or not the movement is safe to perform.
3. Detecting obstacles or avoiding them.
4. Calculating odometry.
5. Performing PID control.
6. Coordinating other motors.
7. Deciding when a motion command is completed.
8. Autonomous navigation.

These responsibilities belong to other components.

## 4. Inputs

This component currently accepts a wheel encoder configuration that contains:
1. Wheel encoder name
2. Pulse-counting GPIO pin
3. The wheel's diameter
4. The number of encoder slots

## 5. Outputs

`Wheel_Encoder` produces wheel encoder counts through:
1. Interrupts triggered from cutting electrical sensor signals.

`Wheel_Encoder` exposes the count through `get_pulse_count()` as well as calculating the distance the wheel traveled through `calculate_distance()`.

## 6. Initial State

A newly created `Wheel_Encoder` must initialize its sensor to a safe state during its construction and before performing normal operation.

The expected initial sensor state is:
1. Pulse count = 0.

## 7. Valid Inputs

`Wheel_Encoder::isr_handler(void *arg)` gets triggered when a hardware interrupt occurs. A valid hardware interrupt is:
- The sensor detecting a falling edge.

The `Wheel_Encoder` interface only represents the valid movement distance traveled by the robot. Higher-level components are the ones responsible for providing valid movement commands.

## 8. Invariants

The following conditions should always hold:

### A. Valid sensor output
`Wheel_Encoder` accepts a falling edge hardware interrupt.

### B. Encoder counter
A movement command must apply the increment counter.

### C. Stop state
A stopped command must reset the increment counter.

### D. New movement
A new movement command must reset the increment counter.

### E. Safe initialization
`Wheel_Encoder` may report itself initialized only after its GPIO configuration, sensor configuration, and ISR registration have all succeeded and its mathematical configuration is valid.

### F. Distance calculation
Distance = pulse_count × wheel circumference / encoder_slots

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
`Wheel_Encoder` should be testable independently of the physical robot. At minimum, tests should cover:

### A. Initialization
- Successful initialization
- GPIO reset failure
- GPIO direction failure
- GPIO configuration failure
- ISR registration failure
- Initialization stops at first failure
- Failed encoder reports non-operational state

### B. Pulse counting
- Initial count is zero
- ISR increments count
- Multiple pulses increment correctly
- Counter reset

### C. Distance
- Zero pulses
- One revolution
- Multiple revolutions
- Fractional revolution

### D. Lockout/Invalid state
- Failed initialization prevents invalid sensor operation.

## 14. Design Goals
`Wheel_Encoder` should prioritize:
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
- `Encoder_Config` struct for GPIO configuration.
- GPIO pin initialization during construction.
- A stored `uint32_t` pulse counter.
- Interrupt Service Routine.
- A `const` pulse counter getter.
- ESP-IDF logging.
- Hardware API error handling.

Known design areas still required:
- Global GPIO/LEDC resource conflict detection.
- Configuration immutability.
- Logging metadata ownership.
- Timing guarantees.