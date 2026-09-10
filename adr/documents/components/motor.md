# Motor

**Status:** Draft \
**Subsystem:** Motor actuation \
**Platform:** ESP32 / ESP-IDF

## 1. Purpose

`Motor` represents and controls a single bidirectional DC motor connected through a motor driver. It provides a low-level interface that allows higher-level components, such as the motor driver and drive train, to command the motor without directly interacting with its GPIO and PWM configurations.

## 2. Responsibilities

`Motor` is responsible for:
1. Configure the GPIO/PWM resources required to control one motor.
2. Maintain the motor's commanded actuation state.
3. Apply the requested motor PWM duty.
4. Drive the motor in the forward/backward direction.
5. Stop the motor.
6. Maintain a safe initial actuator state.
7. Apply only valid actuator commands represented by the `Motor` interface.

## 3. Not responsible for

`Motor` is not responsible for:
1. Determining where the robot should move.
2. Deciding whether or not the movement is safe to perform.
3. Detecting obstacles or avoiding them.
4. Reading wheel encoders.
5. Calculating wheel velocity.
6. Calculating odometry.
7. Performing PID control.
8. Coordinating other motors.
9. Deciding when a motion command is completed.
10. Autonomous navigation.

These responsibilities belong to other components.

## 4. Inputs

This component currently accepts a motor configuration that contains:
1. Motor name
2. Direction-controlling GPIO pins
3. PWM channels

It also accepts a user-defined output value through `set_duty_cycle()` using `uint8_t` to represent the valid duty cycle range of `0-255`.

## 5. Outputs

`Motor` produces electrical actuator signals through:
1. Direction/control outputs
2. PWM duty cycle outputs

`Motor` exposes the current commanded PWM duty cycle through `get_duty_cycle()`.

## 6. Initial State

A newly created `Motor` must initialize its actuator to a safe state during its construction and before performing normal operation.

The expected initial actuator state is:
1. PWM duty = 0.
2. Both control paths receive zero duty cycles.

## 7. Valid Inputs

`Motor::set_duty_cycle()` accepts an `uint8_t` PWM duty cycle value. Valid duty cycle values are:
- `0-255` inclusive.

The `Motor` interface only represents valid actuator duty cycle values. Higher-level components that calculate duty values are the ones responsible for validating and constraining those values before converting them to `uint8_t`.

## 8. Invariants

The following conditions should always hold:

### A. Valid actuator output
`Motor` accepts only a PWM duty cycle representation within the supported hardware range of `0-255`.

### B. Forward direction
A forward command must apply the commanded duty cycle to the forward control path and zero duty cycle to the reverse control path.

### C. Backward direction
A backward command must apply the commanded duty cycle to the reverse control path and zero duty cycle to the forward control path.

### D. Stop state
A stopped command must apply a commanded zero duty cycle to both control paths.

### E. Safe initialization
`Motor` must initialize with zero PWM duty on both control paths.

### F. State meaning
The internally stored duty cycle value represents the commanded actuator output and not a measured physical motor velocity.

### G. Exclusive movement
`Motor` must never command non-zero duty on both directional control paths simultaneously.

## 9. Failure Modes
Potential failure modes include:
- Invalid GPIO configuration.
- Conflicting PWM channels.
- Invalid PWM configuration.
- Hardware configuration API failure.
- Invalid duty cycle command by an upstream component before its conversion to `uint8_t`.
- Unexpected direction transitions or invalid control path state.

Each failure mode should eventually have a defined behavior.

## 10. Error Handling
The component currently calls the ESP-IDF configuration and actuator APIs directly.

The design needs to determine:
- Which hardware API failures must be checked.
- Which failures are recoverable.
- Which failures should prevent initialization.
- How initialization failure is communicated.
- How runtime actuator failure is communicated.

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
`Motor` should be testable independently of the physical robot. At minimum, tests should cover:

### A. Normal behavior
- Initial state
- Set valid duty cycle
- Get commanded duty cycle
- Forward operation
- Backward operation
- Stop operation

### B. Boundary behavior
- Minimum valid output
- Maximum valid output
- Zero output

### C. State transitions
- `Forward -> Forward`
- `Forward -> Backward`
- `Forward -> Stop`
- `Backward -> Forward`
- `Backward -> Backward`
- `Backward -> Stop`
- `Stop -> Forward`
- `Stop -> Backward`
- `Stop -> Stop`

### D. Direction switch maintains exclusivity
- `Forward @ 200:`
    - `-> Backward @ 200`
    - `Verify channel 1 = 0`
    - `Verify channel 2 = 200`
- `Backward @ 200:`
    - `-> Forward @ 200`
    - `Verify channel 1 = 200`
    - `Verify channel 2 = 0`

### E. Fault behavior
- Invalid configuration
- Hardware API failure

### F. Safety behavior
- `Motor` intializes in a stopped state.
- Invalid input cannot create an unsafe actuator state.
- Stop produces the expected electrical output.

## 14. Design Goals
`Motor` should prioritize:
1. Predictable behavior
2. Safe actuator operation
3. Clear ownership of responsibilities
4. Testability
5. Minimal unnecessary complexity
6. Efficient use of embedded resources
7. A small and understandable public interface

The design should prefer simplicity unless additional abstraction provides a concrete engineering benefit.

## 15. Current Implementation Review
The current implementation provides:
- `Motor_Config` struct for GPIO and LEDC channel configuration.
- GPIO and LEDC PWM channel initialization during construction.
- A stored `uint8_t` PWM duty cycle value.
- Forward, backward, and stop commands.
- A `const` duty cycle getter.
- ESP-IDF logging.
- Hardware API error handling.

Known design areas still required:
- Global GPIO/LEDC resource conflict detection.
- Configuration immutability.
- Logging metadata ownership.
- Timing guarantees.