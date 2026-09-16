# Motor_Driver

**Status:** Draft \
**Subsystem:** Motor_Driver actuation \
**Platform:** ESP32 / ESP-IDF

## 1. Purpose

`Motor_Driver` represents and controls 2 bidirectional DC motors. It provides a high-level interface that allows higher-level components, such as the drive train, to command the 2 motors without directly interacting with their GPIO and PWM configurations.

## 2. Responsibilities

`Motor_Driver` is responsible for:
1. Maintaining the 2 motors' commanded actuation states.
2. Applying the requested motor PWM duty to the 2 motors.
3. Driving the motors in the forward/backward direction.
4. Stopping the motors.
5. Maintainig a safe initial actuator state.
6. Applying only valid actuator commands represented by the `Motor_Driver` interface.
7. Coordinating 2 motors.
8. Ensuring the motor PWM duty is validated before passing it to the 2 motors.

## 3. Not Responsible For

`Motor_Driver` is not responsible for:
1. Determining where the robot should move.
2. Deciding whether or not the movement is safe to perform.
3. Detecting obstacles or avoiding them.
4. Reading wheel encoders.
5. Calculating wheel velocity.
6. Calculating odometry.
7. Performing PID control.
8. Coordinating other motors that it is not assigned to.
9. Deciding when a motion command is completed.
10. Autonomous navigation.

These responsibilities belong to other components.

## 4. Inputs

This component currently accepts a motor driver configuration that contains:
1. Motor driver name
2. Left Motor configuration
3. Right Motor configuration

It also accepts user-defined output values through `set_left_duty_cycle()` and `set_right_duty_cycle()` using `uint8_t` to represent the valid duty cycle range of `0-255`.

## 5. Outputs

`Motor_Driver` produces actuator signals through:
1. Direction/control outputs
2. PWM duty cycle outputs

`Motor_Driver` exposes the current commanded PWM duty cycles through `get_duty_cycles()`.

## 6. Initial State

A newly created `Motor_Driver` must initialize its actuators to a safe state during its construction and before performing normal operation.

The expected initial actuator state is:
1. Setting both motors' PWM duty to 0.
2. Ensuring both motors' control paths each receive zero duty cycles.

## 7. Valid Inputs

`Motor_Driver::set_duty_cycles()` accepts 2 `uint8_t` PWM duty cycle values. Valid duty cycle values are:
- `0-255` inclusive.

The `Motor_Driver` interface only represents validating and constraining actuator duty cycle values. Higher-level components that calculate duty values are the ones responsible for correcting duty values before converting them to `uint8_t`.

## 8. Invariants

The following conditions should always hold:

### A. Valid Actuator Output
`Motor_Driver` accepts only a PWM duty cycle representation within the supported hardware range of `0-255`.

### B. Forward Direction
A forward command must apply the commanded duty cycle to the forward control path and zero duty cycle to the reverse control path.

### C. Backward Direction
A backward command must apply the commanded duty cycle to the reverse control path and zero duty cycle to the forward control path.

### D. Turning Left
A left turn command must apply the commanded duty cycle to the forward control path of the Right Motor and to the reverse control path of the Left Motor while applying a zero duty cycle to the reverse control path of the Right Motor and to the forward control path of the Left Motor.

### E. Turning Right
A right turn command must apply the commanded duty cycle to the forward control path of the Left Motor and to the reverse control path of the Right Motor while applying a zero duty cycle to the reverse control path of the Left Motor and to the forward control path of the Right Motor.

### F. Stop State
A stopped command must apply a commanded zero duty cycle to both control paths for both motors.

### G. Safe Initialization
`Motor_Driver` must initialize with zero PWM duty on both control paths for both motors.

### H. State Meaning
The internally stored duty cycle value represents the commanded actuator output and not a measured physical motor velocity.

### I. Exclusive Movement
`Motor_Driver` must never command non-zero duty on both directional controls paths on both motors simultaneously.

## 9. Failure Modes
Potential failure modes include:
- Invalid GPIO configuration.
- Conflicting PWM channels.
- Invalid PWM configuration.
- Hardward configuration API failure.
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
`Motor_Driver` should be testable independently of the physical robot. At minimum, tests should cover:

### A. Normal Behavior
- Initial state
- Set valid duty cycles
- Get commanded duty cycles
- Forward operation
- Backward operation
- Left turn operation
- Right turn operation
- Stop operation

### B. Boundary Behavior
- Minimum valid duty cycle.
- Maximum valid duty cycle.
- Zero duty cycle.
- Independent left/right duty-cycle commands.
- Equal left/right duty cycles.

### C. State Transitions
- `Forward -> Forward`
- `Forward -> Backward`
- `Forward -> Left Turn`
- `Forward -> Right Turn`
- `Forward -> Stop`
- `Backward -> Forward`
- `Backward -> Backward`
- `Backward -> Left Turn`
- `Backward -> Right Turn`
- `Backward -> Stop`
- `Left Turn -> Forward`
- `Left Turn -> Backward`
- `Left Turn -> Left Turn`
- `Left Turn -> Right Turn`
- `Left Turn -> Stop`
- `Right Turn -> Forward`
- `Right Turn -> Backward`
- `Right Turn -> Left Turn`
- `Right Turn -> Right Turn`
- `Right Turn -> Stop`
- `Stop -> Forward`
- `Stop -> Backward`
- `Stop -> Left Turn`
- `Stop -> Right Turn`
- `Stop -> Stop`

### D. Direction Switch Maintains Exclusivity
- `Forward @ 200:`
    - `-> Backward @ 200`
    - `Verify Left Motor channel 1 = 0`
    - `Verify Left Motor channel 2 = 200`
    - `Verify Right Motor channel 1 = 0`
    - `Verify Right Motor channel 2 = 200`
- `Backward @ 200:`
    - `-> Forward @ 200`
    - `Verify Left Motor channel 1 = 200`
    - `Verify Left Motor channel 2 = 0`
    - `Verify Right Motor channel 1 = 200`
    - `Verify Right Motor channel 2 = 0`

### E. Fault Behavior
- Invalid configuration
- Hardware API failure

### F. Safety Behavior
- `Motor_Driver` initializes in a stopped state.
- Invalid input cannot create an unsafe actuator state.
- Stop produces the expected electrical output.

## 14. Design Goals
`Motor_Driver` should prioritize:
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
- `Motor_Driver_Config` struct for Left Motor and Right Motor configuration.
- GPIO and LEDC PWM channel initialization during construction.
- Stored `uint8_t` PWM duty cycle values.
- Forward, backward, left turn, right turn, and stop commands.
- A `const` duty cycle getter.
- ESP-IDF logging.
- Hardware API error handling.
- PWM duty cycle value validation.

Known design areas still required:
- Global GPIO/LEDC resource conflict detection.
- Configuration immutability.
- Logging metadata ownership.
- Timing guarantees.