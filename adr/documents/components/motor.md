# Motor

**Status:** Draft \
**Subsystem:** Motor actuation \
**Platform:** ESP32 / ESP-IDF

## 1. Purpose

`Motor` represents and controls a singular bidirectional DC motor connected through a motor driver. This components translates a requested motor command into the electrical outputs required by the motor driver. It is intended to provide a simple interface for higher-level components (e.g., motor driver and drive train) to manage and control.

## 2. Responsibilities:

`Motor` is responsible for:
1. Configuring the GPIO/PWM resources required to control one motor.
2. Maintain the motor's commanded actuation state.
3. Apply the requested motor speed.
4. Drive the motor in the forward/backward direction.
5. Stop the motor.
6. Maintain a safe initial actuator state.
7. Prevent invalid actuator commands from producing an invalid hardware state.

## 3. Not responsible for:

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

It also accepts a user-defined output value through `set_speed()`.

**Decision:** TBD.

## 5. Outputs

The component produces electrical actuator signals through:
1. Direction/control outputs
2. PWM duty-cycle outputs

The component may also expose the current commanded actuator state to software.

## 6. Initial State

A newly created `Motor` should enter a safe state before normal operation during its object construction.

The expected initial actuator state is:
1. No motor rotation commanded.
2. PWM duty = 0.
3. Both direction outputs represent the stopped state.

## 7. Valid Inputs

This component must define the valid range for its actuator command.

Questions to resolve:
- What is the valid PWM range?
- Is the maximum determined by the LEDC configuration?
- Should `Motor` reject or clamp values above the maximum?
- Should invalid input be reported to the caller?

**Decision:** TBD.

## 8. Invariants

The following conditions should always hold:

### A. Valid actuator output
`Motor` must never intentionally command a PWM value outside the supported hardware range.

### B. Forward direction
A forward command must only output the configured forward control path.

### C. Backward direction
A backward command must only output the configured backward control path.

### D. Stop state
A stopped motor must command zero output.

### E. Safe initialization
`Motor` must not begin operation with a non-zero commanded output.

### F. State meaning
The internally stored motor value represents the commanded actuator output and not measured physical motor velocity.

## 9. Failure Modes
Potential failure modes include:
- Invalid GPIO configuration
- Conflicting PWM channels
- Invalid PWM configuration
- Hardware configuration API failure
- Invalid speed/output request
- Unexpected direction transitions
- Hardware becoming unavailable after initialization
- Software reporting one state while hardware is actually in another state

Each failure mode should eventually have a defined behavior.

**Decision:** TBD.

## 10. Error Handling
The component currently calls the ESP-IDF configuration and actuator APIs directly.

The design needs to determine:
- Which hardware API failures must be checked.
- Which failures are recoverable.
- Which failures should prevent initialization.
- How initialization failure is communicated.
- How runtime actuator failure is communicated.

**Decision:** TBD.

## 11. Timing Requirements
The component must eventual document whether its public operations are:
- Blocking or non-blocking
- Safe to call from an RTOS task
- Safe to call from an ISR
- Expected to complete within a bounded time

Current implementation does not have these formally specified.

**Decision:** TBD.

## 12. Resource Constraints
The implementation should account for:
- RAM usage
- Flash/code size
- Stack usage
- Dynamic memory allocation
- CPU overhead
- Logging overhead

No optimization decision should be made without first identifying a relevant constraint or measurement.

**Decision:** TBD.

## 13. Test Requirements
`Motor` should be testable independently of the physical robot. At minimum, tests should cover:

### Normal behavior
- Initial state
- Set valid speed
- Get commanded speed
- Forward operation
- Backward operation
- Stop operation

### Boundary behavior
- Minimum valid output
- Maximum valid output
- Zero output
- Values immediately outside the valid range

### State transitions
- Forward -> Backward
- Backward -> Forward
- Forward -> Stop
- Backward -> Stop
- Repeated Stop
- Repeated direction commands

### Fault behavior
- Invalid configuration
- Invalid actuator command
- Hardware API failure

### Safety behavior
- Motor intializes stopped
- Invalid input cannot create an unsafe actuator state
- Stop produces the expected electrical output

**Decision:** TBD.

## 14. Design Goals
`Motor` should prioritize:
1. Predictable behavior
2. Safe actuator operation
3. Clear ownership of responsibilities
4. Testability
5. Minimal unnecessary complexity
6. Efficient use of embedded resources
7. A small and understandable public interface.

The design should prefer simplicity unless additional abstraction provides a concrete engineering benefit.

## 15. Current Implementation Review
The current implementation provides:
- A motor configuration struct during construction.
- GPIO configuration and LEDC PWM configuration during construction.
- Stored commanded speed.
- Forward/backward/stop operations.
- Logging through ESP-IDF.

Areas requiring design investigation before refactoring:
- Meaning and naming of `speed`.
- Valid actuator range.
- Error handling for ESP-IDF calls.
- Necessity of `virtual` functions.
- Constructor responsibilities.
- Header dependencies.
- Ownership and lifetime of configuration data.
- Logging responsibility.
- Public API surface.
- Separation between actuator command and measured physical behavior.