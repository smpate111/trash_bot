# Drive_Train

**Status:** Draft \
**Subsystem:** Driove system coordination & wheel-motion measurement \
**Platform:** ESP32 / ESP-IDF

## 1. Purpose

The robot contains 4 independently driven motors arranged as left and right wheel groups. The 4 motors are controlled through 2 `Motor_Driver` objects with one driver assigned to the front axle and one driver assigned to the rear axle.

The robot also contains 2 `Wheel_Encoder` objects that provide wheel-motion measurements for the left and right sides of the robot.

The `Drive_Train` class provides the robot-level interface for coordinating the front and rear motor drivers and consuming left and right wheel encoder measurements. It is the mobility coordination layer between individual actuator drivers, wheel-motion sensors, and higher-level controller logic.

## 2. Responsibilities

`Drive_Train` is responsible for coordinating the robot's 4-wheel drive system and conducting motion commands. Its responsibilities are:
- Coordinate the front and rear `Motor_Driver` objects.
- Apply `left` and `right` PWM duty cycles to both motor drivers.
- Coordinate the direction of the 4 motors to produce forward, backward, left turn, right turn, and stop movements.
- Own the relationship between the `left` and `right` wheel groups.
- Calculate left and right wheel encoder measurements.
- Define measurement intervals associated with movement.
- Provide wheel-motion information that is required by higher-level controllers.

`Drive_Train` coordinates these operations but does not reimplement the hardware behavior owned by lower-level components.

## 3. Not Responsible For
`Drive_Train` is not responsible for:
1. Configuring individual motor GPIO pins or their PWM channels.
2. Implementing motor direction control at the hardware level.
3. Configuring wheel encoder GPIO pins or their interrupts.
4. Synchronizing the wheel encoder pulse counters.
5. Determining whether a commanded movement is safe.
6. Detecting obstacles.
7. Generating trajectories.
8. Performing PID control.
9. Performing a closed-loop velocity control.
10. Performing autonomous navigation.
11. Performing localization.
12. Performing SLAM.
13. Calculating robot poses unless a future design explicitly assigns that responsibility to this component.
14. Creating, owning, or scheduling FreeRTOS tasks.

These responsbilities belong to either the lower-level components or higher-level controllers.

## 4. Inputs
`Drive_Train` accepts a `Train_Config` containing:
1. Drive train name.
2. Front `Motor_Driver`.
3. Rear `Motor_Driver`.
4. Reference to the left `Wheel_Encoder` object.
5. Reference to the right `Wheel_Encoder` object.

The current configuration stores the motor drivers as components of the `Drive_Train` while maintaining references to externally created encoder objects. The `Drive_Train` also accepts runtime movement commands through its public interface. These include:
- Left wheel-group PWM duty cycle.
- Right wheel-group PWM duty cycle.
- Forward movement.
- Backward movement.
- Left turn.
- Right turn.
- Stop.

PWM duty-cycle values are represented using `uint8_t` that corresponds to the supported `0-255` duty-cycle range.

## 5. Outputs
`Drive_Train` produces coordinated acutator commands through the 2 `Motor_Driver` objects. The resulting outputs are:
1. Left-side motor acutation.
2. Right-side motor actuation.
3. Motor direction commands.
4. PWM duty-cycle commands.

Additionally, `Drive_Train` also exposes wheel-motion information derived from the 2 `Wheel_Encoder` objects that includes:
1. Left wheel pulse counting.
2. Right wheel pulse counting.
3. Left wheel distance calculations.
4. Right wheel distance calculations.

The current implementation reads the encoder pulse counts and calculates the wheel travel distance.

## 6. Initial State
The `Drive_Train` must begin in the initial state with the following expectations:
1. The configured front and rear motor drivers are already successfully initialized.
2. The configured left and right wheel encoders are already successfully initialized.
3. No movement commands are intially issued by the `Drive_Train`.
4. The `Drive_Train` is considered stopped until a movement command is issued.
5. Wheel-motion measurement is initialized to a known state.

## 7. Valid Inputs
### A. PWM Duty Cycle
The `Drive_Train` speed interface accepts `uint8_t` with a valid range of `0-255`. The value represents the commanded PWM duty cycle rather than measured wheel velocity.

### B. Measurement Commands
The supported movement commands are:
- Forward
- Backward
- Left Turn
- Right Turn
- Stop

### C. Encoder Measurements
The drive train accepts wheel-motion measurements from the left and right `Wheel_Encoder` objects. It does not directly modify the encoders' pulse counters.

## 8. Invariants
The following conditions should always hold:

### A. Coordinate Actuations
A `Drive_Train` movement command must coordinate both the front and rear motor drivers so that all 4 motors participate in the requested movement.

### B. Forward Movement
A forward command must command the following at the requested duty cycles:
- `Left Motors -> Forward`
- `Right Motors -> Forward`

### C. Backward Movement
A backward command must command the following at the requested duty cycles:
- `Left Motors -> Backward`
- `Right Motors -> Backward`

### D. Left Turn
A left turn command must command the following at the requested duty cycles:
- `Left Motors -> Backward`
- `Right Motors -> Forward`

### E. Right Turn
A right turn command must command the following at the requested duty cycles:
- `Left Motors -> Forward`
- `Right Motors -> Backward`

### F. Stop
A stop command must command both motor drivers to stop all 4 motors.

### G. Encapsulate Encoders
`Drive_Train` must access the encoder's measurements only through its public interface. It must not directly access the encoder's internal pulse counter or interrupt state.

### H. Measurement Calculations
`Drive_Train` owns the definition of when a wheel-motion measurement interval starts and ends. The `Wheel_Encoder` owns the internal pulse-counting mechanism.

### I. State Meaning
The stored PWM duty-cycle values represent the commanded actuator outputs and not the measured wheel velocity.

### J. Left/Right Wheel Group Relationship
The left-side commands must affect both left motors while the right-side commands must affect both right motors.

## 9. Failure Modes
Potential failure modes include:
1. Invalid front motor driver configuration.
2. Invalid rear motor driver configuration.
3. Invalid wheel encoder configuration.
4. A referenced wheel encoder object becoming invalid during the `Drive_Train`'s lifetime.
5. Hardware initialization failure in a dependent component.
6. Invalid or unexpected acutator command state.
7. Wheel encoder failure or missing encoder pulses.
8. Wheel encoder signal noise or unexpected pulses.
9. Left/right wheel measurement mismatch.
10. Failure to detect expected wheel motion after a movement command.

Each failure mode should eventually have an explicitly defined response.

## 10. Error Handling
`Drive_Train` depends on the initialization and runtime behavior of configured `Motor_Driver` and `Wheel_Encoder` objects. The design  must define:
1. How dependent-configuration initialization failures are reported.
2. Whether constructing `Drive_Train` is allowed when a dependency is not initialized.
3. How missing or invalid encoder measurements are handled.
4. How unexpected actuator states are handled.
5. How runtime failures are communicated to higher-level controllers.
6. Whether a detected drive train fault should transition the system to a stopped state.

The `Drive_Train` should not hide hardware or sensor failures from higher-level components.

## 11. Timing Requirements
The `Drive_Train` provides command-oriented operations and measurement processing. Public movement commands should be:
- Non-blocking.
- Intended to be called from RTOS tasks.
- Not intended for ISR use.

The exact timing requirements for future velocity and closed-loop control should be defined when those features are introduced. `Drive_Train` should not assume that a command completing means that the physical motion has completed.

## 12. Resource Constraints
The implementation should account for:
- RAM usage
- Flash/code size
- Stack usage
- Dynamic memory allocation
- CPU overhead
- Logging overhead
- Encoder-processing frequency

No optimization decision should be made without first identifying a relevant constraint or measurement.

## 13. Test Requirements
`Drive_Train` should be testable independently of the physical robot. At minimum, tests should cover:

### A. Normal Behavior
- Valid drive train configuration.
- Front motor driver dependency.
- Rear motor driver dependency.
- Left encoder dependency.
- Right encoder dependency.
- Initial stopped state.
- Forward operation.
- Backward operation.
- Left turn operation.
- Right turn operation.
- Stop operation.

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
    - `Verify Front Left Motor channel 1 = 0`
    - `Verify Front Left Motor channel 2 = 200`
    - `Verify Front Right Motor channel 1 = 0`
    - `Verify Front Right Motor channel 2 = 200`
    - `Verify Back Left Motor channel 1 = 0`
    - `Verify Back Left Motor channel 2 = 200`
    - `Verify Back Right Motor channel 1 = 0`
    - `Verify Back Right Motor channel 2 = 200`
- `Backward @ 200:`
    - `-> Forward @ 200`
    - `Verify Front Left Motor channel 1 = 200`
    - `Verify Front Left Motor channel 2 = 0`
    - `Verify Front Right Motor channel 1 = 200`
    - `Verify Front Right Motor channel 2 = 0`
    - `Verify Back Left Motor channel 1 = 200`
    - `Verify Back Left Motor channel 2 = 0`
    - `Verify Back Right Motor channel 1 = 200`
    - `Verify Back Right Motor channel 2 = 0`

### E. Wheel Encoder Measurement
- Read the left wheel encoder pulse count.
- Read the right wheel encoder pulse count.
- Calculate the left wheel distance.
- Calculate the right wheel distance.
- Detect changing wheel encoder measurements.

### F. Dependency Failure Behavior
- Uninitialized motor driver.
- Uninitialized wheel encoder.
- Invalid dependent configuration.
- Missing wheel encoder measurement.
- Invalid runtime state.

### G. Safety Behavior
- `Drive_Train` starts in a stopped state.
- Stop commands all 4 motors to a stopped state.
- Invalid actuator commands cannot create an unsafe output state.
- Wheel encoder synchronization remains encapsulated in `Wheel_Encoder`.

## 14. Design Goals
`Drive_Train` should prioritize:
1. Predictable coordinated motion.
2. Clear separation from lower-level hardware implementation.
3. Safe actuator operation.
4. Clear ownership of wheel measurements.
5. Testability.
6. Minimal unnecessary complexity.
7. A small and understandable public interface.

The design should prefer simplicity unless additional abstraction provides a concrete engineering benefit.

## 15. Current Implementation Review
The current implementation provides:
- `Train_Config` struct for front and rear motor drivers and left/right wheel encoder references.
- Front and rear motor driver coordination.
- Forward movement, backward movement, left turn, right turn, and stop movement.
- Wheel encoder pulse count access.
- Wheel-motion distance calculations.
- ESP-IDF logging.
- Hardware API error handling.
- PWM duty cycle value validation.

Known design areas still required:
- Global GPIO/LEDC resource conflict detection.
- Configuration immutability.
- Logging metadata ownership.
- Timing guarantees.