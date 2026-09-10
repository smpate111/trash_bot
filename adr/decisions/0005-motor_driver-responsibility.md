# Motor_Driver Responsibility

## Context

Decide what the motor driver component should be responsible for in the system architecture in order to manage low-level motor hardware without coupling hardware details to higher-level movement logic (i.e., obstacle avoidance).

## Options

1. Create a `Motor_Driver` class that owns 2 motor hardware configurations (e.g., `Left Motor`, `Right Motor`).
2. Manage the 2 motor hardware configurations directly though a higher-level controller.

## Decision

I chose `Option 1` that encapsulates 2 motors' hardware configurations to isolate its functionalities from the rest of the architecture.

## Rationale

The rest of the system does not need to know how to control the hardware for 2 motors. This promotes a cleaner hierarchy by making it easier to delegate operations to the motors.

## Consequences

This allows the robot to add a 2-wheeled driver and provide a higher-level control that doesn't rely on controlling the motors at the lower-level. Each `Motor_Driver` instance maintains its own configuration and state, adding another level of per-instance storage. This cost is accepted in exchange for clearer ownership and encapsulation.