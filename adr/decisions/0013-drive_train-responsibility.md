# Drive_Train Responsibility

## Context

Decide what the drive train component should be responsible for in the system architecture in order to manage lower-level components without coupling hardware details to higher-level logic (e.g., robot controller).

## Options

1. Create a `Drive_Train` class that owns 2 motor driver configurations and 2 wheel encoder configurations.
2. Manage the 2 motor driver configurations and 2 wheel encoder configurations directly though a higher-level controller.

## Decision

I chose `Option 1` that encapsulates 2 drivers' and 2 encoders' configurations to isolate its functionalities from the rest of the architecture.

## Rationale

The rest of the system does not need to know how to control the hardware for the 2 encoders or the 2 drivers. This promotes a cleaner hierarchy by making it easier to delegate operations to the drivers.

## Consequences

This allows the robot to add a 4-wheeled drive train and provide a higher-level control that doesn't rely on controlling the motors at the lower-level. Each `Drive_Train` instance maintains its own configuration and state, adding another level of per-instance storage. This cost is accepted in exchange for clearer ownership and encapsulation.