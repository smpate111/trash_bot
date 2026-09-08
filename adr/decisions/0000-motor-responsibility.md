# Motor Responsibility

## Context

Decide what the motor component should be responsible for in the system architecture in order to manage low-level motor hardware without coupling hardware details to higher-level movement logic (i.e., obstacle avoidance).

## Options

1. Create a `Motor` class that owns its own hardware configurations (e.g., GPIO pins, PWM channels) and functions.
2. Manage the hardware configurations directly through a higher-level controller.

## Decision

I chose `Option 1` that encapsulates a motor's hardware configurations to isolate its functionalities from the rest of the architecture.

## Rationale

The rest of the system does not need to know the GPIO pins or PWM channels the motor is assigned to. It also promotes a cleaner hierarchy by making it easier to delegate operations and remap configurations.

## Consequences

This makes it easier to create mock hardware to test because I can independently verify the logic without relying on higher-level controls. However, there will be a bigger memory footprint due to maintaining multiple class instances on the stack/heap.