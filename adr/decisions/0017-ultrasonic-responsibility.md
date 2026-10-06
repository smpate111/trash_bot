# Ultrasonic_Sensor Responsibility

## Context

Decide what the ultrasonic sensor component should be responsible for in the system architecture in order to manage low-level sensor hardware without coupling hardware details to higher-level movement logic (i.e., obstacle avoidance).

## Options

1. Create a `Ultrasonic_Sensor` class that owns its own hardware configurations (e.g., GPIO pins, PWM channels).
2. Manage the hardware configurations directly through a higher-level controller.

## Decision

I chose `Option 1` that encapsulates the ultrasonic sensor's hardware configurations to isolate its functionalities from the rest of the architecture.

## Rationale

The rest of the system does not need to know the GPIO pins the ultrasonic sensor is assigned to. This promotes a cleaner hierarchy by making it easier to delegate operations and remap configurations.

## Consequences

This makes it easier to create mock hardware to test because I can independently verify the logic without relying on higher-level controls. Each `Ultrasonic_Sensor` instance maintains its own configuration and state. This introduces per-instance storage, but the cost is accepted in exchange for clearer ownership and encapsulation.