# Wheel_Encoder Synchronize Pulse Count

## Context

The `Wheel_Encoder` receives a pulse signal from a wheel encoder through a GPIO interrupt. The interrupt service routine (ISR) increments the encoder pulse count whenever an encoder pulse is detected.

The pulse count is also accessed from functions such as `get_pulse_count()`, `reset_count()`, and `calculate_distance()`. Therefore, the pulse counter is shared between the ISR and functions.

There needs to be a synchronization mechanism between these functions so that there is safe concurrent access without introducing any blocking behavior in the ISR.

## Requirements

The synchronization mechanism must:
- Allow the ISR to safely increment the pulse counter.
- Allow functions to safely read and reset the pulse counter.
- Avoid blocking in the ISR.
- Keep minimal implementation in the ISR.
- Avoid creating unnecessary synchronization overhead.
- Provide deterministic and understandable behavior.

## Options

1. Provide no synchronization. The ISR and functions would directly access the pulse counter like a normal integer.
2. Create a mutex to protect access to the pulse counter.
3. Change the pulse counter into an `atomic` counter to allow synchronization between the ISR and functions.

## Decision

I went with `Option 3` so that I could only change the behavior of the pulse counter.

## Rationale

`Option 1` would have caused problematic behavior with the pulse counter because no synchronization would create a unique scenario where if the pulse counter resets while the current movement command was just about to finish its execution, then the reset wouldn't get registered/recognized.

`Option 2` would not have worked because the ISR cannot safely perform a normal blocking synchronization. Additionally, mutexes are meant for task-to-task synchronization where a task can wait, which defeats the purpose of the ISR since it occurs from hardware interrupts that can occur anytime during the robot's operation.

`Option 3` uses atomic operations, such as relaxed operations, to meet the synchronization requirement because it ensures that updates to the pulse counter cannot be lost or partially observed.

## Consequences

Implementing `atomic` allows:
- The ISR to not perform blocking.
- Safely increment pulse counts.
- Safely read and reset the pulse counts in functions.
- Provide a synchronization mechanism that prevents problematic unsynchronized behaviors from occurring.
- Ensures the ISR remains small and simple.