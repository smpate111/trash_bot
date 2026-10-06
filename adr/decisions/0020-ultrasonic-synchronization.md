# Ultrasonic_Sensor Synchronize Distance Calculation

## Context

The `Ultrasonic_Sensor` sends and receives an echo signal from the ultrasonic sensor through a GPIO interrupt. The ISR records the time the signal leaves the sensor and when it returns to calculate the distance traveled (or the distance of the object it detected from the robot) whenever the signal gets triggered.

The distance is also accessed from functions such as `measure_distance()` and `get_distance()`. Therefore, the distance is shared between the ISR and functions.

There needs to be a synchronization mechanism between these functions so that there is safe concurrent access without introducing any blocking behavior in the ISR.

## Requirements

The synchronization mechanism must:
- Allow the ISR to safely record the echo signal's start and end times.
- Allow the ISR to safely calculate the distance of the detected object from the robot.
- Allow functions to safely read the distance.
- Avoid blocking in the ISR.
- Keep minimal implementation in the ISR.
- Avoid creating unnecessary synchronization overhead.
- Provide deterministic and understandable behavior.

## Options

1. Provide no synchronization where the ISR and function would directly access the distance like a normal variable.
2. Create a mutex to protect access to the distance.
3. Change the distance to an `atomic` variable to allow synchronization between the ISR and functions.

## Decision

I went with `Option 3` so that I could only change the behavior of the distance.

## Rationale

`Option 1` would have caused problematic behavior with the distance because no synchronization would create a unique scenario where if the distance updates while the current movement command was just about to finish its execution, then the update wouldn't get registered/recognized.

`Option 2` would not have worked because the ISR cannot safely perform a normal blocking synchronization. Additionally, mutexes are meant for task-to-task synchronization where a task can wait, which defeats the purpose of the ISR since it occurs from hardware interrupts that can occur anytime during the robot's operation.

`Option 3` uses atomic operations, such as relaxed operations, to meet the synchronization requirement because it ensures that updates to the distance cannot be lost or partially observed.

## Consequences

Implementing `atomic` allows:
- The ISR to not perform blocking.
- Safely calculate distances.
- Safely read the distance in functions.
- Provide a synchronization mechanism that prevents problematic unsynchronized behaviors from occurring.
- Ensures the ISR remains small and simple.