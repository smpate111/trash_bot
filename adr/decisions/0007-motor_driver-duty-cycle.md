# Motor_Driver Duty Cycle

## Context

How should `Motor_Driver` handle a calculated PWM duty value that exceeds its supported 8-bit range?

## Options

1. Continue a previous valid input.
2. Stop the motor from moving.
3. Clamp the duty to `255`.
4. Enter a fault state.
5. Have the higher-level logic validate the value before passing it to the function.

## Decision

I chose `Option 3` and `Option 5` where I use `uint8_t` for the `Motor_Driver::set_duty_cycles()` parameter and convert the value to `255` while requiring the upstream component to constrain calculated duty values before converting them to `uint8_t`.

## Rationale

I want `Motor_Driver` to handle valid duties with a type that matches its domain rather than using values that it cannot use. This makes the upstream layer responsible for constraining the valid duty.

## Consequences

If the upstream layer does not perform this constrainment, then an out-of-range value may be to an 8-bit value before reaching `Motor_Driver` as the conversion itself validates the value and is able to be sent to `Motor`.