# Motor Duty Cycle

## Context

How should `Motor` handle a calculated PWM duty value that exceeds its supported 8-bit range?

## Options

1. Continue a previous valid input.
2. Stop the motor from moving.
3. Clamp the duty to `255`.
4. Enter a fault state.
5. Have the higher-level logic validate the value before passing it to the function.

## Decision

I chose `Option 5` where I use `uint8_t` for the `Motor::set_duty_cycle()` parameter and require the upstream component to validate and constrain calculated duty values before converting them to `uint8_t`.

## Rationale

I want `Motor` to handle valid duties with a type that matches its domain rather than using values that it cannot use. This makes the upstream layer responsible for producing the valid duty.

## Consequences

If the upstream layer does not perform this validation, then an out-of-range value may be implicitly converted to an 8-bit value before reaching `Motor` as the conversion itself is defined for unsigned integer types, which potentially results in an unintended PWM command.