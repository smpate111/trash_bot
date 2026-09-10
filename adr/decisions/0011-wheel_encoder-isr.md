# Wheel_Encoder ISR

## Context

How should `Wheel_Encoder` count the number of encoders that passes through it while the wheel is traveling an arbitrary distance?

## Options

1. Create a formula that counts the number by using the PWM duty value, wheel diameter, and the time it traveled.
2. Add `1` to the counter each time the sensor's laser breaks.

## Decision

I chose `Option 2` where I use an interrupt service routine that has the counter insider it.

## Rationale

I want `Wheel_Encoder` to know when to increment the counter via the wheel encoder instead of relying on complex math to estimate the count. This allows me to easily control the conditions needed for the counter to increment.

## Consequences

This allows the upstream layer to use this count in their calculations on determining the distance the wheel traveled based on any given movement.