# Front_Right_Motor GPIO Assignment and Wheel Encoder Pulse Discrepancy

## Context
During forward-motion testing, the left and right wheel encoders consistently produced different pulse counts depsite both the left and right motors being given the same PWM output and movement duration.

The initial configuration showed a larger discrepancy between the 2 encoders' pulse counts where the right wheel encoder commonly reported an additional 4-5 pulses.

This raised the possibility that the configuration of the front-right motor was affecting the right wheel encoder's measurement.

At first, the following components had the initial GPIO Pin configurations:
- `Front Right Motor:`
    - `GPIO PIN 13`
    - `GPIO PIN 14`
- `Left Wheel Encoder:`
    - `GPIO PIN 1`
    - `Wheel Diameter: 80mm`
    - `Encoder Slots: 20`
- `Right Wheel Encoder:`
    - `GPIO PIN 41`
    - `Wheel Diameter: 80mm`
    - `Encoder Slots: 20`

The forward-motion tests were performed with the following steps:
1. Reset both wheel encoders' pulse counters.
2. Command both motors at the same PWM output.
3. Run the forward-motion for 2 seconds.
4. Stop the motors.
5. Record the encoders' pulse counts.

## Observation
The initial configuration revealed a difference between the 2 pulse counters where the right wheel encoder's pulse count reported several additional pulses than the left wheel encoder. Since this discrepancy was repeatable during this test, the front-right motor GPIO pin assignment was investigated as a possible source.

## Change
The front-right motor GPIO pin assignment was changed to the following:
- `GPIO PIN 8`
- `GPIO PIN 3`

The wheel encoders' GPIO pin assignments were not changed.

## Results
After repeating the same test but with a new GPIO pin assignment for the front-right motor, the discrepancy reduced to between -1 and +2 pulses, which supports the hypothesis of GPIO pin assignment potentially being the cause of the discrepancy. However, this doesn't necessarily prove that the original GPIO pin assignment was the direct cause of the discrepancy as there could have been eletrical interference, differences in signal routing, or differences in electrical behavior in the motor that could be the reasons behind this issue.