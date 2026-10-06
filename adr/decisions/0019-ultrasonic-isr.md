# Ultrasonic_Sensor ISR

## Context

How should `Ultrasonic_Sensor` calculate the distance of detected objects while the robot is moving?

## Options

1. Create a formula that calculates the duration the sonic signal took to leave the sensor and return to it to determine the object's distance from the robot.

## Decision

I chose `Option 1` where I use an interrupt service routine that records the signal's time and then calculates distance when the signal reaches the sensor.

## Rationale

I want `Ultrasonic_Sensor` to know when to calculate the distance when it sends and receives signals instead of relying on complex math to estimate the distance. This allows me to easily control the conditions needed to avoid the object before the robot crashes into it.

## Consequences

This allows the upstream layer to use this distance in their calculations on determining what avoidance procedures it should perform based on any given movement.