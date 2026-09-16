# Drive_Train Configuration

## Context

Decide how to store 2 motor drivers' configurations and 2 wheel encoders' configurations in `Drive_Train`. It takes in the name of the drive train and each of the configurations to allow the robot to move forward, move backward, turn left, and turn right while recording the distance traveled by the left and right wheel groups.

## Options

1. Create a struct that takes in the configurations.
2. Store the configurations as individual attributes in a class.

## Decision

I decided to combine both options where I created the configuration struct and then make it into a class attribute.

## Rationale

This better organizes the `Drive_Train` class because I can separate the hardware configuration logic into 1 distinct unit to prevent overwriting it by accident and make it easier to understand what I am accessing.

## Consequences

The configuration remains grouped as a distinct type and is stored as a private member of `Drive_Train`, which improves ownership and readability at the cost of exposing less direct access to the configuration.