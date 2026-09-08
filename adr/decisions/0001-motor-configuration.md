# Motor Configuration

## Context

Decide how to store a motor's configuration. It takes in the name of the motor, the wires connected to the motor's positive and negative terminals, and the LEDC channels assigned to the motor to allow forward and backward spinning.

## Options

1. Create a struct that takes in the configurations.
2. Store the configurations as individual attributes in a class.

## Decision

I decided to combine both options where I created the configuration struct and then make it into a class attribute.

## Rationale

This better organizes the `Motor` class because I can separate the hardware configuration logic into 1 distinct unit to prevent overwritting it by accident and make it easier to understand what I am accessing.

## Consequences

I won't have direct access to the configurations unless I call the attribute. This will create padding in the compiler due to this nesting structure.