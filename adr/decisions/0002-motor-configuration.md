# Motor Configuration

## Context

Decide how to store a motor's configuration. It takes in the name of the motor, the wires connected to the motor's positive and negative terminals, and the LEDC channels assigned to the motor to allow forward and backward spinning.

## Options

1. Create a struct that takes in the configurations.
2. Store the configurations as individual attributes in a class.

## Decision

I decided to combine both options where I created the configuration struct and then make it into a class attribute.

## Rationale

This better organizes the `Motor` class because I can separate the hardware configuration logic into 1 distinct unit to prevent overwriting it by accident and make it easier to understand what I am accessing.

## Consequences

The configuration remains grouped as a distinct type and is stored as a private member of `Motor`, which imporves ownership and readability at the cost of exposing less direct access to the configuration.