# Wheel_Encoder Configuration

## Context

Decide how to store a wheel encoder's configuration. It takes in the name of the wheel encoder, the wires connected to the encoder's positive and negative terminals, the GPIO pin assigned to the wheel encoder to allow for encoder counting, the number of encoder slots, and the wheel's diameter.

## Options

1. Create a struct that takes in the configurations.
2. Store the configurations as individual attributes in a class.

## Decision

I decided to combine both options where I created the configuration struct and then make it into a class attribute.

## Rationale

This better organizes the `Wheel_Encoder` class because I can separate the hardware configuration logic into 1 distinct unit to prevent overwriting it by accident and make it easier to understand what I am accessing.

## Consequences

The configuration remains grouped as a distinct type and is stored as a private member of `Wheel_Encoder`, which improves ownership and readability at the cose of exposing less direct access to the configuration.