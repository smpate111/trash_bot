# Ultrasonic_Sensor Configuration

## Context

Decide how to store an ultrasonic sensors's configuration. It takes in the name of the sensor, the wires connected to the sensor's positive and negative terminals, and the GPIO pins assigned to the sensor to allow for sending and receiving sonic signals.

## Options

1. Create a struct that takes in the configurations.
2. Store the configurations as individual attributes in a class.

## Decision

I decided to combine both options where I created the configuration struct and then make it into a class attribute.

## Rationale

This better organizes the `Ultrasonic_Sensor` class because I can separate the hardware configuration logic into 1 distinct unit to prevent overwriting it by accident and make it easier to understand what I am accessing.

## Consequences

The configuration remains grouped as a distinct type and is stored as a private member of `Ultrasonic_Sensor`, which improves ownership and readability at the cose of exposing less direct access to the configuration.