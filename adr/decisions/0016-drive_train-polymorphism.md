# Drive_Train Polymorphism

## Decision

`Drive_Train` will remain a concrete class that will not use virtual actuator methods because the current architecture has no requirement for runtime substitution of multiple `Drive_Train` implementations.

## Rationale

Testing is performed by faking the underlying ESP-IDF hardware APIs rather than substituting `Drive_Train` itself.