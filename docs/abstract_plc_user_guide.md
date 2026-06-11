# AbstractPLC User Guide

`AbstractPLC` is the core runtime class for building PLC-like Python
applications with pyberryplc. It provides a scan cycle, I/O registers, mode
handling, emergency-stop behaviour, recovery behaviour, and safe defaults.

## Basic Structure

Create a subclass and implement `control_routine`.

```python
from pyberryplc.core import AbstractPLC, EmergencyConfig, SoftwareBackend


class MotorPLC(AbstractPLC):
    def __init__(self, state):
        super().__init__(
            io_backend=SoftwareBackend(state),
            emergency_config=EmergencyConfig(
                emergency_pin="I08",
                reset_pin="I03",
            ),
        )
        self.StartButton = self.add_digital_input("I00", "StartButton")
        self.Motor, self.MotorStatus = self.add_digital_output("Q00", "Motor")
        self.Idle = self.add_marker("Idle", initial=True)

    def control_routine(self) -> None:
        self.Motor.update(self.StartButton.active)
```

## Lifecycle Hooks

The runtime calls these hooks:

- `startup_routine()` once when `run()` starts.
- `control_routine()` every scan in running mode.
- `on_emergency_enter()` once when emergency mode is entered.
- `emergency_routine()` every scan in emergency mode.
- `recover_routine()` once when reset conditions are met.
- `on_recovered()` after the PLC returns to running mode.
- `exit_routine()` during normal shutdown.
- `crash_routine(error)` when an unexpected exception occurs.

Only `control_routine()` is required. All other hooks have safe defaults.

## Emergency and Recovery

`EmergencyConfig` can create standard `EmergencyButton` and `ResetButton`
inputs. By default, the emergency button is treated as a normally-closed
contact.

```python
EmergencyConfig(
    emergency_pin="I08",
    reset_pin="I03",
)
```

Default emergency behaviour:

- force all outputs off;
- deactivate all markers;
- remain in emergency mode while the emergency button is pressed.

Default startup and recovery behaviour:

- markers registered with `initial=True` are activated during startup;
- recovery is allowed when the emergency button is released and reset is active;
- all outputs remain off for the recovery scan;
- all markers are deactivated;
- markers registered with `initial=True` are activated again during recovery.

## Global Emergency Flag

Multiple PLC units can share one emergency state using a `MemoryVariable`.

```python
from pyberryplc.core import MemoryVariable

EmergencyStopActive = MemoryVariable()

config = EmergencyConfig(
    emergency_pin="I08",
    reset_pin="I03",
    global_emergency=EmergencyStopActive,
)
```

When a local emergency is detected, the global emergency flag is latched.
When recovery succeeds, the flag is cleared by default.

## Overriding Emergency Behaviour

Use `on_emergency_enter()` for one-shot actions such as stopping a remote
device. Use `emergency_routine()` for repeated per-scan safety actions.

```python
def on_emergency_enter(self) -> None:
    super().on_emergency_enter()
    self.remote_station.emergency_stop()

def recover_routine(self) -> None:
    super().recover_routine()
    self.remote_station.reset()
```

Avoid sending destructive commands from `emergency_routine()`, because it runs
on every emergency scan.

## Requesting Emergency from Application Logic

Application code can request emergency mode explicitly.

```python
def control_routine(self) -> None:
    if self.SensorFault.active:
        self.request_emergency("Sensor fault detected.")
```

## Failed Recovery

If an application-specific recovery step fails, call `recovery_failed()`.
The PLC remains in emergency mode and the optional global emergency flag is
latched again.

```python
def recover_routine(self) -> None:
    super().recover_routine()
    self.remote_station.reset()
    status, message = self.remote_station.get_response()
    if status == Status.ERROR:
        self.recovery_failed(f"Remote station reset failed: {message}")
```

## Initial Markers

Markers marked as initial are activated by the default startup and recovery
routines.

```python
self.S0 = self.add_marker("S0", initial=True)
```

This is useful for Grafcet/SFC-style PLC applications.
