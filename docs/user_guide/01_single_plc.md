# Chapter 1 - Building a Single-PLC Application

This chapter explains how to build a first `AbstractPLC` application.

The example is intentionally small: a lamp can be switched on and off with two
buttons. Internally, the PLC uses a two-step sequence:

- `S0`: lamp off
- `S1`: lamp on

The goal is not to build a complicated machine, but to show the structure that
most PyBerryPLC applications follow.

## What You Will Build

The application contains:

- one PLC class derived from `AbstractPLC`;
- three digital inputs: start, stop, and exit;
- one digital output: lamp;
- two markers that represent the active sequence step;
- a logger created with `init_logger`;
- a software I/O backend for testing on a development PC;
- a `SoftMachine` UI to operate the virtual inputs and inspect the output.

The same PLC logic can later run on Raspberry Pi GPIO by using the hardware
backend.

## Minimal Project Structure

A small single-PLC application can start with one Python file:

```text
lamp_demo/
  main.py
  logs/
```

The `logs` directory does not have to exist beforehand. `init_logger` creates
it when the application starts.

## Creating a PLC Class

A PLC application is implemented as a class derived from `AbstractPLC`.

The usual pattern is:

1. call `super().__init__()`;
2. create inputs, outputs, markers, timers, counters, and other variables;
3. implement `control_routine()`;
4. override PLC runtime hooks only when the application needs them.

```python
from pyberryplc.core import AbstractPLC


class LampPLC(AbstractPLC):
    def __init__(self):
        super().__init__()

    def control_routine(self) -> None:
        pass
```

`control_routine()` is called once per PLC scan while the PLC is in running
mode.

In a real application, `super().__init__()` usually receives a few parameters
that configure how the PLC runtime behaves:

- `scan_time`: minimum time between the start of two consecutive scans. The
  default is `0.1`, which means 100 ms.
- `logger`: optional `logging.Logger` used by the runtime and by application
  code.
- `io_backend`: backend used to create input and output channels. If omitted,
  the PLC uses `HardwareBackend`.
- `emergency_config`: optional `EmergencyConfig` used to create standard
  emergency-stop and reset inputs.

For example:

```python
super().__init__(
    scan_time=0.1,
    logger=logger,
    io_backend=SoftwareBackend(machine_state),
    emergency_config=EmergencyConfig(
        emergency_pin="I07",
        reset_pin="I08",
    ),
)
```

Other constructor parameters exist for HMI integration and e-mail
notifications, but those are outside the scope of this first chapter.

## Configuring Logging

Use `init_logger` to create a logger for the PLC application.

```python
from pyberryplc.utils.log_utils import init_logger

logger = init_logger(
    "LAMP PLC",
    log_file="logs/lamp_plc.log",
    console=True,
)
```

The logger writes timestamped messages to a rotating log file. With
`console=True`, the same messages are also printed to the terminal.

Pass the logger to `AbstractPLC`:

```python
super().__init__(logger=logger)
```

Inside the PLC class, use `self.logger`:

```python
self.logger.info("Lamp switched on.")
```

## Choosing an I/O Backend

`AbstractPLC` does not need to know whether an input comes from a Raspberry Pi
GPIO pin or from a software test UI. That is handled by the I/O backend.

PyBerryPLC provides two main backends:

- `HardwareBackend`: creates channels connected to physical Raspberry Pi GPIO;
- `SoftwareBackend`: creates in-process software channels for testing with a
  `SoftMachine` through a `SoftMachineState` (see below).

If no backend is supplied, `AbstractPLC` uses `HardwareBackend` by default.

For a first application, it is usually easier to start with `SoftwareBackend`.
That makes it possible to test the PLC logic without wiring hardware.

```python
from pyberryplc.core import SoftwareBackend, SoftMachineState

machine_state = SoftMachineState()

super().__init__(
    logger=logger,
    io_backend=SoftwareBackend(machine_state),
)
```

`SoftMachineState` is the shared state between the PLC and the soft-machine UI.
The PLC reads and writes this state through the `SoftwareBackend`; the soft-machine UI shows
the same state in the browser, where the user can modify the state of the inputs and follow the execution of the PLC-sequence logic via log messages on screen.

## Testing With Soft-Machine

`SoftMachine` is a small NiceGUI-based browser UI used in combination with `SoftwareBackend` and `SoftMachineState` for testing PLC logic without
physical hardware.

It does not replace the PLC runtime. The PLC still runs its normal scan loop.
`SoftMachine` only gives the user a convenient way to:

- toggle software inputs;
- inspect software outputs;
- inspect the registered soft-machine state;
- show log messages in the browser.

The connection between the PLC and the UI is `SoftMachineState`.

```python
machine_state = SoftMachineState()
```

The PLC receives this state through `SoftwareBackend`:

```python
io_backend=SoftwareBackend(machine_state)
```

The soft-machine receives the same state:

```python
soft_machine = SoftMachine(states=machine_state)
```

When the PLC registers an input or output, the software backend also registers
that channel in `SoftMachineState`. The soft-machine can then render the input
or output in the browser.

In a typical soft-machine test, the PLC scan loop runs in a background thread.
This is useful because `SoftMachine.run()` starts the NiceGUI application and
takes over the main thread.

## Adding Inputs and Outputs

Digital inputs are added with `add_digital_input()`:

```python
self.StartButton = self.add_digital_input("I00", "StartButton")
self.StopButton = self.add_digital_input("I01", "StopButton")
self.ExitButton = self.add_digital_input("I02", "ExitButton")
```

Digital outputs are added with `add_digital_output()`:

```python
self.Lamp, self.LampStatus = self.add_digital_output("Q00", "Lamp")
```

The first returned value is the output variable that the PLC writes to. The
second returned value is a status variable in the input register.

When using `SoftwareBackend`, pins such as `"I00"` and `"Q00"` are virtual pin
names. When using `HardwareBackend`, the pin argument must identify the real GPIO pin.

## Working With Markers

Markers are internal memory variables. Markers are not connected to I/O channels. They only exist inside the PLC
application and are useful for sequence steps, latches, and intermediate logic.

```python
self.S0 = self.add_marker("S0", init_value=True)
self.S1 = self.add_marker("S1")
```

In this example, `S0` is active at startup and represents "lamp off". `S1`
represents "lamp on".

Markers are implemented in `pyberryplc` as `MemoryVariable` objects, which
provide useful helpers:

- `active`: `True` when the current state evaluates to true;
- `state`: the current value;
- `rising_edge`: `True` for one scan when the variable changes from off to on;
- `falling_edge`: `True` for one scan when the variable changes from on to off;
- `activate()`: set a single-bit variable to true;
- `deactivate()`: set a single-bit variable to false.

## The PLC Scan Cycle

A PLC application is not an event-driven script. It is a cyclic program.
`AbstractPLC.run()` repeats the same scan sequence until `exit()` is requested
or an unrecoverable fault occurs.

This scan-based structure is important: user code does not normally wait for a
button event or directly write to a GPIO pin. Instead, each scan reads the
current input states, executes the application logic once, writes the output
states, and then waits for the next scan.

### Runtime Loop

Each scan roughly follows this order:

1. update previous states in memory variables;
2. read physical or software inputs into `input_register`;
3. execute the logic for the current PLC mode;
4. write values from `output_register` to physical or software outputs;
5. wait until the next scan should start.

The first step is what makes edge detection possible. A `MemoryVariable` keeps
both `curr_state` and `prev_state`. At the start of a new scan, the current
state of markers and outputs is copied to the previous state. When inputs are
then read, their new value becomes the current state. Properties such as
`rising_edge` and `falling_edge` compare these two states.

The configured `scan_time` is the minimum time between scan starts. If the
application logic takes too long, the PLC cannot keep that timing. This is why
`control_routine()` should stay short and should not contain long blocking
operations.

### Registers

`AbstractPLC` keeps separate memory registers for the PLC application:

- `input_register`: memory variables that represent configured inputs and
  output status variables;
- `output_register`: memory variables whose values will be written to
  configured outputs;
- `marker_register`: internal PLC memory variables used by the application
  logic.

When an input is added with `add_digital_input()`, the PLC creates an input
channel through the I/O backend and a `MemoryVariable` in `input_register`.
At the start of each scan, the input channel is read and the memory variable is
updated.

When an output is added with `add_digital_output()`, the PLC creates an output
channel through the I/O backend and a `MemoryVariable` in `output_register`.
The application updates this memory variable during `control_routine()`. At the
end of the scan, the runtime writes the memory value to the actual output
channel.

**This means application code normally works with memory variables in the
registers, not directly with GPIO objects or software-channel objects.**

For example:

```python
if self.StartButton.rising_edge:
    self.S0.deactivate()
    self.S1.activate()

self.Lamp.update(self.S1.active)
```

The output `Lamp` is not written immediately. It is written by the PLC runtime at the
end of the current scan.

### PLC Modes

The active mode is stored in `self.mode`. It controls what the runtime does
during a scan.

- `PLCMode.INIT`: initial mode before the first running scan;
- `PLCMode.RUNNING`: normal cyclic control mode;
- `PLCMode.EMERGENCY`: emergency-stop and recovery mode;
- `PLCMode.FAULT`: unexpected crash state.

On the first scan, the PLC starts in `PLCMode.INIT`. The runtime activates
initial markers, calls `startup_routine()`, and then switches to
`PLCMode.RUNNING`.

While the PLC is running, the runtime checks the emergency conditions at each scan and then
calls `control_routine()`. If an emergency is requested or detected, the PLC
switches to `PLCMode.EMERGENCY`.

In emergency mode, the runtime instead calls `emergency_routine()` at each
scan. The default emergency routine forces all outputs off and deactivates all
markers. This gives a simple application a safe default even when it does not
override the emergency hook.

Recovery is only attempted when the PLC sees that the emergency situation has
become safe again. With the standard `EmergencyConfig`, this means that the
emergency-stop input is no longer active and the reset input is active. The
method that checks this condition is `can_recover()`. Applications with a
different recovery policy can override that method.

When recovery is allowed, the runtime calls `recover_routine()` and then
returns to running mode. The default recovery behavior forces outputs off,
deactivates markers, and reactivates initial markers.

If an unexpected exception occurs, the PLC switches to `PLCMode.FAULT` and
calls `crash_routine()`. The default crash routine logs a critical message and
forces outputs off. After the crash routine has been called, the scan loop is
left instead of continuing as if nothing happened. If an application overrides
`crash_routine()`, it can add cleanup or re-raise the exception.

## PLC Runtime Hooks

`AbstractPLC` provides runtime hooks that can be overridden when needed.

The runtime hooks are extension points around the scan loop. They let an
application add behavior or modify default behavior at well-defined moments
without rewriting `run()`.
Once the scan loop and PLC modes are understood, the role of each hook becomes
more predictable.

For a first application, these are the most important:

- `control_routine()`: required. Called every scan while the PLC is in
  `RUNNING` mode. Every concrete PLC class must implement this method.
- `startup_routine()`: optional. Called once during the transition from `INIT`
  to `RUNNING`. The default implementation does nothing.
- `on_emergency_enter()`: optional. Called once when the PLC enters
  `EMERGENCY` mode. The default implementation forces all outputs off.
- `emergency_routine()`: optional. Called every scan while the PLC remains in
  `EMERGENCY` mode. The default implementation forces all outputs off and
  deactivates all markers.
- `recover_routine()`: optional. Called once when the recovery conditions are
  met. The default implementation forces outputs off, deactivates all markers,
  and activates the initial markers.
- `on_recovered()`: optional. Called after the PLC has returned to `RUNNING`
  mode. The default implementation does nothing.
- `exit_routine()`: optional. Called during a normal shutdown requested with
  `exit()`. The default implementation forces all outputs off.
- `crash_routine(exception)`: optional. Called after an unexpected exception.
  The default implementation logs the crash and forces all outputs off.

If a hook already has a default implementation, the application does not have
to override it. When overriding a hook with default safety behavior, call
`super()` if that default behavior should be preserved and extended.

## Emergency and Reset Inputs

`EmergencyConfig` lets the PLC create standard emergency-stop and reset inputs.

```python
from pyberryplc.core import EmergencyConfig

EmergencyConfig(
    emergency_pin="I07",
    reset_pin="I08",
)
```

By default, the emergency input is treated as a normally-closed contact. With a
software backend, this means the emergency input starts active in the safe
state. Switching it off in the soft-machine simulates opening the emergency
contact.

The most common configuration options are:

- `emergency_pin`: pin used for the emergency-stop input. If omitted, no
  standard emergency input is created.
- `emergency_label`: label used in `input_register`. The default is
  `"EmergencyButton"`.
- `emergency_nc_contact`: whether the emergency contact is normally closed.
  The default is `True`.
- `reset_pin`: pin used for the reset input. If omitted, no standard reset
  input is created.
- `reset_label`: label used in `input_register`. The default is
  `"ResetButton"`.
- `global_emergency`: optional shared `MemoryVariable` used to latch an
  emergency state across multiple PLC instances.
- `latch_global_on_local_emergency`: whether a local emergency should set the
  shared emergency flag.
- `clear_global_on_recover`: whether a successful recovery should clear the
  shared emergency flag.

The shared emergency options are mainly useful in multiple-PLC applications.
They are introduced here for completeness, but chapter 3 explains the pattern
in more detail.

## Complete Example

Save this as `main.py`.

```python
import threading

from pyberryplc.core import (
    AbstractPLC,
    EmergencyConfig,
    SoftMachineState,
    SoftwareBackend,
)
from pyberryplc.soft_machine import SoftMachine
from pyberryplc.utils.log_utils import init_logger


class LampPLC(AbstractPLC):
    def __init__(self, machine_state: SoftMachineState) -> None:
        # Create one logger for this PLC instance. The same logger can also be
        # shown in the SoftMachine log panel.
        logger = init_logger(
            "LAMP PLC",
            log_file="logs/lamp_plc.log",
            console=True,
        )

        # The SoftwareBackend connects PLC I/O to the shared SoftMachineState
        # instead of to physical Raspberry Pi GPIO pins.
        super().__init__(
            logger=logger,
            io_backend=SoftwareBackend(machine_state),
            emergency_config=EmergencyConfig(
                emergency_pin="I07",
                reset_pin="I08",
            ),
        )

        # Inputs are read into the PLC input register at the start of each scan.
        self.StartButton = self.add_digital_input("I00", "StartButton")
        self.StopButton = self.add_digital_input("I01", "StopButton")
        self.ExitButton = self.add_digital_input("I02", "ExitButton")

        # The first returned variable is written by the PLC logic. The second
        # one mirrors the output status in the input register.
        self.Lamp, self.LampStatus = self.add_digital_output("Q00", "Lamp")

        # S0 is the initial sequence step: lamp off. S1 means lamp on.
        self.S0 = self.add_marker("S0", init_value=True)
        self.S1 = self.add_marker("S1")

    def startup_routine(self) -> None:
        self.logger.info("Starting Lamp PLC.")

    def control_routine(self) -> None:
        # A rising edge makes the exit button act as a momentary command.
        if self.ExitButton.rising_edge:
            self.logger.info("Exit requested.")
            self.exit()

        # Transition from S0 to S1: switch the lamp on.
        if self.S0.active and self.StartButton.rising_edge:
            self.logger.info("Switching lamp on.")
            self.S0.deactivate()
            self.S1.activate()

        # Transition from S1 to S0: switch the lamp off.
        elif self.S1.active and self.StopButton.rising_edge:
            self.logger.info("Switching lamp off.")
            self.S1.deactivate()
            self.S0.activate()

        # Output action: the lamp follows the active sequence step.
        self.Lamp.update(self.S1.active)

    def exit_routine(self) -> None:
        self.logger.info("Stopping Lamp PLC.")
        # The default exit routine forces all outputs off.
        super().exit_routine()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        super().crash_routine(exception)
        raise exception


def main() -> None:
    # This state object is shared by the PLC backend and the SoftMachine UI.
    machine_state = SoftMachineState()
    
    # Instantiate the PLC-application.
    plc = LampPLC(machine_state)

    # As SoftMachine.run() will run the NiceGUI application in the main thread,
    # it is necessary to run the PLC-application in a separate thread.
    plc_thread = threading.Thread(target=plc.run, daemon=True)
    plc_thread.start()

    # The soft-machine shows the registered software inputs and outputs in a
    # browser UI. ``on_exit=plc.exit`` will ask the PLC to shut down cleanly when
    # the Exit-button in the browser UI is clicked.
    soft_machine = SoftMachine(
        states=machine_state,
        logger=plc.logger,
        on_exit=plc.exit,
    )
    soft_machine.run()


if __name__ == "__main__":
    main()
```

Run the application:

```bash
python main.py
```

The soft-machine UI opens on the configured NiceGUI port. Use the virtual
inputs to operate the PLC:

- switch `I00 | StartButton` on briefly to activate `S1` and turn the lamp on;
- switch `I01 | StopButton` on briefly to return to `S0` and turn the lamp off;
- switch `I02 | ExitButton` on briefly to stop the PLC through `exit()`;
- switch `I07 | EmergencyButton` off to simulate an emergency-stop contact
  opening;
- switch `I08 | ResetButton` on to recover after the emergency input is safe
  again.

## Running on Raspberry Pi Hardware

After the logic has been tested with `SoftwareBackend`, the PLC can be moved to
real GPIO.

The smallest change is to remove the software backend:

```python
super().__init__(
    logger=logger,
    emergency_config=EmergencyConfig(
        emergency_pin=7,
        reset_pin=8,
    ),
)
```

Or pass a hardware backend explicitly:

```python
from pyberryplc.core import HardwareBackend

super().__init__(
    logger=logger,
    io_backend=HardwareBackend(),
)
```

On Raspberry Pi, the hardware backend relies on `gpiozero` and the `pigpio`
pin factory. `pigpio` uses a background daemon called `pigpiod` to communicate
with the GPIO hardware.

Check whether the daemon is running:

```bash
systemctl status pigpiod
```

Start it for the current session:

```bash
sudo systemctl start pigpiod
```

Enable it at boot:

```bash
sudo systemctl enable pigpiod
```

If the service is not installed, install the GPIO dependencies first. The exact
commands can differ slightly between Raspberry Pi OS versions, but a typical
setup is:

```bash
sudo apt update
sudo apt install python3-gpiozero pigpio
```

## Common Pitfalls

Keep `control_routine()` short. A PLC scan should not wait for long-running
work. If a task needs time, represent it with states, timers, or a separate
worker. Be especially careful with `for` and `while` loops: a loop that waits
for a sensor, a timer, or a remote action blocks the complete PLC scan. In PLC
logic, it is usually better to represent waiting as a sequence step and let the
next scan check whether the condition has become true.

Write outputs through the memory variables returned by `add_digital_output()`.
The PLC runtime writes the physical or software channel at the end of the scan.

Use edge detection for push buttons. A button that remains active for several
scans can otherwise trigger the same transition repeatedly.

A normal shutdown
should leave the machine in a safe state. The default `exit_routine()` already
forces all outputs off, so overriding code should preserve that behavior unless
there is a deliberate reason to replace it.

Do not create new loggers inside `control_routine()`. Create the logger once
when the PLC object is initialized.
