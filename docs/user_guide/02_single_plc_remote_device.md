# Chapter 2 - Single-PLC Application With a Remote Device

This chapter explains how a single `AbstractPLC` application can communicate
with a remote device.

The focus is the communication mechanism between two sides:

- the PLC side, which acts as a remote device client;
- the remote device side, which acts as a remote device server.

The chapter deliberately stays with one PLC application and one remote device.
Multiple PLC applications, shared datablocks, and system-wide coordination are
covered in a later chapter.

## What Is a Remote Device?

A remote device is a separate controller, process, machine part, or software
service that is not executed inside the PLC scan loop, but can be commanded by
the PLC application.

Typical examples are:

- a loading station controlled by another Python process;
- a microcontroller connected over a serial port;
- a Raspberry Pi handling a local machine unit;
- a device that performs a longer task while the PLC keeps scanning.

The PLC sends commands to the device. The device sends responses back to the
PLC.

## Client Side and Server Side

Remote-device communication always has two sides.

The PLC side uses a client object. PyBerryPLC currently provides two client
interfaces in `pyberryplc.remote_device`:

- `TCPRemoteDeviceClient`: communicates over TCP/IP sockets;
- `SerialRemoteDeviceClient`: communicates over a serial interface using
  `pyserial`.

The remote device side uses a server object. PyBerryPLC currently provides one
server base class:

- `TCPRemoteDeviceServer`: accepts TCP/IP socket connections and dispatches
  received commands.

This means the built-in TCP path is symmetrical: a PLC-side
`TCPRemoteDeviceClient` talks to a device-side `TCPRemoteDeviceServer`.

The serial path currently provides the PLC-side client interface. The device
itself must provide a compatible serial endpoint that reads JSON commands and
writes JSON responses.

## Message Format

The TCP and serial helpers use JSON messages terminated by a newline.

A command sent by the PLC is a dictionary such as:

```python
{"command": "start_loading"}
```

A response sent by the remote device is a dictionary such as:

```python
{"status": "busy", "message": "Loading cycle started."}
```

The exact command names, status values, and message text are application
specific. A practical application should define them explicitly, for example
with `StrEnum`.

```python
from enum import StrEnum


class Status(StrEnum):
    OFF = "off"
    READY = "ready"
    BUSY = "busy"
    DONE = "done"
    ERROR = "error"


class Command(StrEnum):
    CHECK_OPERATIONAL_STATE = "check_operational_state"
    START_LOADING = "start_loading"
    GET_LOADING_PROGRESS = "get_loading_progress"
    EMERGENCY_STOP = "emergency_stop"
    RESET = "reset"
    SHUTDOWN = "shutdown"
```

Using enums is not required, but it avoids scattering string literals across
the PLC and device code.

## Writing a TCP Remote Device Server

A TCP remote device server is implemented by subclassing
`TCPRemoteDeviceServer`.

```python
from typing import Any

from pyberryplc.remote_device import TCPRemoteDeviceServer


class LoadingStationDevice(TCPRemoteDeviceServer):
    def initialize(self) -> None:
        ...

    def handle_command(self, command: dict[str, str]) -> dict[str, Any]:
        ...
```

The user must implement two methods:

- `initialize()`: prepare the device before the server starts handling
  commands;
- `handle_command(command)`: receive one decoded command dictionary and return
  one response dictionary.

The inherited `run()` method starts the TCP message loop. It waits for a client,
reads newline-terminated JSON commands, calls `handle_command()`, and sends the
returned response as newline-terminated JSON.

### Server Initialization

Use `initialize()` for startup work that belongs to the remote device, such as:

- setting an initial status;
- resetting local timers;
- opening local hardware resources;
- moving the remote device to a known safe state.

For a simple simulated device, `initialize()` may do nothing.

```python
def initialize(self) -> None:
    self.status = Status.OFF
```

### Command Dispatch

`handle_command()` is the central dispatch point of the server. It receives a
dictionary that was sent by the client.

```python
def handle_command(self, command: dict[str, str]) -> dict[str, Any]:
    command_name = command.get("command")

    match command_name:
        case Command.CHECK_OPERATIONAL_STATE:
            return self._check_operational_state()
        case Command.START_LOADING:
            return self._start_loading()
        case Command.GET_LOADING_PROGRESS:
            return self._get_loading_progress()
        case Command.EMERGENCY_STOP:
            return self._emergency_stop()
        case Command.RESET:
            return self._reset()
        case Command.SHUTDOWN:
            return self._shutdown()
        case _:
            return {
                "status": Status.ERROR,
                "message": "Unknown command.",
            }
```

Every command handler should return a response dictionary. The response should
at least contain a `status` and a `message`, because that gives the PLC enough
information to continue the sequence or react to an error.

## Writing a PLC-Side TCP Client

The PLC should not send raw command dictionaries throughout its control logic.
It is usually clearer to create an application-specific client class that wraps
`TCPRemoteDeviceClient`.

```python
from pyberryplc.remote_device import TCPRemoteDeviceClient


class LoadingStationClient(TCPRemoteDeviceClient):
    def check_operational_state(self) -> None:
        self.send_command({"command": Command.CHECK_OPERATIONAL_STATE})

    def start_loading(self) -> None:
        self.send_command({"command": Command.START_LOADING})

    def get_loading_progress(self) -> None:
        self.send_command({"command": Command.GET_LOADING_PROGRESS})

    def emergency_stop(self) -> None:
        self.send_command({"command": Command.EMERGENCY_STOP})

    def reset(self) -> None:
        self.send_command({"command": Command.RESET})

    def shutdown(self) -> None:
        self.send_command({"command": Command.SHUTDOWN})
```

The inherited methods that matter most are:

- `connect()`: connect to the remote TCP server;
- `send_command(command_dict)`: send one JSON command;
- `wait_for_response()`: wait for one JSON response;
- `shutdown()`: send a generic shutdown command;
- `close()`: close the socket connection.

In many applications, it is useful to add a small response helper:

```python
def get_response(self) -> tuple[str, str]:
    try:
        response = self.wait_for_response()
    except Exception as error:
        return Status.ERROR, str(error)

    status = str(response.get("status"))
    message = str(response.get("message"))
    return status, message
```

This helper converts communication failures into an error status that the PLC
sequence can handle.

## Serial Client Interface

`SerialRemoteDeviceClient` follows the same client-side idea as
`TCPRemoteDeviceClient`, but communicates over a serial port.

The inherited methods are the same at the PLC side:

- `connect()`;
- `send_command(command_dict)`;
- `wait_for_response()`;
- `shutdown()`;
- `close()`.

The remote device connected to the serial port must implement the other side of
the protocol itself: read newline-terminated JSON commands and write
newline-terminated JSON responses.

Use the serial client when the remote controller is, for example, a
microcontroller connected through USB serial.

## Integrating the Client in a PLC

A remote device client should be treated as an external resource. The PLC
runtime hooks provide natural places to manage that resource.

Create the client in `__init__()`:

```python
self.loading_station = LoadingStationClient(
    host="localhost",
    port=65432,
    logger=self.logger,
)
```

Open the connection in `startup_routine()`:

```python
def startup_routine(self) -> None:
    self.logger.info("Connecting to loading station.")
    self.loading_station.connect()
```

Send commands from sequence actions, preferably when entering a step:

```python
def load_tray(self, step: MemoryVariable) -> None:
    if step.rising_edge:
        self.loading_station.start_loading()
        status, message = self.loading_station.get_response()
        self.logger.info(f"Loading station says: {message}")
```

Poll progress in a later step:

```python
def wait_loaded(self, step: MemoryVariable) -> None:
    self.loading_station.get_loading_progress()
    status, message = self.loading_station.get_response()

    if status == Status.DONE:
        self.TrayLoaded.update(True)
    elif status == Status.ERROR:
        self.LoadingStationFaultActive.update(True)
```

Send an emergency command in `on_emergency_enter()`:

```python
def on_emergency_enter(self) -> None:
    super().on_emergency_enter()
    self.loading_station.emergency_stop()
```

Reset the device in `recover_routine()`:

```python
def recover_routine(self) -> None:
    super().recover_routine()
    self.loading_station.reset()
    status, message = self.loading_station.get_response()
    if status == Status.ERROR:
        self.recovery_failed(f"Remote reset failed: {message}")
```

Shut down and close the connection during normal exit:

```python
def exit_routine(self) -> None:
    super().exit_routine()
    self.loading_station.shutdown()
    self.loading_station.close()
```

Also close the connection in `crash_routine()`:

```python
def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
    super().crash_routine(exception)
    self.loading_station.close()
```

## Complete Example

The complete example below keeps the PLC topology simple: one PLC application
communicates with one remote loading station. The remote device is simulated by
a separate Python process.

The loading station represents a small machine unit that fills trays with
parts. Which parts are loaded is not important for the example. What matters is
the interaction pattern: the PLC checks whether the remote device is ready,
starts a loading cycle, and then keeps asking for progress while the remote
device performs the longer task outside the PLC scan loop.

The example has two files:

```text
remote_tray_loader/
  remote_loading_station.py
  plc_app.py
  logs/
```

### Remote Device Server

`remote_loading_station.py`:

```python
from enum import StrEnum
from typing import Any
import logging
import threading
import time

from pyberryplc.remote_device import TCPRemoteDeviceServer


class Status(StrEnum):
    OFF = "off"
    READY = "ready"
    BUSY = "busy"
    DONE = "done"
    ERROR = "error"


class Command(StrEnum):
    CHECK_OPERATIONAL_STATE = "check_operational_state"
    START_LOADING = "start_loading"
    GET_LOADING_PROGRESS = "get_loading_progress"
    EMERGENCY_STOP = "emergency_stop"
    RESET = "reset"
    SHUTDOWN = "shutdown"


class LoadingStationDevice(TCPRemoteDeviceServer):
    def __init__(
        self,
        logger: logging.Logger,
        host: str = "localhost",
        port: int = 65432,
    ) -> None:
        super().__init__(logger, host, port)
        self.status = Status.OFF
        self._abort = False

    def initialize(self) -> None:
        self.status = Status.READY

    def handle_command(self, command: dict[str, str]) -> dict[str, Any]:
        command_name = command.get("command")

        match command_name:
            case Command.CHECK_OPERATIONAL_STATE:
                return self._response("Ready to load.")
            case Command.START_LOADING:
                return self._start_loading()
            case Command.GET_LOADING_PROGRESS:
                return self._get_loading_progress()
            case Command.EMERGENCY_STOP:
                return self._emergency_stop()
            case Command.RESET:
                return self._reset()
            case Command.SHUTDOWN:
                return self._shutdown()
            case _:
                self.status = Status.ERROR
                return self._response("Unknown command.")

    def _response(self, message: str) -> dict[str, str]:
        return {"status": self.status, "message": message}

    def _start_loading(self) -> dict[str, str]:
        if self.status == Status.BUSY:
            return self._response("Already busy.")

        self.status = Status.BUSY
        self._abort = False
        # The loading cycle takes longer than one command-response exchange.
        # Run it in a worker thread so the TCP message loop can keep responding
        # to progress, emergency, reset, and shutdown commands.
        threading.Thread(target=self._load, daemon=True).start()
        return self._response("Loading cycle started.")

    def _load(self) -> None:
        for _ in range(20):
            if self._abort:
                self.status = Status.OFF
                return
            time.sleep(0.1)
        self.status = Status.DONE

    def _get_loading_progress(self) -> dict[str, str]:
        match self.status:
            case Status.BUSY:
                return self._response("Still loading.")
            case Status.DONE:
                return self._response("Loading done.")
            case _:
                return self._response("No loading cycle active.")

    def _emergency_stop(self) -> dict[str, str]:
        self._abort = True
        self.status = Status.OFF
        return self._response("Emergency stop acknowledged.")

    def _reset(self) -> dict[str, str]:
        self._abort = False
        self.status = Status.READY
        return self._response("Reset acknowledged.")

    def _shutdown(self) -> dict[str, str]:
        self.status = Status.OFF
        return self._response("Shutting down.")


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("REMOTE DEVICE")

    device = LoadingStationDevice(logger)
    device.initialize()
    device.run()


if __name__ == "__main__":
    main()
```

### PLC Application

`plc_app.py`:

```python
import subprocess
import sys
import threading
from pathlib import Path

from pyberryplc.core import (
    AbstractPLC,
    EmergencyConfig,
    MemoryVariable,
    SoftMachineState,
    SoftwareBackend,
)
from pyberryplc.remote_device import TCPRemoteDeviceClient
from pyberryplc.soft_machine import SoftMachine
from pyberryplc.utils.log_utils import init_logger

from remote_loading_station import Command, Status


class LoadingStationClient(TCPRemoteDeviceClient):
    def send_command(self, command: Command) -> None:
        super().send_command({"command": command})

    def get_response(self) -> tuple[str, str]:
        try:
            response = self.wait_for_response()
        except Exception as error:
            return Status.ERROR, str(error)
        return str(response.get("status")), str(response.get("message"))

    def check_operational_state(self) -> None:
        self.send_command(Command.CHECK_OPERATIONAL_STATE)

    def start_loading(self) -> None:
        self.send_command(Command.START_LOADING)

    def get_loading_progress(self) -> None:
        self.send_command(Command.GET_LOADING_PROGRESS)

    def emergency_stop(self) -> None:
        self.send_command(Command.EMERGENCY_STOP)

    def reset(self) -> None:
        self.send_command(Command.RESET)

    def shutdown(self) -> None:
        self.send_command(Command.SHUTDOWN)


class LoadingStationPLC(AbstractPLC):
    def __init__(self, machine_state: SoftMachineState) -> None:
        logger = init_logger(
            "LOADING PLC",
            log_file="logs/loading_plc.log",
            console=True,
        )

        super().__init__(
            logger=logger,
            io_backend=SoftwareBackend(machine_state),
            emergency_config=EmergencyConfig(
                emergency_pin="I07",
                reset_pin="I08",
            ),
        )

        self.StartButton = self.add_digital_input("I00", "StartButton")
        self.TrayAvailable = self.add_digital_input("I01", "TrayAvailable")
        self.ExitButton = self.add_digital_input("I02", "ExitButton")
        self.LoadingActive, _ = self.add_digital_output("Q00", "LoadingActive")

        self.S0 = self.add_marker("S0", init_value=True)
        self.S1 = self.add_marker("S1")
        self.S2 = self.add_marker("S2")
        self.S3 = self.add_marker("S3")
        self.S4 = self.add_marker("S4")

        self.RemoteReady = MemoryVariable()
        self.RemoteDone = MemoryVariable()
        self.RemoteFault = MemoryVariable()

        self.loading_station = LoadingStationClient(logger=self.logger)

    def startup_routine(self) -> None:
        self.logger.info("Connecting to remote loading station.")
        self.loading_station.connect()

    def _sequence_control(self) -> None:
        if self.S0.active and self.StartButton.rising_edge:
            self.S0.deactivate()
            self.S1.activate()
        elif self.S1.active and self.RemoteReady.active and self.TrayAvailable.active:
            self.S1.deactivate()
            self.S2.activate()
        elif self.S2.active:
            self.S2.deactivate()
            self.S3.activate()
        elif self.S3.active and self.RemoteDone.active:
            self.S3.deactivate()
            self.S4.activate()
        elif self.S4.active:
            self.S4.deactivate()
            self.S0.activate()

    def _execute_actions(self) -> None:
        if self.S0.active:
            self.LoadingActive.update(False)

        elif self.S1.active:
            if self.S1.rising_edge:
                self.loading_station.check_operational_state()
                status, message = self.loading_station.get_response()
                self.logger.info(f"Remote device says: {message}")
                self.RemoteReady.update(status == Status.READY)
                self.RemoteFault.update(status == Status.ERROR)

        elif self.S2.active:
            if self.S2.rising_edge:
                self.loading_station.start_loading()
                status, message = self.loading_station.get_response()
                self.logger.info(f"Remote device says: {message}")
                self.LoadingActive.update(status == Status.BUSY)

        elif self.S3.active:
            self.loading_station.get_loading_progress()
            status, message = self.loading_station.get_response()
            self.logger.info(f"Remote device says: {message}")
            self.RemoteDone.update(status == Status.DONE)
            self.RemoteFault.update(status == Status.ERROR)

        elif self.S4.active:
            self.LoadingActive.update(False)
            self.RemoteReady.update(False)
            self.RemoteDone.update(False)

    def control_routine(self) -> None:
        if self.ExitButton.rising_edge:
            self.exit()

        self._sequence_control()
        self._execute_actions()

        if self.RemoteFault.active:
            self.request_emergency("Remote loading station reported an error.")

    def on_emergency_enter(self) -> None:
        super().on_emergency_enter()
        self.loading_station.emergency_stop()

    def recover_routine(self) -> None:
        super().recover_routine()
        self.RemoteReady.update(False)
        self.RemoteDone.update(False)
        self.RemoteFault.update(False)
        self.loading_station.reset()
        status, message = self.loading_station.get_response()
        if status == Status.ERROR:
            self.recovery_failed(f"Remote reset failed: {message}")

    def exit_routine(self) -> None:
        super().exit_routine()
        self.loading_station.shutdown()
        self.loading_station.close()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        super().crash_routine(exception)
        self.loading_station.close()


def main() -> None:
    project_dir = Path(__file__).resolve().parent

    # For this demo, start the remote device as a separate Python process.
    # In a real installation, the remote device could already be running on
    # another controller or machine.
    subprocess.Popen(
        [sys.executable, str(project_dir / "remote_loading_station.py")],
        cwd=project_dir,
    )

    # The PLC itself still runs with software I/O and a SoftMachine UI, just as
    # in chapter 1. The new element is the remote device client inside the PLC.
    machine_state = SoftMachineState()
    plc = LoadingStationPLC(machine_state)

    plc_thread = threading.Thread(target=plc.run, daemon=True)
    plc_thread.start()

    soft_machine = SoftMachine(
        states=machine_state,
        logger=plc.logger,
        on_exit=plc.exit,
    )
    soft_machine.run()


if __name__ == "__main__":
    main()
```

## Common Pitfalls

Do not scatter raw command dictionaries such as
`{"command": "start_loading"}` throughout the PLC logic. Define the command
names centrally, for example with a `Command` enum, and wrap the generic remote
device client in a small device-specific client class. In this chapter,
`LoadingStationClient` is that wrapper: it gives the PLC methods such as
`start_loading()`, `reset()`, and `emergency_stop()`.

Do not start long remote work and then block the PLC scan until it finishes.
Send a start command, return to the scan loop, and poll progress in later
scans.

Always handle communication errors. A remote device can be offline, busy, or
disconnected.

Use emergency and recovery hooks to bring the remote device to a known state.

Close the client connection during normal shutdown and after crashes.
