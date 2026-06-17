# Demo 1 - Multi-PLC Loading Cell

This demo brings together the first three chapters of the PyBerryPLC user
guide:

- chapter 1: a cyclic PLC application with a software I/O backend;
- chapter 2: communication with a remote device;
- chapter 3: multiple PLC units coordinated through shared datablocks.

The application is a simulated loading cell. A main PLC enables production, a
loading-station PLC asks a remote loading station to load a tray, and an infeed
conveyor PLC accepts the loaded tray through a small handshake.

The goal of the demo is not to model a complete machine. It is meant to show
how the building blocks of a larger PyBerryPLC application fit together.

## Files

```text
demos/demo_1/
  datablocks.py
  loading_station_client.py
  plc_conveyor.py
  plc_loading_station.py
  plc_main.py
  remote_loading_station.py
  README.md
```

| File | Role |
| :-- | :-- |
| `plc_main.py` | Starts the complete demo, runs the main PLC, starts the other PLC units, and opens the SoftMachine UI. |
| `plc_loading_station.py` | PLC unit that controls the loading-station sequence. It talks to the remote loading station and coordinates with the conveyor. |
| `plc_conveyor.py` | PLC unit that simulates the infeed conveyor and accepts trays from the loading station. |
| `remote_loading_station.py` | Simulated remote device, running as a separate TCP server process. |
| `loading_station_client.py` | PLC-side wrapper around `TCPRemoteDeviceClient` for the loading-station commands. |
| `datablocks.py` | Shared memory blocks used for communication between PLC units. |

## Architecture

The demo contains three PLC units and one remote device.

```text
SoftMachine UI
    |
    +-- MainPLC
    |     |
    |     +-- shared DB0: production, exit, global emergency
    |
    +-- LoadingStationPLC
    |     |
    |     +-- TCP client
    |     |     |
    |     |     +-- remote_loading_station.py
    |     |
    |     +-- shared DB1: conveyor request / ready / done handshake
    |
    +-- InfeedConveyorPLC
          |
          +-- shared DB1
```

Each PLC has its own `SoftMachineState` and runs its own PLC scan loop in a
separate thread. The remote loading station is deliberately not part of the PLC
scan loop. It runs as a separate process and communicates over TCP.

This separation is important:

- the PLC scan loops stay cyclic;
- long-running loading work happens outside the PLC;
- PLC-to-PLC coordination uses simple shared signals;
- remote-device communication is wrapped behind a small domain-specific client.

## Shared Datablocks

`datablocks.py` defines two shared memory blocks.

### DB0 - System-Level Coordination

| Variable | Writer | Readers | Meaning |
| :-- | :-- | :-- | :-- |
| `ProductionEnable` | `MainPLC` | `LoadingStationPLC`, `InfeedConveyorPLC` | Production is allowed to run. |
| `ExitFlag` | `MainPLC` | `LoadingStationPLC`, `InfeedConveyorPLC` | The complete system should shut down. |
| `EmergencyStopActive` | PLC runtime / emergency logic | All PLC units | Shared global emergency flag. |

### DB1 - Loading Station / Conveyor Handshake

| Variable | Writer | Reader | Meaning |
| :-- | :-- | :-- | :-- |
| `RequestConveyorAccept` | `LoadingStationPLC` | `InfeedConveyorPLC` | Loading station wants to transfer a loaded tray. |
| `ConveyorReadyToAccept` | `InfeedConveyorPLC` | `LoadingStationPLC` | Conveyor can accept the tray. |
| `TrayTransferDone` | `LoadingStationPLC` | `InfeedConveyorPLC` | Tray transfer has completed. |
| `ConveyorFaultActive` | `InfeedConveyorPLC` | `LoadingStationPLC` | Conveyor is in fault state. |

The demo follows the "one writer, one or more readers" rule from chapter 3 of
the user guide. Shared variables are used as level signals, not as per-PLC edge
events.

## Sequence Overview

### MainPLC

The main PLC has two sequence steps:

- `S0`: production disabled;
- `S1`: production enabled.

Use `StartButton` to enable production and `StopButton` to disable it. When
the main PLC is enabled, it writes `ProductionEnable = True` to `DB0`.

The main PLC also owns system shutdown. When `ExitButton` is active, or when
the SoftMachine exits, it writes `ExitFlag = True` and requests all PLC units
to stop.

### LoadingStationPLC

The loading-station PLC performs the loading workflow:

1. wait until production is enabled and the local start input is active;
2. check whether the remote loading station is operational;
3. start a remote loading cycle;
4. poll the remote loading station until loading is done, failed, or timed out;
5. request the conveyor to accept the tray;
6. wait until the conveyor is ready, or time out;
7. simulate the tray transfer without blocking the PLC scan loop;
8. register completion and return to the next cycle.

Remote-device commands are sent through `loading_station_client.py`. Each
domain method, such as `start_loading()` or `get_loading_progress()`, sends one
command and waits for the matching response. This keeps the PLC sequence code
readable and avoids scattered `send_command()` / `wait_for_response()` pairs.

### InfeedConveyorPLC

The infeed conveyor PLC waits until:

- production is enabled;
- its local conveyor start input is active;
- the handoff position is free.

When the loading station requests a tray transfer and the tray exit is free,
the conveyor enters its accept step and writes `ConveyorReadyToAccept = True`.
It returns to its normal sequence when `TrayTransferDone` becomes active.

## Remote Loading Station

`remote_loading_station.py` is a simulated TCP remote device. It accepts JSON
commands from the PLC client and returns JSON responses.

Supported commands:

- `check_operational_state`;
- `start_loading`;
- `get_loading_progress`;
- `emergency_stop`;
- `reset`;
- `shutdown`.

The simulated loading cycle runs in a worker thread inside the remote device.
This is intentional: the remote device can keep responding to progress,
emergency, reset, and shutdown commands while the loading cycle is in progress.

The simulation includes random outcomes:

- an operational check can return ready or error;
- a loading cycle can finish successfully;
- a loading cycle can fail;
- a loading cycle can time out.

This makes the demo useful for observing normal operation, faults, recovery,
and shutdown behavior.

## Running The Demo

From the repository root:

```bash
python demos/demo_1/plc_main.py
```

Or, from inside the demo directory:

```bash
python plc_main.py
```

`plc_main.py` starts the remote loading-station process first, then starts the
main PLC. The main PLC starts the loading-station PLC and conveyor PLC. Finally,
the SoftMachine UI opens so the virtual inputs and datablocks can be inspected.

Log files are written to:

```text
demos/demo_1/logs/
```

## Operating The Demo

In the SoftMachine UI, use the three PLC panels.

### Main PLC Inputs

| Input | Use |
| :-- | :-- |
| `StartButton` | Enable production. |
| `StopButton` | Disable production. |
| `ExitButton` | Shut down the complete demo. |
| `EmergencyButton` | Simulate emergency-stop behavior. |
| `ResetButton` | Recover after emergency conditions are safe again. |

### Loading Station PLC Inputs

| Input | Use |
| :-- | :-- |
| `LoadingStationStart` | Allow the loading-station sequence to run. |
| `TrayToLoadAvailable` | Simulate that a tray is available for loading. |

### Infeed Conveyor PLC Inputs

| Input | Use |
| :-- | :-- |
| `ConveyorStart` | Allow the conveyor sequence to run. |
| `HandoffPositionFree` | Simulate that the conveyor handoff position is free. |
| `TrayExitFree` | Simulate that the tray can leave the transfer position. |

A typical successful cycle is:

1. enable `StartButton` on the main PLC;
2. enable `LoadingStationStart`;
3. enable `TrayToLoadAvailable`;
4. enable `ConveyorStart`;
5. enable `HandoffPositionFree`;
6. enable `TrayExitFree`;
7. watch the steps and datablocks as the loading station and conveyor complete
   the handshake.

Because the remote loading station has randomized outcomes, a cycle can also
enter a fault path. Use the emergency and reset inputs to observe recovery.

## Scan-Cycle Design Notes

PLC code should stay cyclic. It should not wait inside `control_routine()` for
a sensor, timer, remote device, or another PLC unit.

This demo uses the following patterns:

- remote work is started by command and later polled;
- transfer delays are represented with timers checked over multiple scans;
- conveyor coordination uses shared level signals;
- shutdown is requested through the shared `ExitFlag`;
- emergency state is shared through `EmergencyStopActive`.

These patterns are intentionally close to the recommendations in the user
guide. They keep each PLC responsive to exit, emergency, recovery, and changing
input conditions.

## What To Look For

While running the demo, inspect:

- the active sequence steps of each PLC;
- `DB0.ProductionEnable` when the main PLC starts and stops production;
- `DB1.RequestConveyorAccept` when the loading station offers a tray;
- `DB1.ConveyorReadyToAccept` when the conveyor can accept;
- `DB1.TrayTransferDone` when the simulated transfer completes;
- the log messages from each PLC and from the remote loading station.

Together, these signals show the central idea of the demo: independent PLC
scan loops coordinate through small, explicit communication contracts.
