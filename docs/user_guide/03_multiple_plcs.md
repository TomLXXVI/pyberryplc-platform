# Chapter 3 - Multiple-PLCs Application

This chapter explains how to build an application with multiple PLC units
running in the same Python process.

The focus is coordination:

- each PLC unit has its own `AbstractPLC` scan loop;
- each PLC unit runs in its own thread;
- PLC units exchange simple signals through `SharedMemoryBlock`;
- handshakes are used to coordinate work between units.

Remote devices were covered in chapter 2. This chapter can still use a remote
device inside one PLC unit, but the main subject is PLC-to-PLC coordination.

## What You Will Build

The example system is intentionally small, but it contains the main building
blocks of a multiple-PLC application.

The system contains three PLC units:

- `MainPLC`: starts and stops production;
- `LoadingStationPLC`: loads a tray;
- `InfeedConveyorPLC`: accepts the loaded tray.

The PLC units run concurrently. They communicate through shared memory
variables:

- the main PLC enables or disables production;
- the loading station asks whether the conveyor is ready to accept a tray;
- the conveyor answers when it is ready;
- the loading station confirms when the transfer is done;
- all PLC units share a global emergency flag.

## Why Multiple PLC Units?

A larger automation system is often easier to understand when it is divided
into smaller units. Each unit can have its own sequence, local inputs, local
outputs, and local fault handling.

For example:

- a main PLC controls the global operating mode;
- a loading station PLC controls loading logic;
- a conveyor PLC controls transport logic.

Each PLC remains a normal `AbstractPLC` application. The difference is that
several PLC objects are created and each one runs its own `run()` method in a
separate thread.

## Why Threads?

`AbstractPLC.run()` is a continuous scan loop. If three PLC units must scan at
the same time, the program cannot call them one after another in the main
thread:

```python
main_plc.run()
loading_station_plc.run()
conveyor_plc.run()
```

The first call would keep running until the main PLC exits. The other PLC units
would never start.

Instead, every PLC scan loop must run concurrently:

```python
loading_station_thread = threading.Thread(
    target=loading_station_plc.run,
    daemon=True,
)
conveyor_thread = threading.Thread(
    target=conveyor_plc.run,
    daemon=True,
)
```

The main PLC can then start the other PLC threads from its
`startup_routine()`:

```python
def startup_routine(self) -> None:
    self.loading_station_thread.start()
    self.conveyor_thread.start()
```

This pattern lets each PLC keep scanning independently.

## SharedMemoryBlock

`SharedMemoryBlock` is a small container for named `MemoryVariable` objects. It
is used to share simple values between PLC units that run as threads in the
same Python process.

```python
from pyberryplc.core import SharedMemoryBlock, MemoryVariable


db0 = SharedMemoryBlock(
    name="DB0",
    data={
        "ProductionEnable": MemoryVariable(),
        "ExitFlag": MemoryVariable(),
        "EmergencyStopActive": MemoryVariable(),
    },
)
```

Each PLC receives a reference to the same `MemoryVariable` object:

```python
self.ProductionEnable = db0.data["ProductionEnable"]
```

When one PLC updates that variable, the other PLC units see the updated value
because they all refer to the same object.

### Ownership of Shared Variables

Shared memory works best when every shared variable has a clear owner.

The recommended rule is:

**one writer, one or more readers.**

For example:

| Variable | Writer | Readers | Meaning |
| :-- | :-- | :-- | :-- |
| `ProductionEnable` | `MainPLC` | Other PLC units | Production may run |
| `ExitFlag` | `MainPLC` | Other PLC units | System shutdown requested |
| `RequestConveyorAccept` | `LoadingStationPLC` | `InfeedConveyorPLC` | Loading station wants to transfer a tray |
| `ConveyorReadyToAccept` | `InfeedConveyorPLC` | `LoadingStationPLC` | Conveyor can accept the tray |
| `TrayTransferDone` | `LoadingStationPLC` | `InfeedConveyorPLC` | Tray transfer has completed |
| `ConveyorFaultActive` | `InfeedConveyorPLC` | `LoadingStationPLC` | Conveyor has a fault |

Avoid designs where two PLC units write the same shared variable. It becomes
hard to know which PLC owns the state, and the scan order can influence the
result.

### Defining Datablocks

For a multiple-PLC application, it is useful to place shared memory blocks in a
separate module, for example `datablocks.py`.

```python
from pyberryplc.core import SharedMemoryBlock, MemoryVariable


db0 = SharedMemoryBlock(
    name="DB0",
    data={
        "ProductionEnable": MemoryVariable(),
        "ExitFlag": MemoryVariable(),
        "EmergencyStopActive": MemoryVariable(),
    },
)


db1 = SharedMemoryBlock(
    name="DB1",
    data={
        "RequestConveyorAccept": MemoryVariable(),
        "ConveyorReadyToAccept": MemoryVariable(),
        "TrayTransferDone": MemoryVariable(),
        "ConveyorFaultActive": MemoryVariable(),
    },
)
```

Then each PLC imports the same `db0` and `db1` objects.

```python
from datablocks import db0, db1
```

The PLC units do not receive separate copies of the value. They all keep a
reference to the same `MemoryVariable` object. When the writer updates that
object, the readers see the updated state through their own reference to that
same object.

## Handshake Between PLC Units

A handshake is a small protocol made of shared signals. It lets two PLC units
coordinate a task without calling each other's methods directly.

A typical request-ready-done handshake looks like this:

1. The loading station sets `RequestConveyorAccept`.
2. The conveyor sees the request and checks whether it can accept the tray.
3. The conveyor sets `ConveyorReadyToAccept`.
4. The loading station sees that the conveyor is ready and performs the tray
   transfer.
5. The loading station sets `TrayTransferDone`.
6. The conveyor sees that the transfer is done and clears its ready state.
7. The loading station clears the request and done signals.

The exact reset moment depends on the sequence design. The important point is
that every signal has a writer and every transition has a clear condition.

### Example Handshake Signals

In the loading station PLC:

```python
self.RequestConveyorAccept = db1.data["RequestConveyorAccept"]
self.ConveyorReadyToAccept = db1.data["ConveyorReadyToAccept"]
self.TrayTransferDone = db1.data["TrayTransferDone"]
```

The loading station writes the request:

```python
def offer_to_conveyor(step: MemoryVariable) -> None:
    if step.rising_edge:
        self.logger.info("Offer tray to conveyor.")

    self.RequestConveyorAccept.update(True)
```

Then it waits for the conveyor by means of a transition condition. The PLC does
not block here. On every scan, the transition function is evaluated again until
the shared signal becomes active:

```python
def T14_15() -> bool:
    return self.ConveyorReadyToAccept.active
```

After transfer, it writes the done signal:

```python
self.TrayTransferDone.update(True)
```

In the conveyor PLC:

```python
self.RequestConveyorAccept = db1.data["RequestConveyorAccept"]
self.ConveyorReadyToAccept = db1.data["ConveyorReadyToAccept"]
self.TrayTransferDone = db1.data["TrayTransferDone"]
```

The conveyor reads the request and writes the ready signal:

```python
def accept_tray(step: MemoryVariable) -> None:
    if step.rising_edge:
        self.logger.info("Ready to accept tray.")

    self.ConveyorReadyToAccept.update(True)
```

It returns to its normal sequence when the transfer is done:

```python
def T24_21() -> bool:
    return self.TrayTransferDone.active
```

## Main PLC

The main PLC coordinates global commands. It can keep a small sequence with
states such as disabled and enabled.

```python
class MainPLC(AbstractPLC):
    def _create_variables(self) -> None:
        self.StartButton = self.add_digital_input("I00", "StartButton")
        self.StopButton = self.add_digital_input("I01", "StopButton")
        self.ExitButton = self.add_digital_input("I02", "ExitButton")

        self.ProductionEnable = db0.data["ProductionEnable"]
        self.ExitFlag = db0.data["ExitFlag"]
```

When production is enabled, the main PLC writes:

```python
self.ProductionEnable.update(True)
```

Other PLC units read that value:

```python
def T10_11() -> bool:
    return (
        self.ProductionEnable.active
        and self.LoadingStationStart.active
    )
```

For shutdown, the main PLC sets `ExitFlag`:

```python
if self.ExitButton.active:
    self.ExitFlag.update(True)
    self.exit()
```

Other PLC units check the same flag:

```python
def control_routine(self) -> None:
    if self.ExitFlag.active:
        self.exit()
```

### Starting PLC Threads

The main program creates the PLC objects and starts the main PLC in its own
thread. The main PLC can then start the other PLC units from its
`startup_routine()`.

```python
main_plc = MainPLC(
    main_state,
    loading_station_state,
    conveyor_state,
)

main_plc_thread = threading.Thread(target=main_plc.run, daemon=True)
main_plc_thread.start()
```

Inside `MainPLC.__init__()`:

```python
self.loading_station_plc = LoadingStationPLC(
    logger=loading_station_logger,
    soft_machine_state=loading_station_state,
)
self.loading_station_thread = threading.Thread(
    target=self.loading_station_plc.run,
    daemon=True,
)

self.conveyor_plc = InfeedConveyorPLC(
    logger=conveyor_logger,
    soft_machine_state=conveyor_state,
)
self.conveyor_thread = threading.Thread(
    target=self.conveyor_plc.run,
    daemon=True,
)
```

Inside `MainPLC.startup_routine()`:

```python
def startup_routine(self) -> None:
    self.logger.info("Start Main PLC.")
    self.loading_station_thread.start()
    self.conveyor_thread.start()
```

This gives the application one scan loop per PLC unit.

### Stopping PLC Threads

A clean shutdown should ask every PLC unit to leave its scan loop.

One practical pattern is to use a shared `ExitFlag`:

```python
self.ExitFlag = db0.data["ExitFlag"]
```

The main PLC writes it:

```python
self.ExitFlag.update(True)
self.exit()
```

The other PLC units read it:

```python
if self.ExitFlag.active:
    self.exit()
```

When the main PLC exits, it can wait for child PLC threads:

```python
def exit_routine(self) -> None:
    super().exit_routine()
    self.loading_station_thread.join()
    self.conveyor_thread.join()
```

Use joins carefully. A thread can only be joined when it is actually expected
to stop. If a child PLC is waiting forever on a blocking operation, the join
will also wait forever.

## Global Emergency

`EmergencyConfig` can share a global emergency flag between PLC units.

```python
EmergencyConfig(
    emergency_pin="I07",
    reset_pin="I08",
    global_emergency=db0.data["EmergencyStopActive"],
)
```

When one PLC detects a local emergency, it can latch the shared emergency flag.
Other PLC units that use the same `global_emergency` variable see that flag and
enter emergency mode as well.

For child PLC units, it is often useful to avoid clearing the global emergency
flag from every PLC:

```python
EmergencyConfig(
    emergency_pin="I07",
    reset_pin="I08",
    global_emergency=db0.data["EmergencyStopActive"],
    clear_global_on_recover=False,
)
```

That lets the main PLC or another designated owner decide when the global
emergency state is cleared.

## Thread-Safety Notes

`SharedMemoryBlock` is intentionally simple. It stores a dictionary of
`MemoryVariable` objects. It does not currently use a `threading.Lock`.

This means it is not thread-safe in the strict sense. `MemoryVariable.update()`
does two assignments:

```python
self.prev_state = self.curr_state
self.curr_state = value
```

Those assignments are not protected as one atomic transaction.

In practice, this is usually acceptable for the simple boolean-style signals
used in the examples, if the application follows a clear discipline:

- use one writer per shared variable;
- let other PLC units read that variable;
- keep shared values simple, such as booleans or small scalar values;
- avoid read-modify-write logic from multiple PLC threads;
- avoid using shared variables for complex mutable objects.

The most important consequence concerns edge detection. A shared
`MemoryVariable` has one `prev_state` and one `curr_state`, shared by all PLC
units. If one PLC updates that object, it also changes the state history that
the other PLC units observe. Therefore `rising_edge` on a shared variable does
not mean "this receiving PLC has just seen a new event"; it only reflects the
single shared history stored inside that object.

If a receiving PLC needs to detect that a shared signal became active for its
own sequence, copy the shared level signal into a local marker or local
`MemoryVariable` owned by that PLC. The receiving PLC can then perform edge
detection on its own local memory, whose `prev_state` is updated by that PLC's
own scan loop.

For handshake signals, prefer level-based logic:

```python
if self.RequestConveyorAccept.active:
    self.ConveyorReadyToAccept.update(True)
```

This is more robust than relying on a shared `rising_edge`.

If a future application needs stronger guarantees, consider adding a locked
wrapper or extending `SharedMemoryBlock` with explicit synchronization. For the
current examples, the single-writer and level-signal pattern is the intended
use.

## Complete Example Outline

A complete multiple-PLC application can be organized like this:

```text
multi_plc_system/
  datablocks.py
  plc_main.py
  plc_loading_station.py
  plc_conveyor.py
  main.py
  logs/
```

`datablocks.py` defines shared variables. Each PLC module imports those blocks.
The main program creates the PLC objects, starts the main PLC thread, and opens
the `SoftMachine`.

For visual testing, create a separate `SoftMachineState` for each PLC:

```python
main_state = SoftMachineState()
loading_station_state = SoftMachineState()
conveyor_state = SoftMachineState()
```

Pass them to `SoftMachine` as named states:

```python
soft_machine = SoftMachine(
    states={
        "main": main_state,
        "loading_station": loading_station_state,
        "conveyor": conveyor_state,
    },
    datablocks={
        "DB0": db0,
        "DB1": db1,
    },
    logger={
        "main": main_plc.logger,
        "loading_station": main_plc.loading_station_plc.logger,
        "conveyor": main_plc.conveyor_plc.logger,
    },
    on_exit=main_plc.exit,
)
```

This makes it possible to inspect each PLC unit and the shared datablocks from
one browser UI.

## Common Pitfalls

Do not let multiple PLC units write the same shared variable unless there is a
very clear arbitration rule.

Do not rely on shared `rising_edge` as a per-PLC event. Use shared variables as
level signals, and copy them to local memory if edge detection is needed.

Do not block one PLC scan loop while waiting for another PLC. Use shared
signals and let the next scan evaluate whether the other unit is ready.

Do not forget to reset handshake signals. A request or done flag that remains
active can make a later cycle start from the wrong state.

Be careful with `join()`. It is useful during shutdown, but only after the
child PLC has been asked to exit.

Keep ownership visible in the code. Comments such as `main writes / others
read` are simple but valuable.
