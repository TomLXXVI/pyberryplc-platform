from typing import Callable

import logging

from pyberryplc.core import (
    AbstractPLC,
    SoftwareBackend,
    SoftMachineState,
    MemoryVariable,
    TimerOnDelay,
    EmergencyConfig,
)

from loading_station_client import LoadingStation
from remote_loading_station import Status
from datablocks import db0, db1


class LoadingStationPLC(AbstractPLC):

    def __init__(
        self,
        logger: logging.Logger,
        soft_machine_state: SoftMachineState,
    ) -> None:

        super().__init__(
            logger=logger,
            io_backend=SoftwareBackend(soft_machine_state),
            emergency_config=EmergencyConfig(
                emergency_pin="I07",
                reset_pin="I08",
                global_emergency=db0.data["EmergencyStopActive"],
                clear_global_on_recover=False
            )
        )

        # Remote device client
        self.loading_station = LoadingStation(logger=self.logger)

        # Variables
        self._create_variables()

        # Steps, Transitions & Actions
        self._create_steps()
        self.T = self._create_transitions()
        self.A = self._create_actions()

        # Timers
        self.SimulateTask = TimerOnDelay(2)
        self.ConveyorAcceptTimer = TimerOnDelay(5)

    def _create_steps(self) -> None:
        self.S10 = self.add_marker("S10", init_value=True)
        self.S11 = self.add_marker("S11")
        self.S12 = self.add_marker("S12")
        self.S13 = self.add_marker("S13")
        self.S14 = self.add_marker("S14")
        self.S15 = self.add_marker("S15")
        self.S16 = self.add_marker("S16")
        self.S17 = self.add_marker("S17")
        self.S18 = self.add_marker("S18")

    def _create_variables(self) -> None:
        # Shared memory
        self.ProductionEnable = db0.data["ProductionEnable"]
        self.ExitFlag = db0.data["ExitFlag"]

        self.RequestConveyorAccept = db1.data["RequestConveyorAccept"]
        self.ConveyorReadyToAccept = db1.data["ConveyorReadyToAccept"]
        self.TrayTransferDone = db1.data["TrayTransferDone"]
        self.ConveyorFaultActive = db1.data["ConveyorFaultActive"]

        # Inputs
        self.LoadingStationStart = self.add_digital_input("I00", "LoadingStationStart")
        self.TrayToLoadAvailable = self.add_digital_input("I01", "TrayToLoadAvailable")

        # Internal variables
        self.LoadingStationBusy = MemoryVariable()
        self.LoadingStationFaultActive = MemoryVariable()
        self.LoadingCycleStarted = MemoryVariable()
        self.LoadingCycleTimeout = MemoryVariable()
        self.TrayLoaded = MemoryVariable()
        self.TrayTransferRegistered = MemoryVariable()
        self.ConveyorAcceptTimeout = MemoryVariable()
        self.FaultCleared = MemoryVariable()

    def _create_transitions(self) -> dict[str, Callable[[], bool]]:

        def T10_11() -> bool:
            return (
                self.ProductionEnable.active
                and self.LoadingStationStart.active
            )

        def T11_12() -> bool:
            return (
                not self.LoadingStationFaultActive.active
                and self.TrayToLoadAvailable.active
            )

        def T11_18() -> bool:
            return self.LoadingStationFaultActive.active

        def T12_13() -> bool:
            return self.LoadingCycleStarted.active

        def T13_14() -> bool:
            return (
                self.TrayLoaded.active
                and not self.LoadingCycleTimeout.active
                and not self.LoadingStationFaultActive.active
            )

        def T13_18() -> bool:
            return (
                self.LoadingCycleTimeout.active
                or self.LoadingStationFaultActive.active
            )

        def T14_15() -> bool:
            return (
                self.ConveyorReadyToAccept.active
                and not self.ConveyorAcceptTimeout.active
            )

        def T14_17() -> bool:
            return self.ConveyorAcceptTimeout.active

        def T15_16() -> bool:
            return self.TrayTransferDone.active

        def T16_10() -> bool:
            return not self.ProductionEnable.active

        def T16_11() -> bool:
            return (
                self.TrayTransferRegistered.active
                and self.ProductionEnable.active
            )

        def T17_14() -> bool:
            return not self.ConveyorFaultActive.active

        def T17_18() -> bool:
            return self.ConveyorFaultActive.active

        def T18_10() -> bool:
            return self.FaultCleared.active and self.reset_button.active  # type: ignore

        return {
            "T10_11": T10_11,
            "T11_12": T11_12,
            "T11_18": T11_18,
            "T12_13": T12_13,
            "T13_14": T13_14,
            "T13_18": T13_18,
            "T14_15": T14_15,
            "T14_17": T14_17,
            "T15_16": T15_16,
            "T16_11": T16_11,
            "T16_10": T16_10,
            "T17_14": T17_14,
            "T17_18": T17_18,
            "T18_10": T18_10,
        }

    def _create_actions(self) -> dict[str, Callable]:

        def idle(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S10: Idle")

        def check_permissives(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S11: CheckPermissives")
                status, message = self.loading_station.check_operational_state()
                match status:
                    case Status.READY:
                        self.LoadingStationFaultActive.update(False)
                        self.logger.info(f"Loading station says: {message}")
                    case Status.ERROR:
                        self.LoadingStationFaultActive.update(True)
                        self.logger.error(f"Loading station says: {message}")

        def load_tray(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S12: LoadTray")
                self.LoadingCycleStarted.update(False)
                status, message = self.loading_station.start_loading()
                match status:
                    case Status.BUSY:
                        self.LoadingCycleStarted.update(True)
                        self.logger.info(f"Loading station says: {message}")
                    case Status.ERROR:
                        self.LoadingStationFaultActive.update(True)
                        self.logger.error(f"Loading station says: {message}")
                    case _:
                        self.LoadingStationFaultActive.update(True)
                        self.logger.error(f"Unexpected loading station response: {message}")

        def wait_loaded(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S13: WaitLoaded")

            self.TrayLoaded.update(False)
            self.LoadingCycleTimeout.update(False)

            status, message = self.loading_station.get_loading_progress()
            match status:
                case Status.DONE:
                    self.TrayLoaded.update(True)
                    self.logger.info(f"Loading station says: {message}")
                case Status.ERROR:
                    self.LoadingStationFaultActive.update(True)
                    self.logger.error(f"Loading station says: {message}")
                case Status.TIMEOUT:
                    self.LoadingCycleTimeout.update(True)
                    self.logger.info(f"Loading station says: {message}")

        def offer_to_conveyor(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S14: OfferToConveyor")
                self.ConveyorAcceptTimer.reset()
                self.ConveyorAcceptTimeout.update(False)

            self.RequestConveyorAccept.update(True)

            if self.ConveyorAcceptTimer.has_elapsed:
                self.ConveyorAcceptTimeout.update(True)

        def tray_transfer(step: MemoryVariable) -> None:
            # ``self.SimulateTask`` simulates tray transfer.
            if step.rising_edge:
                self.logger.info("S15: TrayTransfer")
                self.SimulateTask.reset()
                self.TrayTransferDone.update(False)

            if self.SimulateTask.has_elapsed:
                self.SimulateTask.reset()
                self.TrayTransferDone.update(True)

        def complete(step: MemoryVariable) -> None:
            # ``self.SimulateTask`` simulates tray registration.
            if step.rising_edge:
                self.logger.info("S16: Complete")
                self.SimulateTask.reset()
                self.TrayTransferRegistered.update(False)

            if self.SimulateTask.has_elapsed and not self.ConveyorReadyToAccept.active:
                self.SimulateTask.reset()
                self.TrayTransferDone.update(False)
                self.TrayTransferRegistered.update(True)

        def blocked(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S17: Blocked")
                self.ConveyorAcceptTimer.reset()

            self.ConveyorAcceptTimeout.update(False)

        def fault(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S18: Fault")

            self.FaultCleared.update(True)

        return {
            "S10": idle,
            "S11": check_permissives,
            "S12": load_tray,
            "S13": wait_loaded,
            "S14": offer_to_conveyor,
            "S15": tray_transfer,
            "S16": complete,
            "S17": blocked,
            "S18": fault,
        }

    def startup_routine(self) -> None:
        self.logger.info("Start Loading Station PLC")
        self.logger.info("Connect to remote loading station.")
        self.loading_station.connect()

    def _reset(self) -> None:
        self.SimulateTask.reset()
        self.ConveyorAcceptTimer.reset()
        self.RequestConveyorAccept.update(False)
        self.TrayTransferDone.update(False)
        self.LoadingStationBusy.update(False)
        self.LoadingStationFaultActive.update(False)
        self.LoadingCycleStarted.update(False)
        self.LoadingCycleTimeout.update(False)
        self.TrayLoaded.update(False)
        self.TrayTransferRegistered.update(False)
        self.ConveyorAcceptTimeout.update(False)
        self.FaultCleared.update(False)

    def recover_routine(self) -> None:
        super().recover_routine()

        self._reset()

        status, message = self.loading_station.reset()
        if status == Status.ERROR:
            self.logger.error(f"Loading station reset failed: {message}")
            self.loading_station.close()
            self.recovery_failed(f"Loading station reset failed: {message}")
        else:
            self.logger.info(f"Loading station says: {message}")

    def _sequence_control(self) -> None:
        if self.S10.active and self.T["T10_11"]():
            self.S10.deactivate()
            self.S11.activate()

        elif self.S11.active and not self.S11.rising_edge:  # make sure action S11 is executed at least once
            if self.T["T11_12"]():
                self.S11.deactivate()
                self.S12.activate()
            elif self.T["T11_18"]():
                self.S11.deactivate()
                self.S18.activate()

        elif self.S12.active and self.T["T12_13"]():
            self.S12.deactivate()
            self.S13.activate()

        elif self.S13.active:
            if self.T["T13_14"]():
                self.S13.deactivate()
                self.S14.activate()
            elif self.T["T13_18"]():
                self.S13.deactivate()
                self.S18.activate()

        elif self.S14.active:
            if self.T["T14_15"]():
                self.S14.deactivate()
                self.S15.activate()
                self.RequestConveyorAccept.update(False)
            elif self.T["T14_17"]():
                self.S14.deactivate()
                self.S17.activate()
                self.RequestConveyorAccept.update(False)

        elif self.S15.active and self.T["T15_16"]():
            self.S15.deactivate()
            self.S16.activate()

        elif self.S16.active:
            if self.T["T16_10"]():
                self.S16.deactivate()
                self.S10.activate()
                self.TrayTransferRegistered.update(False)
            elif self.T["T16_11"]():
                self.S16.deactivate()
                self.S11.activate()
                self.TrayTransferRegistered.update(False)

        elif self.S17.active:
            if self.T["T17_14"]():
                self.S17.deactivate()
                self.S14.activate()
            elif self.T["T17_18"]():
                self.S17.deactivate()
                self.S18.activate()

        elif self.S18.active and self.T["T18_10"]():
            self.S18.deactivate()
            self.S10.activate()

    def _execute_actions(self) -> None:
        if self.S10.active:
            self.A["S10"](self.S10)
        elif self.S11.active:
            self.A["S11"](self.S11)
        elif self.S12.active:
            self.A["S12"](self.S12)
        elif self.S13.active:
            self.A["S13"](self.S13)
        elif self.S14.active:
            self.A["S14"](self.S14)
        elif self.S15.active:
            self.A["S15"](self.S15)
        elif self.S16.active:
            self.A["S16"](self.S16)
        elif self.S17.active:
            self.A["S17"](self.S17)
        elif self.S18.active:
            self.A["S18"](self.S18)

    def control_routine(self) -> None:
        if self.ExitFlag.active:
            self.logger.info("Exit Loading Station PLC")
            self.exit()

        self._sequence_control()
        self._execute_actions()

    def _shutdown_loading_station(self) -> None:
        try:
            status, message = self.loading_station.shutdown()
            if status == Status.ERROR:
                self.logger.warning(f"Loading station shutdown failed: {message}")
            else:
                self.logger.info(f"Loading station says: {message}")
        except Exception:
            self.logger.warning("Failure to send shutdown to loading station.")
        self.loading_station.close()

    def exit_routine(self) -> None:
        super().exit_routine()
        self._shutdown_loading_station()

    def on_emergency_enter(self) -> None:
        super().on_emergency_enter()

        status, message = self.loading_station.emergency_stop()
        if status == Status.ERROR:
            self.logger.error(f"Could not send emergency stop to remote device: {message}.")
        else:
            self.logger.info(f"Loading station says: {message}")

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        super().crash_routine(exception)
        self._shutdown_loading_station()
