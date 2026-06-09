from typing import Callable

import logging
import time

from pyberryplc.core import (
    AbstractPLC,
    SoftwareBackend,
    SoftMachineState,
    MemoryVariable,
    SharedMemoryBlock,
    TimerOffDelay,
    EmergencyException,
)

from loading_station_client import LoadingStation
from remote_loading_station import Status


class LoadingStationPLC(AbstractPLC):

    def __init__(
        self,
        logger: logging.Logger,
        soft_machine_state: SoftMachineState,
        db0: SharedMemoryBlock,  # shared with main plc
        db1: SharedMemoryBlock,  # shared with infeed conveyor
    ) -> None:
        super().__init__(
            io_backend=SoftwareBackend(soft_machine_state),
            logger=logger
        )
        self.init_flag: bool = True

        # Shared memory blocks
        self.db0 = db0
        self.db1 = db1

        # Variables
        self._create_variables()

        # Remote device clients
        self.loading_station = LoadingStation(logger=self.logger)

        # Steps, Transitions & Actions
        self._create_steps()
        self.T = self._create_transitions()
        self.A = self._create_actions()

        # Timers
        self.SimulateTask = TimerOffDelay(2)

    def _create_steps(self) -> None:
        self.S10 = self.add_marker("S10")
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
        self.ProductionEnable = self.db0.data["ProductionEnable"]
        self.Exit = self.db0.data["Exit"]

        self.RequestConveyorAccept = self.db1.data["RequestConveyorAccept"]
        self.ConveyorReadyToAccept = self.db1.data["ConveyorReadyToAccept"]
        self.TrayTransferDone = self.db1.data["TrayTransferDone"]
        self.ConveyorFaultActive = self.db1.data["ConveyorFaultActive"]

        # Inputs
        self.LoadingStationStart = self.add_digital_input("I00", "LoadingStationStart")
        self.TrayToLoadAvailable = self.add_digital_input("I01", "TrayToLoadAvailable")
        self.ResetButton = self.add_digital_input("I02", "ResetButton")
        self.EmergencyButton = self.add_digital_input("I08", "EmergencyButton", NC_contact=True)

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
            return True

        def T16_10() -> bool:
            return not self.ProductionEnable.active

        def T16_11() -> bool:
            return (
                self.TrayTransferRegistered.active
                and self.ProductionEnable.active
            )

        def T17_14() -> bool:
            return (
                self.ConveyorReadyToAccept.active
                and not self.ConveyorFaultActive.active
            )

        def T17_18() -> bool:
            return self.ConveyorFaultActive.active

        def T18_10() -> bool:
            return self.FaultCleared.active and self.ResetButton.active

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

            self.loading_station.check_operational_state()

            status, message = self.loading_station.get_response()
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

                self.loading_station.start_loading()
                self.LoadingCycleStarted.update(True)

            status, message = self.loading_station.get_response()
            match status:
                case Status.BUSY:
                    self.logger.info(f"Loading station says: {message}")

        def wait_loaded(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S13: WaitLoaded")

            self.TrayLoaded.update(False)
            self.LoadingCycleTimeout.update(False)
            self.loading_station.get_loading_progress()

            status, message = self.loading_station.get_response()
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

            self.RequestConveyorAccept.update(True)

        def tray_transfer(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S15: TrayTransfer")

            while self.SimulateTask.running:
                time.sleep(0.1)

            self.SimulateTask.reset()
            self.TrayTransferDone.update(True)

        def complete(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S16: Complete")

            while self.SimulateTask.running:
                time.sleep(0.1)

            self.SimulateTask.reset()
            self.TrayTransferRegistered.update(True)

        def blocked(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S17: Blocked")

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

    def _init_control(self) -> None:
        if self.init_flag:
            self.logger.info("Init loading station PLC")
            self.init_flag = False

            self.logger.info("Connect to remote loading station.")
            self.loading_station.connect()

            self.S10.activate()

    def _sequence_control(self) -> None:
        if self.Exit.active:
            self.logger.info("Closing down loading station PLC")
            self.exit()

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

    def _check_emergency_interlocks(self) -> None:
        if hasattr(self, "db0") and self.db0.data["EmergencyStopActive"].active:
            raise EmergencyException("Global Emergency Stop Activated")

        if "EmergencyButton" in self.input_register and not self.input_register["EmergencyButton"].active:
            self.db0.data["EmergencyStopActive"].update(True)
            raise EmergencyException("Local Emergency Button Pressed")

    def control_routine(self) -> None:
        self._init_control()
        self._check_emergency_interlocks()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        try:
            self.loading_station.shutdown()
        except Exception:
            self.logger.warning("Failure to send shutdown to loading station.")
        self.loading_station.close()

    def emergency_routine(self) -> None:
        self.logger.critical("EMERGENCY STOP: Loading Station safety shutdown.")

        # 1. Force local registers low
        for output_name in self.output_register:
            self.output_register[output_name].update(False)

        # 2. Notify the remote machine immediately via the TCP client!
        try:
            self.loading_station.shutdown()
        except Exception as e:
            self.logger.error(f"Could not send emergency shutdown to remote device: {e}.")
        self.loading_station.close()

        # 3. Reset the steps
        for step in [self.S10, self.S11, self.S12, self.S13, self.S14, self.S15, self.S16, self.S17, self.S18]:
            step.deactivate()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.logger.critical(f"PLC crash: {exception}")
        self.emergency_routine()
        raise exception
