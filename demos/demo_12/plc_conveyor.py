from typing import Callable
import logging

from pyberryplc.core import (
    AbstractPLC,
    SoftwareBackend,
    SoftMachineState,
    SharedMemoryBlock,
    MemoryVariable,
)


class InfeedConveyorPLC(AbstractPLC):

    def __init__(
        self,
        logger: logging.Logger,
        soft_machine_state: SoftMachineState,
        db0: SharedMemoryBlock,  # shared with main plc
        db1: SharedMemoryBlock,  # shared with loading station
    ) -> None:
        super().__init__(
            logger=logger,
            io_backend=SoftwareBackend(soft_machine_state)
        )
        self.init_flag = True

        self.db0 = db0
        self.db1 = db1

        self._create_variables()

        self._create_steps()
        self.T = self._create_transitions()
        self.A = self._create_actions()

    def _create_steps(self) -> None:
        self.S20 = self.add_marker("S20")
        self.S21 = self.add_marker("S21")
        self.S22 = self.add_marker("S22")
        self.S23 = self.add_marker("S23")
        self.S24 = self.add_marker("S24")
        self.S25 = self.add_marker("S25")

    def _create_variables(self) -> None:
        self.ProductionEnable = self.db0.data["ProductionEnable"]
        self.ExitMain = self.db0.data["ExitMain"]

        self.RequestConveyorAccept = self.db1.data["RequestConveyorAccept"]
        self.ConveyorReadyToAccept = self.db1.data["ConveyorReadyToAccept"]
        self.TrayTransferDone = self.db1.data["TrayTransferDone"]
        self.ConveyorFaultActive = self.db1.data["ConveyorFaultActive"]

        self.ConveyorStart = self.add_digital_input("I00", "ConveyorStart")
        self.HandoffPositionFree = self.add_digital_input("I01", "HandoffPositionFree")
        self.TrayExitFree = self.add_digital_input("I02", "TrayExitFree")
        self.ResetRequest = self.add_digital_input("I04", "ResetRequest")

        self.FaultCleared = MemoryVariable()

    def _create_transitions(self) -> dict[str, Callable[[], bool]]:

        def T20_21() -> bool:
            return (
                self.ProductionEnable.active
                and self.ConveyorStart.active
            )

        def T21_22() -> bool:
            return (
                self.HandoffPositionFree.active
                and not self.ConveyorFaultActive.active
            )

        def T21_25() -> bool:
            return self.ConveyorFaultActive.active

        def T22_23() -> bool:
            return (
                (self.RequestConveyorAccept.active and self.TrayExitFree.active)
                or not self.HandoffPositionFree.active
                or not self.ProductionEnable.active
            )

        def T23_24() -> bool:
            return (
                (self.RequestConveyorAccept.active and self.TrayExitFree.active)
                and self.ProductionEnable.active
            )

        def T23_21() -> bool:
            return (
                not(self.RequestConveyorAccept.active and self.TrayExitFree.active)
                and self.ProductionEnable.active
            )

        def T23_20() -> bool:
            return not self.ProductionEnable.active

        def T24_21() -> bool:
            return self.TrayTransferDone.active

        def T25_20() -> bool:
            return self.FaultCleared.active and self.ResetRequest.active

        return {
            "T20_21": T20_21,
            "T21_22": T21_22,
            "T21_25": T21_25,
            "T22_23": T22_23,
            "T23_24": T23_24,
            "T23_21": T23_21,
            "T23_20": T23_20,
            "T24_21": T24_21,
            "T25_20": T25_20,
        }

    def _create_actions(self) -> dict[str, Callable[[MemoryVariable], None]]:

        def idle(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S20: Idle")

        def check_permissives(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S21: CheckPermissives")

        def conveyor_run(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S22: ConveyorRun")

        def conveyor_stop(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S23: ConveyorStop")

        def accept_tray(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S24: AcceptTray")

            self.ConveyorReadyToAccept.update(True)

        def fault(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S25: Fault")

            self.FaultCleared.update(True)

        return {
            "S20": idle,
            "S21": check_permissives,
            "S22": conveyor_run,
            "S23": conveyor_stop,
            "S24": accept_tray,
            "S25": fault,
        }

    def _init_control(self) -> None:
        if self.init_flag:
            self.logger.info("Init infeed conveyor PLC")
            self.init_flag = False
            self.S20.activate()

    def _sequence_control(self) -> None:
        if self.ExitMain.active:
            self.logger.info("Closing down infeed conveyor PLC")
            self.exit()

        if self.S20.active and self.T["T20_21"]():
            self.S20.deactivate()
            self.S21.activate()

        elif self.S21.active and not self.S21.rising_edge:
            if self.T["T21_22"]():
                self.S21.deactivate()
                self.S22.activate()
            elif self.T["T21_25"]():
                self.S21.deactivate()
                self.S25.activate()

        elif self.S22.active and self.T["T22_23"]():
            self.S22.deactivate()
            self.S23.activate()

        elif self.S23.active:
            if self.T["T23_24"]():
                self.S23.deactivate()
                self.S24.activate()
            elif self.T["T23_21"]():
                self.S23.deactivate()
                self.S21.activate()
            elif self.T["T23_20"]():
                self.S23.deactivate()
                self.S20.activate()

        elif self.S24.active and self.T["T24_21"]():
            self.S24.deactivate()
            self.S21.activate()
            self.ConveyorReadyToAccept.update(False)
            self.TrayTransferDone.update(False)

        elif self.S25.active and self.T["T25_20"]():
            self.S25.deactivate()
            self.S20.activate()

    def _execute_actions(self) -> None:
        if self.S20.active:
            self.A["S20"](self.S20)
        if self.S21.active:
            self.A["S21"](self.S21)
        if self.S22.active:
            self.A["S22"](self.S22)
        if self.S23.active:
            self.A["S23"](self.S23)
        if self.S24.active:
            self.A["S24"](self.S24)
        if self.S25.active:
            self.A["S25"](self.S25)

    def control_routine(self) -> None:
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        pass

    def emergency_routine(self) -> None:
        pass

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.logger.critical(exception)
        raise exception
