from typing import Callable
import logging
import random

from pyberryplc.core import AbstractPLC, SharedMemoryBlock, MemoryVariable
from pyberryplc.utils.keyboard_input import KeyInput


class InfeedConveyorPLC(AbstractPLC):

    def __init__(
        self,
        logger: logging.Logger,
        db0: SharedMemoryBlock,  # shared with main plc
        db1: SharedMemoryBlock,  # shared with loading station
    ) -> None:
        super().__init__(logger=logger)

        self.key_input = KeyInput()

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
        self.S26 = self.add_marker("S26")
        self.S27 = self.add_marker("S27")

    def _create_variables(self) -> None:
        self.ProductionEnable = self.db0.data["ProductionEnable"]
        self.RequestConveyorAccept = self.db1.data["RequestConveyorAccept"]
        self.ConveyorFaultActive = MemoryVariable()
        self.ConveyorReady = MemoryVariable()
        self.HandoffPositionFree = MemoryVariable()
        self.ConveyorReadyToAccept = self.db1.data["ConveyorReadyToAccept"]
        self.ConveyorAcceptedTray = self.db1.data["ConveyorAcceptedTray"]
        self.ConveyorRunning = MemoryVariable()
        self.ConveyorStartTimeOut = MemoryVariable()
        self.TrayAtHandoff = MemoryVariable()
        self.TransportTimeOut = MemoryVariable()
        self.TrayHandoffRegistered = MemoryVariable()
        self.FaultCleared = MemoryVariable()
        self.ResetRequest: MemoryVariable = self._create_key_button("r")

    def _create_key_button(self, key: str) -> MemoryVariable:
        self.key_input.keys[key] = MemoryVariable()
        return self.key_input.keys[key]

    def _create_transitions(self) -> dict[str, Callable[[], bool]]:
        def T20_21() -> bool:
            return (
                self.ProductionEnable.active
                and self.RequestConveyorAccept.active
            )

        def T21_22() -> bool:
            return (
                self.ConveyorReady.active
                and self.HandoffPositionFree.active
                and not self.ConveyorFaultActive.active
            )

        def T21_26() -> bool:
            return (
                not self.HandoffPositionFree.active
                and not self.ConveyorFaultActive.active
            )

        def T21_27() -> bool:
            return (
                self.ConveyorFaultActive.active
                or not self.ConveyorReady.active
            )

        def T22_23() -> bool:
            return self.ConveyorAcceptedTray.active

        def T23_24() -> bool:
            return self.ConveyorRunning.active

        def T23_27() -> bool:
            return (
                self.ConveyorStartTimeOut.active
                or self.ConveyorFaultActive.active
            )

        def T24_25() -> bool:
            return self.TrayAtHandoff.active

        def T24_26() -> bool:
            return (
                self.TrayAtHandoff.active
                and not self.HandoffPositionFree.active
            )

        def T24_27() -> bool:
            return (
                self.TransportTimeOut.active
                or self.ConveyorFaultActive.active
            )

        def T25_20() -> bool:
            return self.TrayAtHandoff.active

        def T26_20() -> bool:
            return not self.ProductionEnable.active

        def T26_21() -> bool:
            return (
                self.HandoffPositionFree.active
                and self.ProductionEnable.active
            )

        def T27_20() -> bool:
            return (
                self.FaultCleared.active
                and self.ResetRequest.active
            )

        return {
            "T20_21": T20_21,
            "T21_22": T21_22,
            "T21_26": T21_26,
            "T21_27": T21_27,
            "T22_23": T22_23,
            "T23_24": T23_24,
            "T23_27": T23_27,
            "T24_25": T24_25,
            "T24_26": T24_26,
            "T24_27": T24_27,
            "T25_20": T25_20,
            "T26_20": T26_20,
            "T26_21": T26_21,
            "T27_20": T27_20,
        }

    def _create_actions(self) -> dict[str, Callable]:

        def _idle(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S20: Idle")

        def _check_permissives(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S21: CheckPermissives")




        def _accept_tray(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S22: AcceptTray")

        def _transport(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S23: Transport")

        def _wait_at_outfeed(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S24: WaitAtOutfeed")

        def _complete(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S25: Complete")

        def _blocked(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S26: Blocked")

        def _fault(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S27: Fault")

        return {
            "S20": _idle,
            "S21": _check_permissives,
            "S22": _accept_tray,
            "S23": _transport,
            "S24": _wait_at_outfeed,
            "S25": _complete,
            "S26": _blocked,
            "S27": _fault,
        }

    def _init_control(self) -> None:
        if self.init_flag:
            self.init_flag = False
            self.S20.activate()

    def _sequence_control(self) -> None:
        self.key_input.update()

        if self.S20.active and not self.S20.rising_edge and self.T["T20_21"]():
            self.S20.deactivate()
            self.S21.activate()

        elif self.S21.active and not self.S21.rising_edge:
            if self.T["T21_22"]():
                self.S21.deactivate()
                self.S22.activate()
            elif self.T["T21_26"]():
                self.S21.deactivate()
                self.S26.activate()
            elif self.T["T21_27"]():
                self.S21.deactivate()
                self.S27.activate()

        elif self.S22.active and self.T["T22_23"]():
            self.S22.deactivate()
            self.S23.activate()

        elif self.S23.active:
            if self.T["T23_24"]():
                self.S23.deactivate()
                self.S24.activate()
            elif self.T["T23_27"]():
                self.S23.deactivate()
                self.S27.activate()

        elif self.S24.active:
            if self.T["T24_25"]():
                self.S24.deactivate()
                self.S25.activate()
            elif self.T["T24_26"]():
                self.S24.deactivate()
                self.S26.activate()
            elif self.T["T24_27"]():
                self.S24.deactivate()
                self.S27.activate()

        elif self.S25.active and self.T["T25_20"]():
            self.S25.deactivate()
            self.S20.activate()

        elif self.S26.active:
            if self.T["T26_20"]():
                self.S26.deactivate()
                self.S20.activate()
            elif self.T["T26_21"]():
                self.S26.deactivate()
                self.S21.activate()

        elif self.S27.active and self.T["T27_20"]():
            self.S27.deactivate()
            self.S20.activate()

    def _execute_actions(self) -> None:
        if self.S20.active:
            self.A["S20"](self.S20)

        elif self.S21.active:
            self.A["S21"](self.S21)

        elif self.S22.active:
            self.A["S22"](self.S22)

        elif self.S23.active:
            self.A["S23"](self.S23)

        elif self.S24.active:
            self.A["S24"](self.S24)

        elif self.S25.active:
            self.A["S25"](self.S25)

        elif self.S26.active:
            self.A["S26"](self.S26)

        elif self.S27.active:
            self.A["S27"](self.S27)

    def control_routine(self) -> None:
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        pass

    def emergency_routine(self) -> None:
        pass

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        pass
