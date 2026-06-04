from typing import Callable

import logging

from pyberryplc.core import (
    AbstractPLC, MemoryVariable, ToggleSwitch, SharedMemoryBlock,
    TimerOnDelay
)

from pyberryplc.utils.keyboard_input import KeyInput

from loading_station_client import LoadingStation
from remote_loading_station import Status


class LoadingStationPLC(AbstractPLC):

    def __init__(
        self,
        logger: logging.Logger,
        db0: SharedMemoryBlock,  # shared with main plc
        db1: SharedMemoryBlock,  # shared with infeed conveyor
    ) -> None:
        super().__init__(logger=logger)

        self.key_input = KeyInput()

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

    def _create_steps(self) -> None:
        self.S10 = self.add_marker("S10")
        self.S11 = self.add_marker("S11")
        self.S12 = self.add_marker("S12")
        self.S13 = self.add_marker("S13")
        self.S14 = self.add_marker("S14")
        self.S15 = self.add_marker("S15")
        self.S16 = self.add_marker("S16")
        self.S17 = self.add_marker("S17")

    def _create_key_switch(self, key: str) -> ToggleSwitch:
        self.key_input.keys[key] = MemoryVariable()
        switch = ToggleSwitch(self.key_input.keys[key])
        return switch

    def _create_key_button(self, key: str) -> MemoryVariable:
        self.key_input.keys[key] = MemoryVariable()
        return self.key_input.keys[key]

    def _create_variables(self) -> None:
        self.ProductionEnable = self.db0.data["ProductionEnable"]

        self.TrayToLoadAvailable = self._create_key_button("t")

        self.LoadingStationReady = MemoryVariable()
        self.LoadingStationBusy = MemoryVariable()
        self.LoadingStationBlocked = MemoryVariable()
        self.LoadingStationFaultActive = MemoryVariable()
        self.LoadingCycleStarted = MemoryVariable()
        self.LoadingCycleTimeout = MemoryVariable()
        self.TrayLoaded = MemoryVariable()
        self.TrayTransferRegistered = MemoryVariable()

        self.RequestConveyorAccept = self.db1.data["RequestConveyorAccept"]
        self.ConveyorReadyToAccept = self.db1.data["ConveyorReadyToAccept"]
        self.ConveyorAcceptedTray = self.db1.data["ConveyorAcceptedTray"]

        self.ConveyorAcceptTimeout = MemoryVariable()
        self.TimerConveyorAccept = TimerOnDelay(2.0)

        self.FaultCleared = MemoryVariable()
        self.ResetRequest: MemoryVariable = self._create_key_button("r")

    def _create_transitions(self) -> dict[str, Callable[[], bool]]:

        def T10_11() -> bool:
            return (
                self.ProductionEnable.active
                and not self.LoadingStationFaultActive.active
            )

        def T11_12() -> bool:
            return (
                self.LoadingStationReady.active
                and self.ConveyorReadyToAccept.active
                and self.TrayToLoadAvailable.active
                and not self.LoadingStationFaultActive.active
            )

        def T11_16() -> bool:
            return (
                not self.ConveyorReadyToAccept.active
                and not self.LoadingStationFaultActive.active
            )

        def T11_17() -> bool:
            return (
                self.LoadingStationFaultActive.active
                or not self.LoadingStationReady.active
            )

        def T12_13() -> bool:
            return self.LoadingCycleStarted.active

        def T13_14() -> bool:
            return self.TrayLoaded.active

        def T13_17() -> bool:
            return (
                self.LoadingCycleTimeout.active
                or self.LoadingStationFaultActive.active
            )

        def T14_15() -> bool:
            return self.ConveyorAcceptedTray.active

        def T14_16() -> bool:
            return (
                not self.ConveyorReadyToAccept.active
                and not self.ConveyorAcceptedTray.active
            )

        def T14_17() -> bool:
            return (
                self.ConveyorAcceptTimeout.active
                or self.LoadingStationFaultActive.active
            )

        def T15_10() -> bool:
            return self.TrayTransferRegistered.active

        def T16_11() -> bool:
            return (
                self.ConveyorReadyToAccept.active
                and self.ProductionEnable.active
            )

        def T16_10() -> bool:
            return not self.ProductionEnable.active

        def T17_10() -> bool:
            return self.FaultCleared.active and self.ResetRequest.active

        return {
            "T10_11": T10_11,
            "T11_12": T11_12,
            "T11_16": T11_16,
            "T11_17": T11_17,
            "T12_13": T12_13,
            "T13_14": T13_14,
            "T13_17": T13_17,
            "T14_15": T14_15,
            "T14_16": T14_16,
            "T14_17": T14_17,
            "T15_10": T15_10,
            "T16_10": T16_10,
            "T16_11": T16_11,
            "T17_10": T17_10,
        }

    def _create_actions(self) -> dict[str, Callable]:

        def reset(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S10: Idle")

            # Reset internal variables at the start of a new scan cycle.
            self.LoadingStationReady.update(False)
            self.LoadingCycleStarted.update(False)
            self.LoadingStationFaultActive.update(False)
            self.LoadingCycleTimeout.update(False)
            self.TrayLoaded.update(False)
            self.TrayTransferRegistered.update(False)
            self.FaultCleared.update(False)

        def check_permissives(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S11: CheckPermissives")

            _check_loading_station(step)
            _check_conveyor(step)

        def _check_loading_station(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S11: Check operational state of loading station")

                self.loading_station.check_operational_state()

            status, message = self.loading_station.get_response()
            match status:
                case Status.READY:
                    self.LoadingStationReady.update(True)
                    self.logger.info(f"Loading station says: {message}")
                case Status.ERROR:
                    self.LoadingStationFaultActive.update(True)
                    self.logger.error(f"Loading station says: {message}")

        def _check_conveyor(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S11: Check operational state of conveyor")

            self.ConveyorReadyToAccept.update(True)

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

            self.RequestConveyorAccept.update(True)  # request conveyor to accept tray

        def complete(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S15: Complete")

            self.RequestConveyorAccept.update(False)  # conveyor accepted tray: turn off the request
            self.TrayTransferRegistered.update(True)

        def blocked(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S16: Blocked")

        def fault(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S17: Fault")

            self.FaultCleared.update(True)

        return {
            "S10": reset,
            "S11": check_permissives,
            "S12": load_tray,
            "S13": wait_loaded,
            "S14": offer_to_conveyor,
            "S15": complete,
            "S16": blocked,
            "S17": fault,
        }

    def _init_control(self) -> None:
        if self.init_flag:
            self.init_flag = False

            self.logger.info("Connect to loading station.")
            self.loading_station.connect()

            self.S10.activate()

    def _sequence_control(self) -> None:
        self.key_input.update()

        if self.S10.active and self.T["T10_11"]():
            self.S10.deactivate()
            self.S11.activate()

        elif self.S11.active and not self.S11.rising_edge:  # make sure action S11 is executed at least once
            if self.T["T11_12"]():
                self.S11.deactivate()
                self.S12.activate()
            elif self.T["T11_16"]():
                self.S11.deactivate()
                self.S16.activate()
            elif self.T["T11_17"]():
                self.S11.deactivate()
                self.S17.activate()

        elif self.S12.active and self.T["T12_13"]():
            self.S12.deactivate()
            self.S13.activate()

        elif self.S13.active:
            if self.T["T13_14"]():
                self.S13.deactivate()
                self.S14.activate()
            elif self.T["T13_17"]():
                self.S13.deactivate()
                self.S17.activate()

        elif self.S14.active:
            if self.TimerConveyorAccept.has_elapsed:
                self.ConveyorAcceptTimeout.update(True)
            if self.T["T14_15"]() or self.T["T14_16"]() or self.T["T14_17"]():
                self.S14.deactivate()
                self.TimerConveyorAccept.reset()
                self.ConveyorAcceptTimeout.update(False)
                if self.T["T14_15"]():
                    self.S15.activate()
                elif self.T["T14_16"]():
                    self.S16.activate()
                elif self.T["T14_17"]():
                    self.S17.activate()

        elif self.S15.active and self.T["T15_10"]():
            self.S15.deactivate()
            self.S10.activate()

        elif self.S16.active:
            if self.T["T16_11"]():
                self.S16.deactivate()
                self.S11.activate()
            elif self.T["T16_10"]():
                self.S16.deactivate()
                self.S10.activate()

        elif self.S17.active and self.T["T17_10"]():
            self.S17.deactivate()
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

    def control_routine(self) -> None:
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        try:
            self.loading_station.shutdown()
        except Exception:
            self.logger.warning("Failure to send shutdown to loading station.")
        self.loading_station.close()

    def emergency_routine(self) -> None:
        self.exit_routine()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()


def main():
    import os
    import subprocess

    from pyberryplc.utils.log_utils import init_logger

    from datablocks import db0, db1

    os.system("clear")

    project_path = "/shared/python-projects/pyberryplc-platform/demos/demo_12"
    venv_activate = "/shared/python-projects/pyberryplc-platform/.venv/bin/activate"
    loading_station_script = os.path.join(project_path, "remote_loading_station.py")
    subprocess.Popen([
        'bash', '-c',
        f'source {venv_activate} && python {loading_station_script}'
    ])

    logger = init_logger("PLC")
    plc = LoadingStationPLC(logger, db0, db1)
    plc.run()


if __name__ == '__main__':
    main()
