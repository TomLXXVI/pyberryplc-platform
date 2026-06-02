import logging

from pyberryplc.core import AbstractPLC, MemoryVariable, ToggleSwitch, EmergencyException
from pyberryplc.utils.keyboard_input import KeyInput

from loading_station_client import LoadingStation
from remote_loading_station import Status


class LoadingStationPLC(AbstractPLC):

    def __init__(self, logger: logging.Logger):
        super().__init__(logger=logger)
        self.key_input = KeyInput()

        self.init_flag: bool = True

        # Steps
        self.S10 = self.add_marker("S10")
        self.S11 = self.add_marker("S11")
        self.S12 = self.add_marker("S12")
        self.S13 = self.add_marker("S13")
        self.S14 = self.add_marker("S14")
        self.S15 = self.add_marker("S15")
        self.S16 = self.add_marker("S16")
        self.S17 = self.add_marker("S17")

        # Inputs
        self.ProductionEnable: ToggleSwitch = self._create_key_switch("p")
        self.TrayToLoadAvailable: MemoryVariable = self._create_key_button("t")

        self.LoadingStationReady: bool = False
        self.LoadingStationBusy: bool = False
        self.LoadingStationBlocked: bool = False
        self.LoadingStationFaultActive: bool = False
        self.LoadingCycleStarted: bool = False
        self.LoadingCycleTimeout: bool = False

        self.TrayLoaded: bool = False
        self.TrayTransferRegistered: bool = False

        self.ConveyorReadyToAccept: bool = False
        self.ConveyorAcceptedTray: bool = False
        self.ConveyorAcceptTimeout: bool = False

        self.FaultCleared: bool = False
        self.ResetRequest: bool = False

        # Outputs
        self.StartLoadingCycle: bool = False
        self.RequestConveyorAccept: bool = False

        # Remote device clients
        self.loading_station = LoadingStation(logger=self.logger)

    def _create_key_switch(self, key: str) -> ToggleSwitch:
        self.key_input.keys[key] = MemoryVariable()
        switch = ToggleSwitch(self.key_input.keys[key])
        return switch

    def _create_key_button(self, key: str) -> MemoryVariable:
        self.key_input.keys[key] = MemoryVariable()
        return self.key_input.keys[key]

    def _T10_11(self) -> bool:
        return (
            self.ProductionEnable.active
            and self.TrayToLoadAvailable.active
            and not self.LoadingStationFaultActive
        )

    def _T11_12(self) -> bool:
        return (
            self.LoadingStationReady
            and self.ConveyorReadyToAccept
            and not self.LoadingStationFaultActive
        )

    def _T11_16(self) -> bool:
        return (
            not self.ConveyorReadyToAccept
            and not self.LoadingStationFaultActive
        )

    def _T11_17(self) -> bool:
        return (
            self.LoadingStationFaultActive
            or not self.LoadingStationReady
        )

    def _T12_13(self) -> bool:
        return self.LoadingCycleStarted

    def _T13_14(self) -> bool:
        return self.TrayLoaded

    def _T13_17(self) -> bool:
        return (
            self.LoadingCycleTimeout
            or self.LoadingStationFaultActive
        )

    def _T14_15(self) -> bool:
        return self.ConveyorAcceptedTray

    def _T14_16(self) -> bool:
        return (
            not self.ConveyorReadyToAccept
            and not self.ConveyorAcceptedTray
        )

    def _T14_17(self) -> bool:
        return (
            self.ConveyorAcceptTimeout
            or self.LoadingStationFaultActive
        )

    def _T15_10(self) -> bool:
        return self.TrayTransferRegistered

    def _T16_11(self) -> bool:
        return self.ConveyorReadyToAccept and self.ProductionEnable.active

    def _T16_10(self) -> bool:
        return not self.ProductionEnable.active

    def _T17_10(self) -> bool:
        return self.FaultCleared and self.ResetRequest

    def reset(self) -> None:
        self.LoadingStationReady = False
        self.LoadingStationFaultActive = False
        self.LoadingCycleStarted = False
        self.TrayLoaded = False
        self.TrayTransferRegistered = False
        self.ConveyorReadyToAccept = False
        self.ConveyorAcceptedTray = False

    def check_permissives(self, step: MemoryVariable, step_id: str) -> None:
        self._check_loading_station(step, step_id)
        self._check_conveyor()

    def _check_loading_station(self, step: MemoryVariable, step_id: str) -> None:
        if step.rising_edge:
            self.logger.info("Check loading station status.")
            self.loading_station.get_status(step_id)

        status, message = self.loading_station.handle_response()
        match status:
            case Status.READY:
                self.LoadingStationReady = True
                self.logger.info(f"Loading station says: {message}")
            case Status.ERROR:
                self.LoadingStationFaultActive = True
                self.logger.error(f"Error while waiting for loading station: {message}")

    def _check_conveyor(self) -> None:
        self.ConveyorReadyToAccept = True

    def start_loading_cycle(self, step: MemoryVariable) -> None:
        if step.rising_edge:
            self.logger.info("Start loading cycle.")
            self.loading_station.start()

        status, message = self.loading_station.handle_response()
        match status:
            case Status.BUSY:
                self.LoadingCycleStarted = True
                self.logger.info(f"Loading station says: {message}")

    def wait_loading_cycle_finish(self, step: MemoryVariable, step_id: str) -> None:
        if step.rising_edge:
            self.logger.info("Wait for loading cycle to finish.")

        self.loading_station.get_status(step_id)

        status, message = self.loading_station.handle_response()
        match status:
            case Status.DONE:
                self.TrayLoaded = True
                self.logger.info(f"Loading station says: {message}")
            case Status.ERROR:
                self.LoadingStationFaultActive = True
                self.logger.error(f"Error while waiting for loading station: {message}")

    def offer_to_conveyor(self, step: MemoryVariable) -> None:
        if step.rising_edge:
            self.logger.info("Offer tray to conveyor.")
        self.ConveyorAcceptedTray = True

    def register_tray_status(self, step: MemoryVariable) -> None:
        if step.rising_edge:
            self.logger.info("Register tray to MES.")
        self.TrayTransferRegistered = True

    def block_station(self, step: MemoryVariable) -> None:
        if step.rising_edge:
            self.logger.info("Block station.")

    def handle_fault(self, step: MemoryVariable) -> None:
        if step.rising_edge:
            self.logger.info("Fault detected.")
        raise EmergencyException

    def _init_control(self) -> None:
        if self.init_flag:
            self.init_flag = False

            self.logger.info("Connect to loading station.")
            self.loading_station.connect()

            self.S10.activate()

    def _sequence_control(self) -> None:
        self.key_input.update()
        self.ProductionEnable.update()

        if self.S10.active and self._T10_11():
            self.S10.deactivate()
            self.S11.activate()

        if self.S11.active and not self.S11.rising_edge:
            if self._T11_12():
                self.S11.deactivate()
                self.S12.activate()
            if self._T11_16():
                self.S11.deactivate()
                self.S16.activate()
            if self._T11_17():
                self.S11.deactivate()
                self.S17.activate()

        if self.S12.active and self._T12_13():
            self.S12.deactivate()
            self.S13.activate()

        if self.S13.active:
            if self._T13_14():
                self.S13.deactivate()
                self.S14.activate()
            if self._T13_17():
                self.S13.deactivate()
                self.S17.activate()

        if self.S14.active:
            if self._T14_15():
                self.S14.deactivate()
                self.S15.activate()
            if self._T14_16():
                self.S14.deactivate()
                self.S16.activate()
            if self._T14_17():
                self.S14.deactivate()
                self.S17.activate()

        if self.S15.active and self._T15_10():
                self.S15.deactivate()
                self.S10.activate()

        if self.S16.active:
            if self._T16_11():
                self.S16.deactivate()
                self.S11.activate()
            if self._T16_10():
                self.S16.deactivate()
                self.S10.activate()

        if self.S17.active and self._T17_10():
            self.S17.deactivate()
            self.S10.activate()

    def _execute_actions(self) -> None:
        if self.S10.active:
            self.reset()

        if self.S11.active:
            self.check_permissives(self.S11, "S11")

        if self.S12.active:
            self.start_loading_cycle(self.S12)

        if self.S13.active:
            self.wait_loading_cycle_finish(self.S13, "S13")

        if self.S14.active:
            self.offer_to_conveyor(self.S14)

        if self.S15.active:
            self.register_tray_status(self.S15)

        if self.S16.active:
            self.block_station(self.S16)

        if self.S17.active:
            self.handle_fault(self.S17)

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

    os.system("clear")

    project_path = "/shared/python-projects/pyberryplc-platform/demos/demo_12"
    venv_activate = "/shared/python-projects/pyberryplc-platform/.venv/bin/activate"
    loading_station_script = os.path.join(project_path, "remote_loading_station.py")
    subprocess.Popen([
        'bash', '-c',
        f'source {venv_activate} && python {loading_station_script}'
    ])

    logger = init_logger("PLC")
    plc = LoadingStationPLC(logger)
    plc.run()


if __name__ == '__main__':
    main()
