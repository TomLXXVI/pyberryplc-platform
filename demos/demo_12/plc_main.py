from typing import Callable
import threading

from pyberryplc.core import (
    AbstractPLC,
    SoftMachineState,
    SoftwareBackend,
    MemoryVariable,
    EmergencyException,
)
from pyberryplc.utils.log_utils import init_logger

from datablocks import db0, db1
from plc_loading_station import LoadingStationPLC
from plc_conveyor import InfeedConveyorPLC


class MainPLC(AbstractPLC):

    def __init__(
        self, 
        main_state: SoftMachineState,
        loading_station_state: SoftMachineState,
        infeed_conveyor_state: SoftMachineState,
    ) -> None:
        super().__init__(
            logger=init_logger("MAIN PLC", log_file="logs/main_plc.log", console=False),
            io_backend=SoftwareBackend(main_state)
        )

        self.init_flag = True

        self.db0 = db0

        self.loading_station_plc = LoadingStationPLC(
            logger=init_logger("LOADING STATION PLC", log_file="logs/loading_plc.log", console=False),
            soft_machine_state=loading_station_state,
            db0=db0,
            db1=db1
        )
        self.loading_station_thread = threading.Thread(target=self.loading_station_plc.run)

        self.infeed_conveyor_plc = InfeedConveyorPLC(
            logger=init_logger("INFEED CONVEYOR PLC", log_file="logs/infeed_conveyor_plc.log", console=False),
            soft_machine_state=infeed_conveyor_state,
            db0=db0,
            db1=db1
        )
        self.infeed_conveyor_thread = threading.Thread(target=self.infeed_conveyor_plc.run)

        self._create_variables()

        self._create_steps()
        self.T = self._create_transitions()
        self.A = self._create_actions()

    def _create_variables(self) -> None:
        self.StartButton = self.add_digital_input("I00", "StartButton")
        self.StopButton = self.add_digital_input("I01", "StopButton")
        self.ExitButton = self.add_digital_input("I02", "ExitButton")
        self.ResetButton = self.add_digital_input("I03", "ResetButton")
        self.EmergencyButton = self.add_digital_input("I08", "EmergencyButton", NC_contact=True)

        self.ProductionEnable = self.db0.data["ProductionEnable"]
        self.Exit = self.db0.data["Exit"]

    def _create_steps(self) -> None:
        self.S0 = self.add_marker("S0")
        self.S1 = self.add_marker("S1")

    def _create_transitions(self) -> dict[str, Callable[[], bool]]:

        def T0_1() -> bool:
            return self.StartButton.active

        def T1_0() -> bool:
            return self.StopButton.active

        return {
            "T0_1": T0_1,
            "T1_0": T1_0,
        }

    def _create_actions(self) -> dict[str, Callable[[MemoryVariable], None]]:

        def disabled(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S0: Disabled")

            self.ProductionEnable.update(False)

        def enabled(step: MemoryVariable) -> None:
            if step.rising_edge:
                self.logger.info("S1: Enabled")

            self.ProductionEnable.update(True)

        return {
            "S0": disabled,
            "S1": enabled,
        }

    def _init_control(self) -> None:
        if self.init_flag:
            self.logger.info("Init main PLC")
            self.init_flag = False
            
            self.loading_station_thread.start()
            self.infeed_conveyor_thread.start()

            self.S0.activate()

    def _sequence_control(self) -> None:
        if self.ExitButton.active:
            self.logger.info("Received ExitButton. Closing down the system.")
            self.Exit.update(True)
            self.exit()
        
        if self.S0.active and self.T["T0_1"]():
            self.S0.deactivate()
            self.S1.activate()
        elif self.S1.active and self.T["T1_0"]():
            self.S1.deactivate()
            self.S0.activate()

    def _execute_actions(self) -> None:
        if self.S0.active:
            self.A["S0"](self.S0)
        elif self.S1.active:
            self.A["S1"](self.S1)

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
        self.loading_station_thread.join()
        self.infeed_conveyor_thread.join()

    def emergency_routine(self) -> None:
        self.logger.critical("MAIN PLC: Emergency state. Waiting for child threads to terminate safely.")
        self.loading_station_thread.join(timeout=1.0)
        self.infeed_conveyor_thread.join(timeout=1.0)

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.logger.critical(f"PLC crash: {exception}")
        self.emergency_routine()
        raise exception


def main():
    import os
    import subprocess
    import sys
    from pyberryplc.soft_machine import SoftMachine
    
    main_state = SoftMachineState()
    loading_station_state = SoftMachineState()
    infeed_conveyor_state = SoftMachineState()

    project_path = os.path.dirname(os.path.abspath(__file__))
    loading_station_script = os.path.join(project_path, "remote_loading_station.py")
    subprocess.Popen(
        [sys.executable, loading_station_script],
        cwd=project_path,
        creationflags=(
            subprocess.CREATE_NEW_CONSOLE
            if os.name == "nt"
            else 0
        ),
    )

    main_plc = MainPLC(
        main_state,
        loading_station_state, 
        infeed_conveyor_state
    )
    main_plc_thread = threading.Thread(target=main_plc.run, daemon=True)
    main_plc_thread.start()
    
    soft_machine = SoftMachine(
        states={
            "main": main_state,
            "loading_station": loading_station_state,
            "infeed": infeed_conveyor_state
        },
        datablocks={"DB0": db0, "DB1": db1},
        logger={
            "main": main_plc.logger,
            "loading_station": main_plc.loading_station_plc.logger,
            "infeed": main_plc.infeed_conveyor_plc.logger,
        },
        on_exit=main_plc.exit
    )
    soft_machine.run()


if __name__ == '__main__':
    main()
