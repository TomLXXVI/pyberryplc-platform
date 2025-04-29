import sys
if (path := "/shared/python-projects/pyberryplc-platform") not in sys.path:
    sys.path.append(path)

import time
import random
import logging

from pyberryplc.core import SharedData, AbstractPLC
from plc_server.plc_server_framework import PLCServer


shared_data = SharedData(
    hmi_buttons={"start_motion": False},
    hmi_switches={"enable_system": False},
    hmi_analog_inputs={"speed_setpoint": 0.0},
    hmi_outputs={"motor_running": False, "alarm_active": False, "plc_fault": False},
    hmi_data={}
)


class StepperHMIControlledPLC(AbstractPLC):
    """Concrete PLC that reacts to HMI commands to start motor simulation."""

    def __init__(self, shared_data: SharedData, logger: logging.Logger) -> None:
        super().__init__(scan_time=0.1, shared_data=shared_data, logger=logger)
        self._motor_timer_start: float = 0.0
        self._motor_duration: float = 5.0
        self._motor_running_internal: bool = False

    def control_routine(self) -> None:
        """Process HMI input commands and simulate motor behavior."""
        # Check if HMI command 'start_motion' is activated
        if self.hmi_input_registry["start_motion"].rising_edge:
            self.logger.info('Motor started.')
            self._start_motor()

        if self._motor_running_internal:
            elapsed = time.monotonic() - self._motor_timer_start
            if elapsed >= self._motor_duration:
                self._stop_motor()

    def _start_motor(self) -> None:
        """Internal method to start motor simulation."""
        self._motor_timer_start = time.monotonic()
        self._motor_running_internal = True
        self.hmi_output_registry["motor_running"].update(True)

    def _stop_motor(self) -> None:
        """Internal method to stop motor simulation and possibly trigger alarm."""
        self._motor_running_internal = False
        self.hmi_output_registry["motor_running"].update(False)

        # Simuleer alarm met 20% kans
        if random.random() < 0.8:
            self.hmi_output_registry["alarm_active"].update(True)
            raise RuntimeError
        else:
            self.hmi_output_registry["alarm_active"].update(False)

    def exit_routine(self):
        pass

    def emergency_routine(self):
        pass

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.logger.critical("PLC crashed!!!")


if __name__ == '__main__':
    
    uvicorn_logger = logging.getLogger("uvicorn")
    
    plc = StepperHMIControlledPLC(shared_data=shared_data, logger=uvicorn_logger)
    
    server = PLCServer(shared_data=shared_data, plc_instance=plc)
    server.start_plc()
    server.run_server()
