import time

from pyberryplc.utils.log_utils import init_logger
from pyberryplc.core import AbstractPLC, SharedData
from pyberryplc.stepper import TMC2208StepperMotor, TMC2208UART


logger = init_logger()


class XYMotionPLC(AbstractPLC):
    
    def __init__(
        self, 
        shared_data: SharedData, 
        left_port: str = "/dev/ttyUSB1",
        right_port: str = "/dev/ttyUSB0"
    ) -> None:
        super().__init__(shared_data=shared_data, logger=logger)

        self.motor_x = TMC2208StepperMotor(
            step_pin=21,
            dir_pin=27,
            microstep_resolution="1/64",
            uart=TMC2208UART(port=left_port),
            logger=self.logger,
            name="motor X"
        )

        self.motor_y = TMC2208StepperMotor(
            step_pin=19,
            dir_pin=20,
            microstep_resolution="1/64",
            uart=TMC2208UART(port=right_port),
            logger=self.logger,
            name="motor Y"
        )

        self._init_flag = False

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")

        self.start_motion = self.hmi_input_register["start_motion"]
        self.motor_x_busy = self.hmi_output_register["motor_x_busy"]
        self.motor_y_busy = self.hmi_output_register["motor_y_busy"]

    def _init_control(self):
        if not self._init_flag:
            self._init_flag = True
                       
            self.motor_x.enable()
            self.motor_x.set_microstepping()
            self.motor_x.set_current_via_uart(run_current_pct=19, hold_current_pct=10)
            
            self.motor_y.enable()
            self.motor_y.set_microstepping()
            self.motor_y.set_current_via_uart(run_current_pct=19, hold_current_pct=10)
            
            self.logger.info("Drivers enabled and configured.")
            self.X0.activate()
    
    def _sequence_control(self):
        # Update the `motor_busy` memory variable at each PLC scan cycle with
        # the current running state of the motor
        self.motor_x_busy.update(self.motor_x.busy)
        self.motor_y_busy.update(self.motor_y.busy)

        if self.X0.active and self.start_motion.active:
            self.logger.info("Start motion.")
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and self._motion_finished():
            self.logger.info("Motion finished.")
            self.X1.deactivate()
            self.logger.info("Waiting for next move command...")
            self.X0.activate()

    def _motion_finished(self) -> bool:
        c1 = not self.motor_x_busy.active and self.motor_y_busy.falling_edge
        c2 = not self.motor_y_busy.active and self.motor_x_busy.falling_edge
        if c1 or c2:
            return True
        return False
    
    def _execute_actions(self):
        if self.X1.rising_edge:
            mp_x = self.shared_data.hmi_data["motion_profile_x"]
            mp_y = self.shared_data.hmi_data["motion_profile_y"]
            self.motor_x.start_rotation_profile(mp_x)
            self.motor_y.start_rotation_profile(mp_y)
    
    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exit. Shutting down...")
        self.motor_x.disable()
        self.motor_y.disable()
        self.motor_x = self.motor_y = None

    def emergency_routine(self):
        self.logger.warning("Emergency stop. Shutting down...")
        self.motor_x.disable()
        self.motor_y.disable()
        self.motor_x = self.motor_y = None

    def crash_routine(self, exception):
        self.logger.error("PLC crashed. Shutting down...")
        self.motor_x.disable()
        self.motor_y.disable()
        self.motor_x = self.motor_y = None
        raise exception
