from logging import Logger

from pyberryplc.core import AbstractPLC, SharedData, MemoryVariable
from pyberryplc.stepper import TMC2208StepperMotor, TMC2208UART


class StepperMotorPLC(AbstractPLC):
    
    def __init__(self, shared_data: SharedData, logger: Logger):
        super().__init__(shared_data=shared_data, logger=logger)

        self.motor = TMC2208StepperMotor(
            step_pin=27,
            dir_pin=26,
            microstep_resolution="1/4",
            uart=TMC2208UART(port="/dev/ttyAMA0"),
            logger=self.logger,
        )
        
        self._init_flag = False
        
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")

        self.start_motion = self.hmi_input_register["start_motion"]
        self.motor_busy = self.hmi_output_register["motor_busy"]
    
    def _init_control(self):
        if not self._init_flag:
            self._init_flag = True

            self.motor.enable()
            self.motor.set_microstepping()
            self.motor.set_current_via_uart(run_current_pct=19, hold_current_pct=10)

            self.logger.info("Driver enabled and configured.")

            self.X0.activate()
    
    def _sequence_control(self):
        # Update the `motor_busy` memory variable at each PLC scan cycle with
        # the current running state of the motor
        self.motor_busy.update(self.motor.busy)
        
        if self.X0.active and self.start_motion.active:
            self.logger.info("Start motion.")
            self.X0.deactivate()
            self.X1.activate()
        
        if self.X1.active and self.motor_busy.falling_edge:
            self.logger.info("Motion finished.")
            self.X1.deactivate()
            self.logger.info("Waiting for next move command...")
            self.X0.activate()
        
    def _execute_actions(self):
        if self.X1.active:
            if self.X1.rising_edge:
                motion_profile = self.shared_data.hmi_data["motion_profile"]
                self.motor.start_rotation_profile(motion_profile)
        
        if self.X2.rising_edge:
            self.exit()
    
    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exit PLC routine. Shutting down...")
        self.motor.disable()
        self.motor = None

    def emergency_routine(self):
        self.logger.warning("Emergency stop invoked. Shutting down...")
        self.motor.disable()
        self.motor = None

    def crash_routine(self, exception):
        self.logger.error("PLC crashed. Shutting down...")
        self.motor.disable()
        self.motor = None
        raise exception
