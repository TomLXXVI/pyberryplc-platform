from logging import Logger

from pyberryplc.core import AbstractPLC, SharedData
from pyberryplc.stepper import (
    TMC2208StepperMotor, 
    PinConfig,
    TMC2208UART,
    RotatorType,
)


class StepperMotorPLC(AbstractPLC):
    
    def __init__(self, shared_data: SharedData, logger: Logger):
        super().__init__(shared_data=shared_data, logger=logger)

        # Set up stepper motor driver
        self.stepper = TMC2208StepperMotor(
            pin_config=PinConfig(
                step_pin_number=21,
                dir_pin_number=27
            ),
            logger=self.logger,
            name="motor X",
            uart=TMC2208UART(port="/dev/ttyUSB1")
        )
        self.stepper.attach_rotator(RotatorType.PROFILE_THREADED)
        
        self._init_flag = False
        
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")

        self.start_motion = self.hmi_input_register["start_motion"]
        self.motor_busy = self.hmi_output_register["motor_busy"]
    
    def _init_control(self):
        if not self._init_flag:
            self._init_flag = True

            # Stepper driver configuration via UART
            self.stepper.enable(high_sensitivity=True)
            self.stepper.configure_microstepping(
                resolution="1/4",
                ms_pins=None,
                full_steps_per_rev=200
            )
            self.stepper.set_current_via_uart(
                run_current_pct=35.0,
                hold_current_pct=10.0
            )

            self.logger.info("Driver enabled and configured.")

            self.X0.activate()
    
    def _sequence_control(self):
        # Update the `motor_busy` memory variable at each PLC scan cycle with
        # the current running state of the motor
        self.motor_busy.update(self.stepper.rotator.busy)
        
        if self.X0.active and self.start_motion.active:
            self.logger.info("Starting motion...")
            self.X0.deactivate()
            self.X1.activate()
        
        if self.X1.active and self.motor_busy.falling_edge:
            self.logger.info("Motion finished.")
            self.X1.deactivate()
            self.logger.info("Waiting for a new move command...")
            self.X0.activate()
        
    def _execute_actions(self):
        if self.X1.rising_edge:
            self.stepper.rotator.profile = self.shared_data.hmi_data["profile"]
            self.stepper.rotator.start()
    
    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exit PLC routine. Shutting down...")
        self.stepper.disable()
        self.stepper = None

    def emergency_routine(self):
        self.logger.warning("Emergency stop invoked. Shutting down...")
        self.stepper.disable()
        self.stepper = None

    def crash_routine(self, exception):
        self.logger.error("PLC crashed. Shutting down...")
        self.stepper.disable()
        self.stepper = None
        raise exception
