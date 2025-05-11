import os
from pyberryplc.core import AbstractPLC
from pyberryplc.stepper import (
    TMC2208StepperMotor, 
    TMC2208UART,
    PinConfig,
    RotatorType,
    Direction
)
from pyberryplc.motion import TrapezoidalProfile
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput


class StepperUARTTestPLC(AbstractPLC):

    def __init__(self, logger):
        super().__init__(logger=logger)

        self.key_input = KeyInput()

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
        self.stepper.direction = Direction.COUNTERCLOCKWISE
        self.stepper.rotator.profile = TrapezoidalProfile(
            ds_tot=180,
            dt_tot=1,
            dt_acc=0.25
        )

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")

        self.input_flag = True

    def _init_control(self):
        if self.input_flag:
            self.input_flag = False

            # Stepper driver configuration via UART
            self.stepper.enable(high_sensitivity=True)
            self.stepper.configure_microstepping(
                resolution="1/16",
                ms_pins=None,
                full_steps_per_rev=200
            )
            self.stepper.set_current_via_uart(
                run_current_pct=35.0,
                hold_current_pct=10.0
            )

            self.X0.activate()

    def _sequence_control(self):
        if not self.stepper.rotator.busy:
            if self.X0.active and self.key_input.rising_edge("s"):
                self.logger.info("Start: rotating forward")
                self.X0.deactivate()
                self.X1.activate()

            if self.X1.active and self.key_input.rising_edge("r"):
                self.logger.info("Reverse: rotating backward")
                self.X1.deactivate()
                self.X2.activate()

            if self.X2.active and self.key_input.is_pressed("q"):
                self.logger.info("Back to idle")
                self.X2.deactivate()
                self.X0.activate()

    def _execute_actions(self):
        if self.X0.rising_edge:
            self.logger.info("Press 's' to start motor")

        if self.X1.rising_edge:
            self.stepper.rotator.direction = Direction.COUNTERCLOCKWISE
            self.stepper.rotator.start()
            self.logger.info("Press 'r' to start motor in reverse")

        if self.X2.rising_edge:
            self.stepper.rotator.direction = Direction.CLOCKWISE
            self.stepper.rotator.start()
            self.logger.info("Press 'q' to go back to idle")

    def control_routine(self):
        self.key_input.update()
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exiting... disabling driver.")
        self.stepper.disable()

    def emergency_routine(self):
        pass

    def crash_routine(self, exception: Exception) -> None:
        self.logger.error("PLC crashed! - Disabling driver.")
        self.stepper.disable()


if __name__ == "__main__":
    os.system("clear")
    init_logger()
    logger = init_logger(name="STEPPER PLC")
    logger.info("Run `stepper_motor05.py`")
    plc = StepperUARTTestPLC(logger)
    plc.run()
