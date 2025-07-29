from pyberryplc.core import AbstractPLC
from pyberryplc.motion import RotationDirection
from pyberryplc.stepper import (
    TMC2208StepperMotor,
    PinConfig,
    TMC2208UART,
    RotatorType
)
from pyberryplc.utils.keyboard_input import KeyInput


class SingleStepperPLC(AbstractPLC):

    def __init__(self, logger):
        super().__init__(logger=logger)
        self.key_input = KeyInput()
        self.stepper = self._create_stepper()
        self.init_flag = True
        self.completed_flag = False
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")

    def _create_stepper(self) -> TMC2208StepperMotor:
        stepper = TMC2208StepperMotor(
            pin_config=PinConfig(step_pin=21, dir_pin=27),
            logger=self.logger,
            name="x-axis",
            uart=TMC2208UART(port="/dev/ttyUSB0")
        )
        stepper.attach_rotator(RotatorType.FIXED)
        stepper.rotator.angle = 720.0  # deg
        stepper.rotator.omega = 180.0  # deg/s
        return stepper

    def _configure_stepper(self):
        self.stepper.enable()
        self.stepper.configure_microstepping(
            resolution="1/2",
            full_steps_per_rev=200
        )
        self.stepper.set_current_via_uart(
            run_current_pct=35,
            hold_current_pct=10
        )

    def _init(self):
        if self.init_flag:
            self.init_flag = False
            self._configure_stepper()
            self.X0.activate()

    def _sequence_control(self):
        self.key_input.update()

        if self.X0.active:
            if self.key_input.rising_edge("f"):
                self.X0.deactivate()
                self.X1.activate()
            if self.key_input.rising_edge("b"):
                self.X0.deactivate()
                self.X2.activate()

        if self.X1.active and self.completed_flag:
            self.logger.info("Forward motion finished.")
            self.completed_flag = False
            self.X1.deactivate()
            self.X0.activate()

        if self.X2.active and self.completed_flag:
            self.logger.info("Backward motion finished.")
            self.completed_flag = False
            self.X2.deactivate()
            self.X0.activate()

    def _execute_actions(self):
        if self.X0.rising_edge:
            self.logger.info(
                "Press f-key for forward motion, "
                "or b-key for backward motion."
            )
        
        if self.X1.rising_edge:
            self.stepper.rotator.direction = RotationDirection.CCW
            self.stepper.rotator.rotate()  # blocking function
            self.completed_flag = True

        if self.X2.rising_edge:
            self.stepper.rotator.direction = RotationDirection.CW
            self.stepper.rotator.rotate()  # blocking function
            self.completed_flag = True

    def control_routine(self) -> None:
        self._init()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        self.stepper.disable()

    def emergency_routine(self) -> None:
        self.exit_routine()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
        raise exception


if __name__ == '__main__':

    import os

    from pyberryplc.utils.log_utils import init_logger

    os.system("clear")
    logger = init_logger(name="demo1")
    logger.info("Starting demo 1...")
    plc = SingleStepperPLC(logger)
    plc.run()
