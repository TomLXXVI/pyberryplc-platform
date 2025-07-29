from pyberryplc.motion import SCurvedProfile, RotationDirection
from pyberryplc.core import AbstractPLC
from pyberryplc.stepper import (
    TMC2208StepperMotor,
    PinConfig,
    TMC2208UART,
    RotatorType
)
from pyberryplc.utils.keyboard_input import KeyInput


def create_motion_profile():
    mp = SCurvedProfile(
        ds_tot=720.0,  # deg
        a_max=360.0,   # deg/s2
        v_max=180.0,   # deg/s
        v_i=0.0,
        v_f=0.0
    )
    return mp


class SingleStepperPLC(AbstractPLC):

    def __init__(self, logger):
        super().__init__(logger=logger)
        self.key_input = KeyInput()
        self.stepper = self._create_stepper()
        self.init_flag = True

        self.L1, _ = self.add_digital_output(pin=13, label="L1")
        self.L2, _ = self.add_digital_output(pin=26, label="L2")

        self.X0 = self.add_marker("X0")
        self.X11 = self.add_marker("X11")
        self.X12 = self.add_marker("X12")
        self.X21 = self.add_marker("X21")
        self.X22 = self.add_marker("X22")
        self.X3 = self.add_marker("X3")

    def _create_stepper(self) -> TMC2208StepperMotor:
        stepper = TMC2208StepperMotor(
            pin_config=PinConfig(step_pin=21, dir_pin=27),
            logger=self.logger,
            name="x-axis",
            uart=TMC2208UART(port="/dev/ttyUSB0")
        )
        stepper.attach_rotator(RotatorType.DYNAMIC_THREADED)
        stepper.rotator.profile = create_motion_profile()
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
                self.X11.activate()
            if self.key_input.rising_edge("b"):
                self.X0.deactivate()
                self.X21.activate()

        if self.X11.active and self.key_input.rising_edge("s"):
            self.logger.info("Start forward motion.")
            self.X11.deactivate()
            self.X12.activate()

        if self.X21.active and self.key_input.rising_edge("s"):
            self.logger.info("Start backward motion.")
            self.X21.deactivate()
            self.X22.activate()

        if self.X12.active and self.key_input.rising_edge("e"):
            self.logger.info("Stop forward motion.")
            self.X12.deactivate()
            self.X3.activate()

        if self.X22.active and self.key_input.rising_edge("e"):
            self.logger.info("Stop backward motion.")
            self.X22.deactivate()
            self.X3.activate()

        if self.X3.active and not self.stepper.rotator.busy:
            self.X3.deactivate()
            self.X0.activate()

    def _execute_actions(self):
        if self.X0.rising_edge:
            self.logger.info(
                "Press f-key for forward motion, "
                "or b-key for backward motion."
            )

        if self.X11.rising_edge:
            self.logger.info(
                "Press s-key to start forward motion."
            )

        if self.X12.rising_edge:
            self.stepper.rotator.direction = RotationDirection.CCW
            self.stepper.rotator.start()
            self.L1.activate()
            self.L2.deactivate()
            self.logger.info(
                "Press e-key to stop forward motion."
            )

        if self.X21.rising_edge:
            self.logger.info(
                "Press s-key to start backward motion."
            )

        if self.X22.rising_edge:
            self.stepper.rotator.direction = RotationDirection.CW
            self.stepper.rotator.start()
            self.L2.activate()
            self.L1.deactivate()
            self.logger.info(
                "Press e-key to stop backward motion."
            )

        if self.X3.rising_edge:
            self.stepper.rotator.stop()
            self.L1.deactivate()
            self.L2.deactivate()

    def control_routine(self) -> None:
        self._init()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        self.stepper.disable()
        self.L1.deactivate()
        self.L2.deactivate()

    def emergency_routine(self) -> None:
        self.exit_routine()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
        raise exception


if __name__ == '__main__':

    import os

    from pyberryplc.utils.log_utils import init_logger

    os.system("clear")
    logger = init_logger(name="Demo 4")
    logger.info("Starting demo 4...")
    plc = SingleStepperPLC(logger)
    plc.run()
