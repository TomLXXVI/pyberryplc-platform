from pyberryplc.stepper import TMC2208StepperMotor, PinConfig, MicrostepPinConfig, RotatorType
from pyberryplc.core import AbstractPLC
from pyberryplc.utils.keyboard_input import KeyInput
from pyberryplc.motion.profile import TrapezoidalProfile, RotationDirection


def create_stepper_motor(logger):
    stepper = TMC2208StepperMotor(
        pin_config=PinConfig(step_pin=21, dir_pin=27, en_pin=13),
        logger=logger,
        name="x-motor",
        uart=None
    )
    stepper.attach_rotator(RotatorType.MOTION_PROFILE)
    stepper.rotator.profile = TrapezoidalProfile(
        ds_tot=720.0,
        a_max=360.0,
        v_max=180.0,
        v_i=0.0,
        v_f=0.0
    )
    return stepper


class StepperPLC(AbstractPLC):

    def __init__(self, logger):
        super().__init__(logger=logger)

        self.key_input = KeyInput()
        self.stepper = create_stepper_motor(logger)

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")

        self.input_flag = True

    def _init_control(self):
        if self.input_flag:
            self.input_flag = False
            self.stepper.enable()
            self.stepper.configure_microstepping(
                resolution="1/2",
                ms_pins=MicrostepPinConfig(ms1_pin=20, ms2_pin=19)
            )
            self.X0.activate()

    def _sequence_control(self):
        self.key_input.update()

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
            self.stepper.rotator.direction = RotationDirection.CCW
            self.stepper.rotator.rotate()
            self.logger.info("Press 'r' to start motor in reverse")

        if self.X2.rising_edge:
            self.stepper.rotator.direction = RotationDirection.CW
            self.stepper.rotator.rotate()
            self.logger.info("Press 'q' to go back to idle")

    def control_routine(self):
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Exiting... disabling driver.")
        self.stepper.disable()  # Turn the driver back off.

    def emergency_routine(self):
        pass

    def crash_routine(self, exception: Exception) -> None:
        self.logger.error("PLC crashed! - Disabling driver.")
        self.stepper.disable()  # Turn the driver back off.
        raise exception

if __name__ == "__main__":
    import os
    from pyberryplc.utils.log_utils import init_logger

    os.system("clear")
    logger = init_logger(name="STEPPER PLC")
    logger.info("Run `stepper_motor03.py`")
    plc = StepperPLC(logger)
    plc.run()
