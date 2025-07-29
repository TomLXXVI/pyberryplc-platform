from pyberryplc.motion import (
    RotationDirection,
    SCurvedProfile
)
from pyberryplc.stepper import (
    TMC2208StepperMotor,
    PinConfig,
    TMC2208UART,
    RotatorType
)
from pyberryplc.core import AbstractPLC
from pyberryplc.utils.keyboard_input import KeyInput


def create_motion_profile():
    mp = SCurvedProfile(
        ds_tot=720.0,
        a_max=360.0,
        v_max=180.0,
        v_i=0.0,
        v_f=0.0
    )
    return mp


def create_stepper_x(logger):
    stepper_x = TMC2208StepperMotor(
        pin_config=PinConfig(step_pin=27, dir_pin=17, use_pigpio=True),
        logger=logger,
        name="x-axis",
        uart=TMC2208UART(port="/dev/ttyUSB1", logger=logger)
    )
    stepper_x.attach_rotator(RotatorType.DYNAMIC_THREADED)
    stepper_x.rotator.profile = create_motion_profile()
    return stepper_x


def create_stepper_y(logger):
    stepper_y = TMC2208StepperMotor(
        pin_config=PinConfig(step_pin=24, dir_pin=23, use_pigpio=True),
        logger=logger,
        name="y-axis",
        uart=TMC2208UART(port="/dev/ttyUSB2", logger=logger)
    )
    stepper_y.attach_rotator(RotatorType.DYNAMIC_THREADED)
    stepper_y.rotator.profile = create_motion_profile()
    return stepper_y


def create_stepper_z(logger):
    stepper_z = TMC2208StepperMotor(
        pin_config=PinConfig(step_pin=6, dir_pin=5, use_pigpio=True),
        logger=logger,
        name="z-axis",
        uart=TMC2208UART(port="/dev/ttyUSB0", logger=logger)
    )
    stepper_z.attach_rotator(RotatorType.DYNAMIC_THREADED)
    stepper_z.rotator.profile = create_motion_profile()
    return stepper_z


def configure_stepper(stepper: TMC2208StepperMotor):
    stepper.enable()
    stepper.configure_microstepping(
        resolution="1/2",
        full_steps_per_rev=200
    )
    stepper.set_current_via_uart(
        run_current_pct=77,
        hold_current_pct=10
    )


class JogSubroutine:
    
    def __init__(self, main: 'MultiAxisPLC', axis: str):
        self.main = main
        self.logger = main.logger
        self.key_input = main.key_input

        self.stepper = None
        match axis:
            case "x":
                self.stepper = main.stepper_x
            case "y":
                self.stepper = main.stepper_y
            case "z":
                self.stepper = main.stepper_z

        self.X_00 = main.X_00
        self.X_11 = main.X_11
        self.X_21 = main.X_21
        self.X_01 = main.X_01

        step_markers = [
            self.X_00,
            self.X_11, self.X_21,
            self.X_01
        ]
        for step_marker in step_markers:
            step_marker.deactivate()

    def start(self):
        self.X_00.activate()

    def sequence_control(self):
        if self.X_00.active:
            if self.key_input.is_pressed("f"):
                self.X_00.deactivate()
                self.X_11.activate()
            if self.key_input.is_pressed("b"):
                self.X_00.deactivate()
                self.X_21.activate()

        if self.X_11.active and not self.key_input.is_pressed("f"):
            self.X_11.deactivate()
            self.X_01.activate()

        if self.X_21.active and not self.key_input.is_pressed("b"):
            self.X_21.deactivate()
            self.X_01.activate()

        if self.X_01.active and not self.stepper.rotator.busy:
            self.X_01.deactivate()
            self.main.X0.activate()
    
    def execute_actions(self):
        if self.X_00.rising_edge:
            self.logger.info(
                "Press and hold f-key for forward motion, "
                "or b-key for backward motion."
            )

        if self.X_11.rising_edge != self.X_21.rising_edge:
            if self.X_11.rising_edge:
                self.stepper.rotator.direction = RotationDirection.CCW
            if self.X_21.rising_edge:
                self.stepper.rotator.direction = RotationDirection.CW
            self.stepper.rotator.start()

        if self.X_01.rising_edge:
            self.logger.info("Motion stopped.")
            self.stepper.rotator.stop()


class MultiAxisPLC(AbstractPLC):

    def __init__(self, logger):
        super().__init__(logger=logger)
        self.key_input = KeyInput()

        self.stepper_x = create_stepper_x(logger)
        self.stepper_y = create_stepper_y(logger)
        self.stepper_z = create_stepper_z(logger)

        self.init_flag = True

        self.X0 = self.add_marker("X0")

        self.X_00 = self.add_marker("X_00")
        self.X_11 = self.add_marker("X_11")
        self.X_21 = self.add_marker("X_21")
        self.X_12 = self.add_marker("X_12")
        self.X_22 = self.add_marker("X_22")
        self.X_01 = self.add_marker("X_01")

        self.subroutine = None

    def _init(self):
        if self.init_flag:
            self.init_flag = False

            configure_stepper(self.stepper_x)
            configure_stepper(self.stepper_y)
            configure_stepper(self.stepper_z)

            self.X0.activate()

    def _sequence_control(self):
        self.key_input.update()

        if self.X0.active:
            if self.key_input.rising_edge("x"):
                self.X0.deactivate()
                self.subroutine = JogSubroutine(self, "x")
                self.subroutine.start()
            if self.key_input.rising_edge("y"):
                self.X0.deactivate()
                self.subroutine = JogSubroutine(self, "y")
                self.subroutine.start()
            if self.key_input.rising_edge("z"):
                self.X0.deactivate()
                self.subroutine = JogSubroutine(self, "z")
                self.subroutine.start()

        if self.subroutine is not None:
            self.subroutine.sequence_control()

    def _execute_actions(self):
        if self.X0.rising_edge:
            self.subroutine = None
            self.logger.info(
                "Press x-key for x-axis motion, "
                "or y-key for y-axis motion, "
                "or z-key for z-axis motion."
            )

        if self.subroutine is not None:
            self.subroutine.execute_actions()

    def control_routine(self) -> None:
        self._init()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        self.stepper_x.disable()
        self.stepper_y.disable()
        self.stepper_z.disable()

    def emergency_routine(self) -> None:
        self.exit_routine()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
        raise exception


if __name__ == '__main__':

    import os

    from pyberryplc.utils.log_utils import init_logger

    os.system("clear")
    logger = init_logger(name="Demo 5")
    logger.info("Starting demo 5...")
    plc = MultiAxisPLC(logger)
    plc.run()
