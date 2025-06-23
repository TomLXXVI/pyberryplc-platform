import logging

from pyberryplc.core import AbstractPLC, TimerSingleScan
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput
from pyberryplc.utils.tmc2208_utils import RMSCurrentLogger
from pyberryplc.motion import TrapezoidalProfile, RotationDirection
from pyberryplc.stepper import TMC2208StepperMotor, PinConfig, TMC2208UART, RotatorType


def get_stepper_motor(logger: logging.Logger) -> TMC2208StepperMotor:
    """
    Creates a `TMC2208StepperMotor` instance. The `TMC2208StepperMotor` instance
    is equipped with a `DynamicRotatorThreaded` rotator
    (`RotatorType.DYNAMIC_THREADED`). This allows to start the stepper motor
    by pressing a start button, and the motor will keep on turning until a stop
    button is pressed.
    """
    stepper = TMC2208StepperMotor(
        pin_config=PinConfig(
            step_pin_ID=21,
            dir_pin_ID=27
        ),
        logger=logger,
        name="Y-axis",
        uart=TMC2208UART(port="/dev/ttyUSB1")
    )
    stepper.attach_rotator(RotatorType.DYNAMIC_THREADED)
    stepper.rotator.direction = RotationDirection.COUNTERCLOCKWISE
    stepper.rotator.profile = TrapezoidalProfile(
        ds_tot=720.0,  # deg
        a_m=1000.0,    # deg/s2
        v_m=360.0,     # deg/s
        v_i=0.0,
        v_f=0.0
    )
    return stepper


def configure_stepper_motor(stepper: TMC2208StepperMotor) -> None:
    """Enables the stepper motor driver. Configures full-step mode. Sets the
    motor run and hold current.
    """
    stepper.enable()
    stepper.configure_microstepping(resolution="full", full_steps_per_rev=200)
    stepper.set_current_via_uart(run_current_pct=71.0, hold_current_pct=10.0)


class TestApp1(AbstractPLC):

    def __init__(self):
        super().__init__(logger=init_logger("PLC"))
        self.key_input = KeyInput()
        self.input_flag = True
                
        self.stepper = get_stepper_motor(self.logger)
        self.current_logger = RMSCurrentLogger(self.stepper.uart)
        
        self.X0 = self.add_marker("X0")
        self.X10 = self.add_marker("X10")
        self.X11 = self.add_marker("X11")
        self.X2 = self.add_marker("X2")

        self.T0 = TimerSingleScan(1.0)
        self.T1 = TimerSingleScan(0.2)

    def _init_control(self) -> None:
        if self.input_flag:
            self.input_flag = False
            self.current_logger.open()
            configure_stepper_motor(self.stepper)
            self.X0.activate()

    def _sequence_control(self) -> None:
        if self.T1.has_elapsed:
            self.current_logger.log()
        
        if self.X0.active and self.key_input.rising_edge("f"):
            self.logger.info("Start motor forward")
            self.X0.deactivate()
            self.X10.activate()  # start motor forward (counterclockwise)

        if self.X0.active and self.key_input.rising_edge("b"):
            self.logger.info("Start motor backward")
            self.X0.deactivate()
            self.X11.activate()  # start motor backward (clockwise)

        if self.X10.active and self.key_input.rising_edge("e"):
            self.logger.info("Stop motor")
            self.X10.deactivate()
            self.X2.activate()  # stop motor

        if self.X11.active and self.key_input.rising_edge("e"):
            self.logger.info("Stop motor")
            self.X11.deactivate()
            self.X2.activate()  # stop motor

        if self.X2.active and self.T0.has_elapsed:
            self.X2.deactivate()
            self.X0.activate()

    def _execute_actions(self) -> None:
        if self.X10.rising_edge:
            self.stepper.rotator.direction = RotationDirection.COUNTERCLOCKWISE
            self.stepper.rotator.start()

        if self.X11.rising_edge:
            self.stepper.rotator.direction = RotationDirection.CLOCKWISE
            self.stepper.rotator.start()

        if self.X2.rising_edge:
            self.stepper.rotator.stop()

    def control_routine(self) -> None:
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        self.logger.info("Exiting... disabling driver.")
        self.stepper.disable()  # Turn the driver back off.
        self.current_logger.close()
    
    def emergency_routine(self) -> None:
        self.logger.info("Emergency stop... disabling driver.")
        self.stepper.disable()
        self.current_logger.close()
        
    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.logger.info("PLC crash... disabling driver.")
        self.stepper.disable()
        self.current_logger.close()
        raise exception


if __name__ == '__main__':
    import os
    os.system("clear")
    app = TestApp1()
    app.run()
