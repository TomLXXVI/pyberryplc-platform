import os
from pyberryplc.core import AbstractPLC
from pyberryplc.stepper import TMC2208StepperMotor, TMC2208UART
from pyberryplc.motion import TrapezoidalProfile
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput


class StepperUARTTestPLC(AbstractPLC):

    def __init__(self):
        super().__init__()

        self.key_input = KeyInput()

        self.stepper = TMC2208StepperMotor(
            step_pin=27,
            dir_pin=26,
            full_steps_per_rev=200,
            microstep_resolution="1/16",
            uart=TMC2208UART(port="/dev/ttyAMA0"),
            high_sensitivity=True,
            logger=self.logger
        )

        self.profile = TrapezoidalProfile(
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

            # Stepper driver configuration 
            self.stepper.enable()
            self.stepper.set_microstepping()
            self.stepper.set_current_via_uart(
                run_current_pct=19,
                hold_current_pct=10
            )

            self.X0.activate()

    def _sequence_control(self):
        if not self.stepper.busy:
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

        if self.X1.active:
            if self.X1.rising_edge:
                self.stepper.start_rotation_profile(
                    direction="forward",
                    profile=self.profile
                )
                self.logger.info("Press 'r' to start motor in reverse")

        if self.X2.active:
            if self.X2.rising_edge:
                self.stepper.start_rotation_profile(
                    direction="backward",
                    profile=self.profile
                )
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
        self.exit_routine()


if __name__ == "__main__":
    os.system("clear")
    init_logger()
    plc = StepperUARTTestPLC()
    plc.run()
