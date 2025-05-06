"""
Test script for dynamic motion profile using DynamicDelayGenerator and SFC-style
PLC logic.
Control via keyboard input:
- Press 's' to start motion.
- Press 'r' to stop (trigger deceleration).
"""

import os
from pyberryplc.core.plc import AbstractPLC
from pyberryplc.motion_profiles.motion_profile import TrapezoidalProfile
from pyberryplc.stepper import TMC2208StepperMotor, TMC2208UART
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput


class StepperDynamicPLC(AbstractPLC):
    def __init__(self):
        super().__init__()

        self.key_input = KeyInput()

        self.stepper = TMC2208StepperMotor(
            step_pin=27,
            dir_pin=26,
            full_steps_per_rev=200,
            microstep_resolution="1/8",
            uart=TMC2208UART(port="/dev/ttyAMA0"),
            high_sensitivity=True,
            logger=self.logger
        )

        self.profile = TrapezoidalProfile(v_m=180.0, a_m=720.0, ds_tot=90.0)
        
        self.X0 = self.add_marker("X0")  # Idle
        self.X1 = self.add_marker("X1")  # Motion active
        self.X2 = self.add_marker("X2")
        
        self._init_done = False

    def _init_control(self):
        if not self._init_done:
            self._init_done = True
            self.stepper.enable()
            self.stepper.set_microstepping()
            self.stepper.set_current_via_uart(
                run_current_pct=19, 
                hold_current_pct=10
            )
            self.X0.activate()

    def _sequence_control(self):
        if self.X0.active and not self.stepper.busy and self.key_input.rising_edge("s"):
            self.logger.info("Key 's' pressed: starting motion.")
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and self.key_input.rising_edge("r"):
            self.logger.info("Key 'r' pressed: stopping motion.")
            self.X1.deactivate()
            self.X2.activate()
        
        if self.X2.active and not self.stepper.busy:
            self.X2.deactivate()
            self.X0.activate()

    def _execute_actions(self):
        if self.X1.rising_edge:
            self.logger.info("Stepper starts moving.")
            self.stepper.start_rotation_dynamic(
                self.profile, 
                direction="forward"
            )
        
        if self.X2.rising_edge:
            self.logger.info("Stepper is stopped.")
            self.stepper.stop_rotation_dynamic()
        
    def control_routine(self):
        self.key_input.update()
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self):
        self.logger.info("Disabling stepper driver.")
        self.stepper.disable()

    def emergency_routine(self):
        self.logger.warning("Emergency routine triggered.")
        self.stepper.disable()

    def crash_routine(self, exception: Exception) -> None:
        self.exit_routine()


if __name__ == "__main__":
    os.system("clear")
    init_logger(logging_level="debug")
    plc = StepperDynamicPLC()
    plc.run()
