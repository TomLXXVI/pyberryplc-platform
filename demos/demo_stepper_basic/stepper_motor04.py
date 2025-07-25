"""
Demo on how to use the `TMC2208StepperMotor` class. Performing a rotation with
a motion profile (`RotatorType.MOTION_PROFILE`) and reversing the rotation 
direction. The rotation is blocking: the PLC scan cycle is blocked as long as
the rotation has not finished.

Note: When using the keyboard for input to the PLC program, it is required to
launch this script as a "sudo user". For this purpose a shell script 
`run_with_keyboard.sh` is provided in the demo folder. Launch each of the Python
scripts in this demo folder by running `source run_with_keyboard.sh` at the
console prompt.
"""
from pyberryplc.core import AbstractPLC
from pyberryplc.stepper import (
    PinConfig,
    TMC2208StepperMotor, 
    TMC2208UART,
    RotatorType,
)
from pyberryplc.motion.profile import TrapezoidalProfile
from pyberryplc.motion import RotationDirection
from pyberryplc.utils.keyboard_input import KeyInput


def stepper_motor_setup(logger):
    stepper = TMC2208StepperMotor(
        pin_config=PinConfig(
            step_pin=21,
            dir_pin=27
        ),
        logger=logger,
        name="x-motor",
        uart=TMC2208UART(port="/dev/ttyUSB0")
    )
    stepper.attach_rotator(RotatorType.MOTION_PROFILE)
    stepper.rotator.profile = TrapezoidalProfile(
        ds_tot=720,
        a_max=360,
        v_max=180,
        v_i=0.0,
        v_f=0.0
    )
    return stepper


class StepperUARTPLC(AbstractPLC):
    
    def __init__(self, logger):
        super().__init__(logger=logger)
        
        # Use keyboard connected with the Raspberry Pi to send inputs to
        # the PLC program.
        self.key_input = KeyInput()

        # Stepper motor setup.
        self.stepper = stepper_motor_setup(self.logger)

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")

        self.input_flag = True
    
    def _init_control(self):
        # Initialization routine runs only once in the first scan cycle of the
        # PLC program. `self.input_flag` prevents re-initialization in 
        # subsequent scan cycles.
        if self.input_flag:
            self.input_flag = False
            
            # Stepper driver configuration via UART.
            self.stepper.enable(high_sensitivity=True)  # Turn the driver on.
            self.stepper.configure_microstepping(
                resolution="1/2",
                ms_pins=None,
                full_steps_per_rev=200
            )
            self.stepper.set_current_via_uart(
                run_current_pct=35.0,
                hold_current_pct=10.0
            )
            
            self.X0.activate()
    
    def _sequence_control(self):
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
            # Set rotation direction and give the motor the command to rotate
            # counterclockwise.
            self.stepper.rotator.direction = RotationDirection.CCW
            self.stepper.rotator.rotate()
            self.logger.info("Press 'r' to start motor in reverse")
            
        if self.X2.rising_edge:
            self.stepper.rotator.direction = RotationDirection.CW
            self.stepper.rotator.rotate()
            self.logger.info("Press 'q' to go back to idle")
            
    def control_routine(self):
        # Executes the PLC program.
        self.key_input.update()  # Reads any keyboard input.
        
        self._init_control()
        self._sequence_control()
        self._execute_actions()
    
    def exit_routine(self):
        # The exit routine is called by pressing <Ctrl-Z> (on the keyboard of 
        # the computer that launched the PLC program; this is not necessarily
        # the Raspberry Pi if the PLC program was launched via SSH-console). 
        self.logger.info("Exiting... disabling driver.")
        self.stepper.disable()  # Turn the driver back off.

    def emergency_routine(self):
        # Called when an `EmergencyException` is raised from within 
        # `control_routine()`. So, raising an `EmergencyException` must be 
        # programmed in the PLC program.
        pass
    
    def crash_routine(self, exception: Exception) -> None:
        # Called when any unexpected error in the PLC program should happen. 
        # It allows for example to disable the stepper motor driver under any
        # faulty circumstance.
        self.logger.error("PLC crashed! - Disabling driver.")
        self.stepper.disable()  # Turn the driver back off.


if __name__ == "__main__":
    import os
    from pyberryplc.utils.log_utils import init_logger

    os.system("clear")
    logger = init_logger(name="STEPPER PLC")
    logger.info("Run `stepper_motor04.py`")
    plc = StepperUARTPLC(logger)
    plc.run()
