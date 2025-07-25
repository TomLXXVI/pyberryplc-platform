"""
Manual Mode Subroutine
----------------------
Program to move the machine axes manually via the keyboard of the Raspberry Pi.
- Key 'a' starts to rotate the X-axis in forward direction.
- Key 'b' starts to rotate the X-axis in backward direction.
- Key 'c' starts to rotate the Y-axis in forward direction.
- Key 'd' starts to rotate the Y-axis in backward direction.
- Key 'e' starts to rotate the Z-axis in forward direction.
- Key 'f' starts to rotate the Z-axis in backward direction.
- Key 's' stops any movement of the axes.
To exit the program, <Ctrl-Z> needs to pressed (from the computer where the 
console that launched the program, is running). 
"""
from pyberryplc.core import TAbstractPLC
from pyberryplc.core.timers import TimerSingleScan
from pyberryplc.utils.keyboard_input import KeyInput
from pyberryplc.motion import RotationDirection

from .core import (
    create_dynamic_stepper_motor,
    configure_stepper_motor,
    load_motor_configurations
)


class ManualModeSubRoutine:
    
    def __init__(
        self, 
        main_routine: TAbstractPLC, 
        cfg_filepath: str 
    ) -> None:
        self.main = main_routine
        self.key_input = KeyInput()
        self.logger = self.main.logger
        
        cfgs = load_motor_configurations(cfg_filepath)
        self.xmotor_cfg = cfgs[0]
        self.ymotor_cfg = cfgs[1]
        self.zmotor_cfg = cfgs[2]
        self.xmotor = create_dynamic_stepper_motor(
            name="X-axis",
            cfg=self.xmotor_cfg,
            logger=self.main.logger
        )
        self.ymotor = create_dynamic_stepper_motor(
            name="Y-axis",
            cfg=self.ymotor_cfg,
            logger=self.main.logger
        )
        if self.zmotor_cfg is not None:
            self.zmotor = create_dynamic_stepper_motor(
                name="Z-axis",
                cfg=self.zmotor_cfg,
                logger=self.main.logger
            )
        else:
            self.zmotor = None
    
        self.init_flag = True
        self.T0 = TimerSingleScan(0.5)
        self.X0 = self.main.add_marker("Manual.X0")
        self.X11 = self.main.add_marker("Manual.X11")
        self.X12 = self.main.add_marker("Manual.X12")
        self.X21 = self.main.add_marker("Manual.X21")
        self.X22 = self.main.add_marker("Manual.X22")
        self.X31 = self.main.add_marker("Manual.X31")
        self.X32 = self.main.add_marker("Manual.X32")
        self.X4 = self.main.add_marker("Manual.X4")
    
    def _init_control(self) -> None:
        if self.init_flag:
            configure_stepper_motor(self.xmotor, self.xmotor_cfg)
            configure_stepper_motor(self.ymotor, self.ymotor_cfg)
            if self.zmotor is not None:
                configure_stepper_motor(self.zmotor, self.zmotor_cfg)
            self.init_flag = False
            self.X0.activate()
    
    def _sequence_control(self) -> None:
        if self.X0.active and self.key_input.rising_edge("a"):
            self.X0.deactivate()
            self.X11.activate()
        
        if self.X0.active and self.key_input.rising_edge("b"):
            self.X0.deactivate()
            self.X12.activate()
        
        if self.X0.active and self.key_input.rising_edge("c"):
            self.X0.deactivate()
            self.X21.activate()
        
        if self.X0.active and self.key_input.rising_edge("d"):
            self.X0.deactivate()
            self.X22.activate()
        
        if self.zmotor:
            if self.X0.active and self.key_input.rising_edge("e"):
                self.X0.deactivate()
                self.X31.activate()
            
            if self.X0.active and self.key_input.rising_edge("f"):
                self.X0.deactivate()
                self.X32.activate()
        
        if self.X11.active and self.key_input.rising_edge("s"):
            self.X11.deactivate()
            self.X4.activate()
        
        if self.X12.active and self.key_input.rising_edge("s"):
            self.X12.deactivate()
            self.X4.activate()
        
        if self.X21.active and self.key_input.rising_edge("s"):
            self.X21.deactivate()
            self.X4.activate()
    
        if self.X22.active and self.key_input.rising_edge("s"):
            self.X22.deactivate()
            self.X4.activate()
        
        if self.X31.active and self.key_input.rising_edge("s"):
            self.X31.deactivate()
            self.X4.activate()
    
        if self.X32.active and self.key_input.rising_edge("s"):
            self.X32.deactivate()
            self.X4.activate()
        
        if self.X4.active and self.T0.has_elapsed:
            self.X4.deactivate()
            self.X0.activate()
    
    def _execute_actions(self) -> None:
        if self.X11.rising_edge:
            self.xmotor.rotator.direction = RotationDirection.CCW
            self.xmotor.rotator.start()
        
        if self.X12.rising_edge:
            self.xmotor.rotator.direction = RotationDirection.CW
            self.xmotor.rotator.start()
        
        if self.X21.rising_edge:
            self.ymotor.rotator.direction = RotationDirection.CCW
            self.ymotor.rotator.start()
        
        if self.X22.rising_edge:
            self.ymotor.rotator.direction = RotationDirection.CW
            self.ymotor.rotator.start()

        if self.X31.rising_edge:
            self.zmotor.rotator.direction = RotationDirection.CCW
            self.zmotor.rotator.start()

        if self.X32.rising_edge:
            self.zmotor.rotator.direction = RotationDirection.CW
            self.zmotor.rotator.start()
        
        if self.X4.rising_edge:
            if self.xmotor.rotator.busy:
                self.xmotor.rotator.stop()
            if self.ymotor.rotator.busy:
                self.ymotor.rotator.stop()
            if self.zmotor is not None and self.zmotor.rotator.busy:
                self.zmotor.rotator.stop()
    
    def call(self) -> None:
        self.key_input.update()
        self._init_control()
        self._sequence_control()
        self._execute_actions()
    
    def exit(self) -> None:
        self.xmotor.disable()
        self.logger.info("X-axis motor disabled.")
        self.ymotor.disable()
        self.logger.info("Y-axis motor disabled.")
        if self.zmotor is not None:
            self.zmotor.disable()
            self.logger.info("Z-axis motor disabled.")
