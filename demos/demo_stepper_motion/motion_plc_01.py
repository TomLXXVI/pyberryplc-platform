"""
Demo on how to use the `XYZMotionPLC` class to create a motion control PLC
application. Loads a 2D-trajectory from file and runs the segment movements one
by one when commanded to move (by pressing the 's' key on the keyboard).
"""
import os
import logging

from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput
from pyberryplc.stepper import XYZMotionPLC
from pyberryplc.core.timers import TimerOnDelay
from pyberryplc.core.counters import CounterDown


class MotionPLC(XYZMotionPLC):
        
    def __init__(self, logger: logging.Logger):
        super().__init__(
            hmi_data=None,
            logger=logger,
            motor_config_filepath="motor_config.toml"
        )
        self.key_input = KeyInput()
        
        self.L1, _ = self.add_digital_output(23, "L1")
        self.T1 = TimerOnDelay(3)
        self.C1 = CounterDown(0)
        
        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        self.X3 = self.add_marker("X3")
        self.X4 = self.add_marker("X4")
        
        self.trajectory_loaded = self.add_marker("trajectory_loaded")
        self.trajectory_finished = self.add_marker("trajectory_finished")
        self.num_segments = 0
        
    def init_control(self) -> None:
        self.logger.info("Press 's' to start.")
        self.X0.activate()
    
    def sequence_control(self) -> None:
        self.key_input.update()
        
        if self.X0.active and self.key_input.rising_edge("s"):
            self.logger.info("Loading trajectory...")
            self.X0.deactivate()
            self.X1.activate()
        
        if self.X1.active and self.trajectory_loaded.active:
            self.logger.info("Running trajectory...")
            self.X1.deactivate()
            self.X2.activate()
        
        if self.X2.active and self.motion_control_status.ready and self.C1.value < self.num_segments:
            self.logger.info(f"Movement {self.num_segments - self.C1.value} is completed.")
            self.logger.info("Turn L1 on for 3 s.")
            self.X2.deactivate()
            self.X3.activate()
        
        if self.X3.active and self.T1.has_elapsed:
            self.X3.deactivate()
            self.X4.activate()
        
        if self.X4.active:
            if self.X4.rising_edge: 
                if not self.trajectory_finished.active:
                    self.logger.info("Press 's' again to continue.")
                else:
                    self.logger.info("Trajectory completed. Going back to X0.")
                    self.logger.info("Press 's' to repeat.")
            
            if not self.L1.active:
                if not self.trajectory_finished.active and self.key_input.rising_edge("s"):
                    self.X4.deactivate()
                    self.X2.activate()
                elif self.trajectory_finished.active:
                    self.X4.deactivate()
                    self.X0.activate()
    
    def execute_actions(self) -> None:
        if self.X0.rising_edge:
            self.trajectory_loaded.deactivate()
            self.trajectory_finished.deactivate()
            self.C1.reset()
            
        if self.X1.rising_edge:
            self.num_segments = self.motion_controller.load_trajectory("demo_trajectory.json")
            self.C1.set(self.num_segments)
            self.trajectory_loaded.activate()
        
        if self.X2.rising_edge:
            if self.C1.value > 0:
                self.motion_controller.move()
                self.C1.count_down()
            if self.C1.value == 0:
                self.trajectory_finished.activate()
        
        if self.X3.rising_edge:
            self.L1.activate()
        
        if self.X4.rising_edge:
            self.L1.deactivate()
            self.T1.reset()


if __name__ == "__main__":
    os.system("clear")
    logger = init_logger(name="MOTION PLC")
    logger.info("Run `demo_stepper_02`")
    plc = MotionPLC(logger)
    plc.run()
