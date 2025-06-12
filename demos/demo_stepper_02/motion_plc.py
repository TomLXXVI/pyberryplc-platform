"""
Demo of how to use the `XYZMotionPLC` class to create a motion control PLC
application. Loads a 2D-trajectory from file and runs the segment movements one
by one on command.
"""
import os
import logging

from pyberryplc.utils.log_utils import init_logger
from pyberryplc.utils.keyboard_input import KeyInput
from pyberryplc.stepper import XYZMotionPLC


class MotionPLC(XYZMotionPLC):
    
    def __init__(self, logger: logging.Logger):
        super().__init__(
            shared_data=None,
            logger=logger,
            motor_config_filepath="motor_config.toml"
        )
        self.key_input = KeyInput()

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        
        self.trajectory_is_loaded = self.add_marker("trajectory_is_loaded")
        self.trajectory_is_finished = self.add_marker("trajectory_is_finished")
        
    def init_control(self) -> None:
        self.logger.info("Press 's' to start executing the trajectory.")
        self.X0.activate()
    
    def sequence_control(self) -> None:
        self.key_input.update()
        
        if self.X0.active and self.key_input.rising_edge("s"):
            self.logger.info("Loading trajectory...")
            self.X0.deactivate()
            self.X1.activate()
        
        if self.X1.active and self.trajectory_is_loaded.active:
            self.logger.info("Running trajectory...")
            self.X1.deactivate()
            self.X2.activate()
        
        if self.X2.active and self.trajectory_is_finished.active:
            self.logger.info("Trajectory completed.")
            self.logger.info("Press 's' to load and run trajectory again.")
            self.X2.deactivate()
            self.X0.activate()
    
    def execute_actions(self) -> None:
        if self.X0.rising_edge:
            self.trajectory_is_loaded.deactivate()
            self.trajectory_is_finished.deactivate()
        
        if self.X1.rising_edge:
            self.motion_controller.load_trajectory("demo_trajectory.json")
            self.trajectory_is_loaded.activate()
        
        if self.X2.active and self.motion_control_status.ready:
            i = self.motion_controller.move()
            if i != -1:
                self.logger.info(f"Now executing segment {i}...")
            else:
                self.trajectory_is_finished.activate()


if __name__ == "__main__":
    os.system("clear")
    logger = init_logger(name="MOTION PLC")
    logger.info("Run `demo_stepper_02`")
    plc = MotionPLC(logger)
    plc.run()
