from pyberryplc.core import AbstractPLC, CounterDown, MemoryVariable, EmergencyException
from pyberryplc.stepper.controller import XYZMotionController, MotionStatus
from pyberryplc.utils.keyboard_input import KeyInput


class TrajectoryPLC(AbstractPLC):

    def __init__(self, logger, traj_file_path: str):
        super().__init__(logger=logger)
        self.traj_file_path = traj_file_path

        self.key_input = KeyInput()

        self.motion_controller = XYZMotionController(self, "motor_config.toml", logger)
        self.motion: MotionStatus | None = None

        self.init_flag = True

        self.X0 = self.add_marker("X0")
        self.X1 = self.add_marker("X1")
        self.X2 = self.add_marker("X2")
        self.X3 = self.add_marker("X3")

        self.C1 = CounterDown(0)
        self.traj_loaded = MemoryVariable()
        self.traj_finished = MemoryVariable()
        self.num_segments = 0

    def _init_control(self):
        if self.init_flag:
            self.init_flag = False
            self.motion_controller.enable()
            self.logger.info("Press s-key to start.")
            self.X0.activate()

    def _sequence_control(self):
        self.key_input.update()
        self.motion = self.motion_controller.get_motion_status()

        if self.key_input.rising_edge("e"):
            raise EmergencyException

        if self.X0.active and self.key_input.rising_edge("s"):
            self.logger.info("Loading trajectory from file...")
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and self.traj_loaded.active:
            self.logger.info("Executing trajectory...")
            self.X1.deactivate()
            self.X2.activate()

        if self.X2.active and self.motion.all_ready and self.C1.value < self.num_segments:
            i = self.num_segments - self.C1.value
            self.logger.info(f"Segment {i} completed.")
            self.X2.deactivate()
            self.X3.activate()

        if self.X3.active:
            if self.X3.rising_edge:
                if not self.traj_finished.active:
                    self.logger.info("Press s-key for next segment.")
                else:
                    self.logger.info("Trajectory completed.")
            if not self.traj_finished.active and self.key_input.rising_edge("s"):
                self.X3.deactivate()
                self.X2.activate()
            elif self.traj_finished.active:
                self.X3.deactivate()
                self.exit()

    def _execute_actions(self):

        if self.X1.rising_edge:
            self.num_segments = self.motion_controller.load_trajectory(self.traj_file_path)
            self.C1.set(self.num_segments)
            self.traj_loaded.activate()

        if self.X2.rising_edge:
            if self.C1.value > 0:
                self.motion_controller.move()
                self.C1.count_down()
            if self.C1.value == 0:
                self.traj_finished.activate()

    def control_routine(self) -> None:
        self._init_control()
        self._sequence_control()
        self._execute_actions()

    def exit_routine(self) -> None:
        self.motion_controller.disable()

    def emergency_routine(self) -> None:
        self.exit_routine()

    def crash_routine(self, exception: Exception | KeyboardInterrupt) -> None:
        self.exit_routine()
        raise exception


if __name__ == '__main__':

    import os
    from pyberryplc.utils.log_utils import init_logger

    os.system("clear")
    logger = init_logger(name="Demo 6")
    logger.info("Starting demo 6...")
    plc = TrajectoryPLC(logger, traj_file_path="trajectory.json")
    plc.run()
