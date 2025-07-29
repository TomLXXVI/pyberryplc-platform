from pyberryplc.core import (
    TAbstractPLC,
    MemoryVariable,
    EmergencyException,
    AbstractPLC
)
from pyberryplc.motion import RotationDirection, TrapezoidalProfile
from pyberryplc.stepper import XYZMotionController, MotionStatus
from pyberryplc.utils.keyboard_input import KeyInput


class AutomaticMode:

    def __init__(
        self,
        main: TAbstractPLC,
        trajectory_file: str
    ) -> None:
        self.main = main
        self.logger = main.logger
        self.key_input = main.key_input
        self.motion_controller = main.motion_controller
        self.motion = main.motion

        self.trajectory_file = trajectory_file

        self.X0 = main.add_marker("Auto.X0")
        self.X1 = main.add_marker("Auto.X1")
        self.X2 = main.add_marker("Auto.X2")

        self.traj_loaded = MemoryVariable()
        self.traj_finished = MemoryVariable()

        self.init_flag = True
        self.exit = MemoryVariable(False)

    def _init_control(self):
        if self.init_flag:
            self.init_flag = False
            self.X0.activate()

    def _sequence_control(self):
        if self.key_input.rising_edge("e"):
            raise EmergencyException

        if self.X0.active and self.key_input.rising_edge("s"):
            self.logger.info("Loading trajectory...")
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active and self.traj_loaded.active:
            self.logger.info("Executing trajectory...")
            self.X1.deactivate()
            self.X2.activate()

        if self.X2.active and self.traj_finished.active:
            self.logger.info("Trajectory finished.")
            self.X2.deactivate()
            self.X0.activate()
            self.exit.activate()

    def _execute_actions(self):
        if self.X0.active:
            self.traj_loaded.deactivate()
            self.traj_finished.deactivate()

        if self.X1.rising_edge:
            self.motion_controller.load_trajectory(self.trajectory_file)
            self.traj_loaded.activate()

        if self.X2.active:
            if self.motion.all_ready:
                self.motion_controller.move()
            if self.motion.all_finished:
                self.traj_finished.activate()

    def call(self):
        self.motion = self.main.motion
        self._init_control()
        self._sequence_control()
        self._execute_actions()


class JogMode:

    def __init__(
        self,
        main: TAbstractPLC
    ) -> None:
        self.main = main
        self.logger = main.logger
        self.key_input = main.key_input
        self.motion_controller = main.motion_controller
        self.motion = main.motion

        self.init_flag = True

        self.X0 = main.add_marker("Jog.X0")
        self.X11 = main.add_marker("Jog.X11")
        self.X12 = main.add_marker("Jog.X12")
        self.X21 = main.add_marker("Jog.X21")
        self.X22 = main.add_marker("Jog.X22")
        self.X31 = main.add_marker("Jog.X31")
        self.X32 = main.add_marker("Jog.X32")

        self.exit = MemoryVariable(False)

    def _init_control(self):
        if self.init_flag:
            self.init_flag = False
            self.X0.activate()

    def _sequence_control(self):
        if self.X0.active:
            if self.key_input.rising_edge("x"):
                self.X0.deactivate()
                self.X11.activate()
            if self.key_input.rising_edge("y"):
                self.X0.deactivate()
                self.X21.activate()
            if self.key_input.rising_edge("z"):
                self.X0.deactivate()
                self.X31.activate()

        if self.X11.active and self.motion.jog_mode_active:
            self.X11.deactivate()
            self.X12.activate()

        if self.X21.active and self.motion.jog_mode_active:
            self.X21.deactivate()
            self.X22.activate()

        if self.X31.active and self.motion.jog_mode_active:
            self.X31.deactivate()
            self.X32.activate()

        if self.X12.active and not self.motion.jog_mode_active:
            self.X12.deactivate()
            self.X0.activate()
            self.exit.activate()

        if self.X22.active and not self.motion.jog_mode_active:
            self.X22.deactivate()
            self.X0.activate()
            self.exit.activate()

        if self.X32.active and not self.motion.jog_mode_active:
            self.X32.deactivate()
            self.X0.activate()
            self.exit.activate()

    def _execute_actions(self):
        if self.X11.active:
            if self.X11.rising_edge:
                self.logger.info(
                    "Jog x-axis. Press and hold f-key for forward motion, or "
                    "b-key for backward motion."
                )
            self._run_jog("x")

        if self.X21.active:
            if self.X21.rising_edge:
                self.logger.info(
                    "Jog y-axis. Press and hold f-key for forward motion, or "
                    "b-key for backward motion."
                )
            self._run_jog("y")

        if self.X31.active:
            if self.X31.rising_edge:
                self.logger.info(
                    "Jog z-axis. Press and hold f-key for forward motion, or "
                    "b-key for backward motion."
                )
            self._run_jog("z")

        if self.X12.active:
            self._stop_jog("x")

        if self.X22.active:
            self._stop_jog("y")

        if self.X32.active:
            self._stop_jog("z")

    def _run_jog(self, axis: str):
        if self.key_input.rising_edge("f"):
            self.motion_controller.start_jog_mode(axis, RotationDirection.CCW)
        if self.key_input.rising_edge("b"):
            self.motion_controller.start_jog_mode(axis, RotationDirection.CW)

    def _stop_jog(self, axis: str):
        if not self.key_input.is_pressed("f") and not self.key_input.is_pressed("b"):
            self.motion_controller.stop_jog_mode(axis)

    def call(self):
        self.motion = self.main.motion
        self._init_control()
        self._sequence_control()
        self._execute_actions()


class MotionPLC(AbstractPLC):

    def __init__(self, logger, traj_filepath: str):
        super().__init__(logger=logger)
        self.key_input = KeyInput()

        self.motion_controller = XYZMotionController(
            master=self,
            config_filepath="motor_config.toml",
            logger=self.logger,
            jog_mode_profile=TrapezoidalProfile(
                ds_tot=720.0,
                a_max=360.0,
                v_max=180.0,
                v_i=0.0,
                v_f=0.0
            )
        )
        self.motion: MotionStatus | None = None

        self.auto = AutomaticMode(self, traj_filepath)
        self.jog = JogMode(self)

        self.init_flag = True

        self.X0 = self.add_marker("Main.X0")
        self.X1 = self.add_marker("Main.X1")
        self.X2 = self.add_marker("Main.X2")

    def _init_control(self):
        if self.init_flag:
            self.init_flag = False
            self.motion_controller.enable()
            self.X0.activate()

    def _sequence_control(self):
        if self.X0.active:
            if self.key_input.rising_edge("a"):
                self.X0.deactivate()
                self.X1.activate()
            if self.key_input.rising_edge("j"):
                self.X0.deactivate()
                self.X2.activate()

        if self.X1.active and self.auto.exit.active:
            self.X1.deactivate()
            self.auto.exit.deactivate()
            self.X0.activate()

        if self.X2.active and self.jog.exit.active:
            self.X2.deactivate()
            self.jog.exit.deactivate()
            self.X0.activate()

    def _execute_actions(self):
        if self.X0.rising_edge:
            self.logger.info(
                "Press a-key for automatic mode, "
                "or j-key for jog mode."
            )

        if self.X1.active:
            if self.X1.rising_edge:
                self.logger.info("Auto mode. Press s-key to start.")
            self.auto.call()

        if self.X2.active:
            if self.X2.rising_edge:
                self.logger.info(
                    "Jog mode. Select axis: "
                    "x-key for x-axis, "
                    "y-key for y-axis, or "
                    "z-key for z-axis."
                )
            self.jog.call()

    def control_routine(self) -> None:
        self.key_input.update()
        self.motion = self.motion_controller.get_motion_status()

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
    logger = init_logger(name="Demo 7")
    logger.info("Starting demo 7...")
    plc = MotionPLC(logger, traj_filepath="trajectory.json")
    plc.run()
