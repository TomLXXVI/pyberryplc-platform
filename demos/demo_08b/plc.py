from pyberryplc.core import (
    MemoryVariable,
    AbstractPLC,
    EmergencyException
)
from pyberryplc.motion import TrapezoidalProfile
from pyberryplc.stepper import XYZMotionController, MotionStatus


class AutomaticMode:

    def __init__(
        self,
        main: 'MotionPLC',
    ) -> None:
        self.main = main
        self.logger = main.logger
        self.motion_controller = main.motion_controller
        self.motion = main.motion

        self.X0 = main.add_marker("Auto.X0")
        self.X1 = main.add_marker("Auto.X1")
        self.X2 = main.add_marker("Auto.X2")

        self.traj_loaded = MemoryVariable()
        self.traj_finished = MemoryVariable()

        self.init_flag = True
        self.exit = MemoryVariable(False)
        self.num_segments = 0

    def _init_control(self):
        if self.init_flag:
            self.init_flag = False
            self.X0.activate()

    def _sequence_control(self):
        if self.X0.active and self.main.auto_start.active:
            self.logger.info("Loading trajectory...")
            self.X0.deactivate()
            self.X1.activate()

        if self.X1.active:
            if self.traj_loaded.active:
                self.logger.info("Executing trajectory...")
                self.X1.deactivate()
                self.X2.activate()
            if self.num_segments == -1:
                self.logger.info("Got an invalid trajectory.")
                self.X1.deactivate()
                self.X0.activate()
                self.exit.activate()

        if self.X2.active and self.traj_finished.active:
            self.logger.info("Trajectory finished.")
            self.X2.deactivate()
            self.X0.activate()
            self.exit.activate()

    def _execute_actions(self):
        if self.X0.active:
            self.traj_loaded.deactivate()
            self.traj_finished.deactivate()
            self.num_segments = 0

        if self.X1.rising_edge:
            self.num_segments = self.motion_controller.load_trajectory(
                None,
                trajectory=self.main.hmi_data.data["trajectory"]
            )
            if self.num_segments > 0:
                self.traj_loaded.activate()
                self.main.hmi_data.data["plc_state"] = "running"

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
        main: 'MotionPLC'
    ) -> None:
        self.main = main
        self.logger = main.logger
        self.motion_controller = main.motion_controller
        self.motion = main.motion
        self.jog_mode_profiles = main.jog_mode_profiles

        self.X0 = main.add_marker("Jog.X0")
        self.X11 = main.add_marker("Jog.X11")
        self.X12 = main.add_marker("Jog.X12")
        self.X21 = main.add_marker("Jog.X21")
        self.X22 = main.add_marker("Jog.X22")
        self.X31 = main.add_marker("Jog.X31")
        self.X32 = main.add_marker("Jog.X32")

        self.init_flag = True
        self.exit = MemoryVariable(False)

        x_rdir_ref = self.motion_controller.x_motor_cfg["rdir_ref"]
        y_rdir_ref = self.motion_controller.y_motor_cfg["rdir_ref"]
        z_rdir_ref = self.motion_controller.z_motor_cfg["rdir_ref"]
        self.x_rdir_ref = self.motion_controller.get_rotation_direction(x_rdir_ref)
        self.y_rdir_ref = self.motion_controller.get_rotation_direction(y_rdir_ref)
        self.z_rdir_ref = self.motion_controller.get_rotation_direction(z_rdir_ref)

    def _init_control(self):
        if self.init_flag:
            self.init_flag = False
            self.X0.activate()

    def _sequence_control(self):
        if self.X0.active:
            if self.main.jog_xf.active or self.main.jog_xb.active:
                self.X0.deactivate()
                self.X11.activate()  # jog X-axis
            if self.main.jog_yf.active or self.main.jog_yb.active:
                self.X0.deactivate()
                self.X21.activate()  # jog Y-axis
            if self.main.jog_zf.active or self.main.jog_zb.active:
                self.X0.deactivate()
                self.X31.activate()  # jog Z-axis

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
            if self.main.jog_xf.rising_edge:
                self.motion_controller.start_jog_mode("x", self.x_rdir_ref)
                self.main.hmi_data.data["plc_state"] = "running"
            if self.main.jog_xb.rising_edge:
                self.motion_controller.start_jog_mode("x", ~self.x_rdir_ref)
                self.main.hmi_data.data["plc_state"] = "running"

        if self.X21.active:
            if self.main.jog_yf.rising_edge:
                self.motion_controller.start_jog_mode("y", self.y_rdir_ref)
                self.main.hmi_data.data["plc_state"] = "running"
            if self.main.jog_yb.rising_edge:
                self.motion_controller.start_jog_mode("y", ~self.y_rdir_ref)
                self.main.hmi_data.data["plc_state"] = "running"

        if self.X31.active:
            if self.main.jog_zf.rising_edge:
                self.motion_controller.start_jog_mode("z", self.z_rdir_ref)
                self.main.hmi_data.data["plc_state"] = "running"
            if self.main.jog_zb.rising_edge:
                self.motion_controller.start_jog_mode("z", ~self.z_rdir_ref)
                self.main.hmi_data.data["plc_state"] = "running"

        if self.X12.active:
            if self.main.jog_xf.falling_edge or self.main.jog_xb.falling_edge:
                self.motion_controller.stop_jog_mode("x")

        if self.X22.active:
            if self.main.jog_yf.falling_edge or self.main.jog_yb.falling_edge:
                self.motion_controller.stop_jog_mode("y")

        if self.X32.active:
            if self.main.jog_zf.falling_edge or self.main.jog_zb.falling_edge:
                self.motion_controller.stop_jog_mode("z")

    def call(self):
        self.motion = self.main.motion

        # Setting jog speed is not tied to any step of the jog mode sequence.
        self.motion_controller.set_jog_mode_profile(
            self.jog_mode_profiles[
                self.main.hmi_data.data.get("jog_speed", "medium")
            ]
        )

        self._init_control()
        self._sequence_control()
        self._execute_actions()


class MotionPLC(AbstractPLC):

    def __init__(self, hmi_data, logger):
        super().__init__(hmi_data=hmi_data, logger=logger)

        self.jog_mode_profiles = {
            "slow": TrapezoidalProfile(
                ds_tot=720.0,
                a_max=1080.0,
                v_max=90.0,
                v_i=0.0,
                v_f=0.0
            ),
            "medium": TrapezoidalProfile(
                ds_tot=720.0,
                a_max=1080.0,
                v_max=360.0,
                v_i=0.0,
                v_f=0.0
            ),
            "fast": TrapezoidalProfile(
                ds_tot=720.0,
                a_max=1080.0,
                v_max=720.0,
                v_i=0.0,
                v_f=0.0
            )
        }

        self.motion_controller = XYZMotionController(
            master=self,
            config_filepath="motor_config.toml",
            logger=self.logger,
            jog_mode_profile=self.jog_mode_profiles["medium"]
        )
        self.motion: MotionStatus | None = None

        self.auto = AutomaticMode(self)
        self.jog = JogMode(self)

        self.init_flag = True

        self.X0 = self.add_marker("Main.X0")
        self.X1 = self.add_marker("Main.X1")
        self.X2 = self.add_marker("Main.X2")

        # Inputs from HMI
        self.auto_start = self.hmi_input_register["auto_start"]
        self.jog_start = self.hmi_input_register["jog_start"]
        self.jog_xf = self.hmi_input_register["jog_x+"]
        self.jog_xb = self.hmi_input_register["jog_x-"]
        self.jog_yf = self.hmi_input_register["jog_y+"]
        self.jog_yb = self.hmi_input_register["jog_y-"]
        self.jog_zf = self.hmi_input_register["jog_z+"]
        self.jog_zb = self.hmi_input_register["jog_z-"]
        self.emergency = self.hmi_input_register["emergency"]

        # Outputs to HMI
        self.x_motor_finished = self.hmi_output_register["x_motor_finished"]
        self.y_motor_finished = self.hmi_output_register["y_motor_finished"]
        self.x_motor_ready = self.hmi_output_register["x_motor_ready"]
        self.y_motor_ready = self.hmi_output_register["y_motor_ready"]
        self.x_travel_time = self.hmi_output_register["x_travel_time"]
        self.y_travel_time = self.hmi_output_register["y_travel_time"]
        self.z_travel_time = self.hmi_output_register["z_travel_time"]

    def _init_control(self):
        if self.init_flag:
            self.init_flag = False
            self.motion_controller.enable()
            self.X0.activate()

    def _sequence_control(self):
        if self.X0.active and self.hmi_data.data["plc_state"] == "idle":
            if self.hmi_data.data["mode"] == "auto" and self.auto_start.active:
                self.X0.deactivate()
                self.X1.activate()
            if self.hmi_data.data["mode"] == "jog" and self.jog_start.active:
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
        if self.X0.active:
            self.hmi_data.data["plc_state"] = "idle"

        if self.X1.active:
            if self.X1.rising_edge:
                self.logger.info("Auto mode active.")
            self.auto.call()

        if self.X2.active:
            if self.X2.rising_edge:
                self.logger.info("Jog mode active.")
            self.jog.call()

    def control_routine(self) -> None:
        if self.emergency.active:
            raise EmergencyException

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
