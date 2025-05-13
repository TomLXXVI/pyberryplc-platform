import math

from nicegui import ui

from pyberryplc.hmi import AbstractHMI, ErrorDialog
from pyberryplc.core import SharedData
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.motion.control import xy_motion_control

from plc_app import XYMotionPLC


class XYMotionHMI(AbstractHMI):
    
    def __init__(self):
        self.shared = SharedData(
            hmi_buttons={"start_motion": False},
            hmi_data={
                "motion_profile_x": None,
                "motion_profile_y": None
            },
            hmi_digital_outputs={
                "motor_x_ready": False,
                "motor_x_busy": False,
                "motor_y_ready": False,
                "motor_y_busy": False,
            },
            hmi_analog_outputs={
                "travel_time_x": float('nan'),
                "travel_time_y": float('nan'),
            }
        )
        # GUI-widgets that need a reference.
        self.p1_x: ui.number | None = None
        self.p1_y: ui.number | None = None
        self.p2_x: ui.number | None = None
        self.p2_y: ui.number | None = None
        self.pitch: ui.number | None = None
        self.omega: ui.number | None = None
        self.dt_acc: ui.number | None = None
        self.profile: ui.select | None = None
        self.travel_time_x: ui.label | None = None
        self.travel_time_y: ui.label | None = None
        self.error_dialog: ErrorDialog | None = None
        
        super().__init__(
            title="XY Motion Demo",
            shared_data=self.shared,
            plc_app=XYMotionPLC,
            logger=init_logger(console=False),
            port=8081
        )
    
    def build_ui(self) -> None:
        ui.label(self.title).classes("text-2xl font-bold mb-4")
        
        with ui.row():
            ui.label("Start position:")
            self.p1_x = ui.number("X1", value=0.0)
            self.p1_y = ui.number("Y1", value=0.0)

        with ui.row():
            ui.label("End position:")
            self.p2_x = ui.number("X2", value=0.02)
            self.p2_y = ui.number("Y2", value=0.02)

        with ui.row():
            self.pitch = ui.number("Pitch [rev/m]", value=100)
            self.omega = ui.number("Speed [°/s]", value=45.0)
            self.dt_acc = ui.number("Acceleration time [s]", value=0.2)

        with ui.row():
            self.profile = ui.select(["trapezoidal", "S-curved"], value="trapezoidal")

        with ui.row():
            ui.button("Start", on_click=self._start_motion)
            ui.button("Exit HMI", on_click=self.exit_hmi)
        
        with ui.column():
            self.travel_time_x = ui.label("Travel time X-axis")
            self.travel_time_y = ui.label("Travel time Y-axis")
    
    def update_status(self) -> None:
        travel_time_x = self.shared.hmi_analog_outputs["travel_time_x"]
        travel_time_y = self.shared.hmi_analog_outputs["travel_time_y"]
        if not math.isnan(travel_time_x):
            self.travel_time_x.text = f"Travel time X-axis: {travel_time_x:.3f} s"
        if not math.isnan(travel_time_y):
            self.travel_time_y.text = f"Travel time Y-axis: {travel_time_y:.3f} s"
        
    def _start_motion(self) -> None:
        p1 = (self.p1_x.value, self.p1_y.value)
        p2 = (self.p2_x.value, self.p2_y.value)
        pitch = self.pitch.value
        omega = self.omega.value
        dt_acc = self.dt_acc.value
        profile_type = self.profile.value
        
        try:
            mp_x, mp_y = xy_motion_control(p1, p2, pitch, omega, dt_acc, profile_type)
        except Exception as e:
            msg = f"Creation of motion profiles failed: {e}"
            self.error_dialog = ErrorDialog(msg).open()
            self.logger.error(msg)
            return
        
        self.shared.hmi_data["motion_profile_x"] = mp_x
        self.shared.hmi_data["motion_profile_y"] = mp_y
        self.shared.hmi_buttons["start_motion"] = True
    