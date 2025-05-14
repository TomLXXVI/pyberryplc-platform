import math

from pyberryplc.hmi import AbstractHMI, ErrorDialog
from pyberryplc.core import SharedData
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.motion.control import xy_motion_control

from plc_app import XYMotionPLC


class XYMotionHMI(AbstractHMI):
    
    def __init__(self, app, ui):
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
        
        super().__init__(
            title="XY Motion Demo",
            app=app,
            ui=ui,
            shared_data=self.shared,
            plc_app=XYMotionPLC,
            logger=init_logger(console=False),
            port=8081
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
   
    def build_gui(self) -> None:
        self.ui.label(self.title).classes("text-2xl font-bold mb-4")
        
        with self.ui.column():
            self.ui.label("Start Position").classes("font-bold")
            with self.ui.row():
                self.p1_x = self.ui.number("X1", value=0.0)
                self.p1_y = self.ui.number("Y1", value=0.0)

        with self.ui.column():
            self.ui.label("End Position").classes("font-bold")
            with self.ui.row():
                self.p2_x = self.ui.number("X2", value=0.02)
                self.p2_y = self.ui.number("Y2", value=0.02)
        
        with self.ui.column():
            self.ui.label("Motion Profile").classes("font-bold")
            with self.ui.row():
                self.profile = self.ui.select(["trapezoidal", "S-curved"], value="trapezoidal")
            with self.ui.row():
                self.pitch = self.ui.number("Pitch [rev/m]", value=100)
                self.omega = self.ui.number("Speed [°/s]", value=45.0)
                self.dt_acc = self.ui.number("Acceleration time [s]", value=0.2)

        with self.ui.row():
            self.ui.button("Start", on_click=self._start_motion)
    
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
            msg = f"Calculation of motion profiles failed: {e}"
            ErrorDialog(self, msg).open()
            self.logger.error(msg)
            return
        
        self.shared.hmi_data["motion_profile_x"] = mp_x
        self.shared.hmi_data["motion_profile_y"] = mp_y
        self.shared.hmi_buttons["start_motion"] = True

    def update_status(self) -> None:
        motor_x_ready = self.shared.hmi_digital_outputs["motor_x_ready"]
        motor_y_ready = self.shared.hmi_digital_outputs["motor_y_ready"]
        motor_x_busy = self.shared.hmi_digital_outputs["motor_x_busy"]
        motor_y_busy = self.shared.hmi_digital_outputs["motor_y_busy"]
        travel_time_x = self.shared.hmi_analog_outputs["travel_time_x"]
        travel_time_y = self.shared.hmi_analog_outputs["travel_time_y"]
        
        if motor_x_ready and not motor_x_busy:
            motor_x_status_text = "ready"
        elif motor_x_busy:
            motor_x_status_text = "running"
        elif not math.isnan(travel_time_x):
            motor_x_status_text = f"travel time {travel_time_x:.3f} s"
        else:
            motor_x_status_text = "no info"
        
        if motor_y_ready and not motor_y_busy:
            motor_y_status_text = "ready"
        elif motor_y_busy:
            motor_y_status_text = "running"
        elif not math.isnan(travel_time_y):
            motor_y_status_text = f"travel time {travel_time_y:.3f} s"
        else:
            motor_y_status_text = "no info"
        
        status_html = ""
        if motor_x_status_text:
            status_html += f"<b>Motor X:</b> {motor_x_status_text}"
        if motor_y_status_text:
            status_html += " | "
            status_html += f"<b>Motor Y:</b> {motor_y_status_text}"
        
        self.status_html.set_content(status_html)
