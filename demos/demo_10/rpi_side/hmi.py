from pathlib import Path
import json
import tomllib
import hashlib

import plotly.graph_objects as go

from pyberryplc.hmi import AbstractHMI
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.core.memory import HMISharedData
from pyberryplc.motion import RotationDirection, PointToPointTrajectory, SCurvedProfile

from plc import MotionPLC


BASE_DIR = Path(__file__).parent.resolve()
SAVE_DIRECTORY = BASE_DIR / 'trajectory_outputs'
SAVE_DIRECTORY.mkdir(parents=True, exist_ok=True)


hmi_data = HMISharedData(
    buttons={
        "auto_start": False,
        "jog_start": False,
        "emergency": False
    },
    switches={
        "jog_x+": False,
        "jog_x-": False,
        "jog_y+": False,
        "jog_y-": False,
        "jog_z+": False,
        "jog_z-": False,
    },
    digital_outputs={
        "x_motor_finished": False,
        "x_motor_ready": False,
        "y_motor_finished": False,
        "y_motor_ready": False,
    },
    analog_outputs={
        "x_travel_time": 0.0,
        "y_travel_time": 0.0,
        "z_travel_time": 0.0
    },
    data={
        "mode": "",
        "plc_state": "",
        "jog_speed": "medium",
        "trajectory": None,
        "segment_count": 0,
    }
)


class TrajectoryPlotlyPlotter:

    def __init__(self, ui, w="500px", h="500px"):
        self.ui = ui
        self.fig = go.Figure(go.Scatter(x=[0.0], y=[0.0]))
        self.set_figure_layout()
        self.w = w
        self.h = h

    def set_figure_layout(self):
        self.fig.update_layout(
            title=None,
            xaxis=dict(
                title=dict(
                    text="x-axis",
                    font=dict(family="Helvetica, Arial, sans-serif", size=13)
                ),
                tickfont=dict(family="Helvetica, Arial, sans-serif", size=13),
            ),
            yaxis=dict(
                title=dict(
                    text="y-axis",
                    font=dict(family="Helvetica, Arial, sans-serif", size=13)
                ),
                tickfont=dict(family="Helvetica, Arial, sans-serif", size=13),
            ),
            legend=dict(
                orientation="h",
                x=0.5,
                y=-0.2,
                xanchor="center",
                font=dict(
                    family="Helvetica, Arial, sans-serif",
                    size=13
                )
            ),
            margin=dict(l=40, r=40, t=40, b=40)
        )

    def create_plot(self):
        return self.ui.plotly(self.fig).classes(f"w-[{self.w}] h-[{self.h}]")

    def draw_figure(self, x, y, coords):
        self.fig.data = []  # Clear previous plot
        self.fig.add_trace(go.Scatter(
            x=x,
            y=y,
            mode="lines+markers",
            name="desired trajectory"
        ))
        if coords is not None:
            self.fig.add_trace(go.Scatter(
                x=coords[0] * 1000,
                y=coords[1] * 1000,
                mode="lines",
                name="actual trajectory"
            ))


class MotionProfilePlotlyPlotter:

    def __init__(self, ui, w="500px"):
        self.ui = ui
        self.fig = go.Figure()
        self.set_figure_layout()
        self.w = w

    def set_figure_layout(self, ylabel: str = "velocity, mm/s"):
        self.fig.update_layout(
            title=None,
            margin=dict(t=40, b=40, l=40, r=40),
            xaxis=dict(
                title=dict(
                    text="time, s",
                    font=dict(family="Helvetica, Arial, sans-serif", size=13)
                ),
                tickfont=dict(family="Helvetica, Arial, sans-serif", size=13),
            ),
            yaxis=dict(
                title=dict(
                    text=ylabel,
                    font=dict(family="Helvetica, Arial, sans-serif", size=13)
                ),
                tickfont=dict(family="Helvetica, Arial, sans-serif", size=13),
            ),
            legend=dict(
                orientation="h",
                x=0.5,
                y=-0.2,
                xanchor="center",
                font=dict(
                    family="Helvetica, Arial, sans-serif",
                    size=13
                )
            )
        )

    def create_plot(self):
        return self.ui.plotly(self.fig).classes(f"w-[{self.w}]")

    def draw_figure(self, x: tuple, y: tuple):
        self.fig.data = []  # Clear previous plot
        self.fig.add_trace(go.Scatter(
            x=x[0],
            y=y[0],
            mode="lines",
            name="X-axis"
        ))
        self.fig.add_trace(go.Scatter(
            x=x[1],
            y=y[1],
            mode="lines",
            name="Y-axis"
        ))


class AutomaticPanel:

    def __init__(self, parent: 'CNCHMI'):
        self.parent = parent
        self.ui = parent.ui

        self.json_file_select = None
        self.selected_file = ""

        self.signals_path = None
        self.meta_path = None

        self.points_mm = []
        self.trajectory = None
        self.pos_profile = None
        self.vel_profile = None
        self.acc_profile = None
        self.stepper_signals = None

        self.selected_profile = None

        self.trajectory_plotter = TrajectoryPlotlyPlotter(self.ui)
        self.trajectory_plot = None

        self.motion_profile_plotter = MotionProfilePlotlyPlotter(self.ui)
        self.motion_profile_plot = None

        self.motor_config = self._read_motor_configurations()

    @classmethod
    def build(cls, parent):
        self = cls(parent)

        with self.ui.tab_panel("Automatic Mode"):
            with self.ui.row():
                with self.ui.column().classes("gap-4"):
                    # JSON file select widget
                    self._build_file_select_widget()

                with self.ui.column().classes("gap-4"):
                    with self.ui.row():
                        # Trajectory plot
                        self._build_trajectory_plot()

                        # Motion profile viewer
                        self._build_motion_profile_plot()

                self.ui.separator()

            with self.ui.row():
                self.ui.button("Send to PLC", on_click=self._send_to_plc, color="primary")
                self.ui.button("EMERGENCY STOP", on_click=self._raise_emergency, color="red")

        return self

    def _build_file_select_widget(self):
        with self.ui.column().classes("w-full border rounded p-4"):
            self.json_file_select = self.ui.select(
                options=self._list_json_files(),
                label="Select JSON trajectory",
                on_change=self._set_selected_file
            ).classes("w-full")

            self.ui.button(
                "Load trajectory",
                on_click=self._load_selected_json,
                color="primary"
            ).classes("mt-2")

    def _build_trajectory_plot(self):
        with self.ui.column().classes("border rounded p-4"):
            self.ui.label("Trajectory").classes("text-lg font-bold")
            self.trajectory_plot = self.trajectory_plotter.create_plot()

    def _build_motion_profile_plot(self):
        with self.ui.column().classes("border rounded p-4"):
            self.ui.label("Motion Profiles").classes("text-lg font-bold")
            with self.ui.tabs(value="Velocity", on_change=self._on_tab_change).props("dense"):
                self.ui.tab("Velocity")
                self.ui.tab("Position")
                self.ui.tab("Acceleration")
            self.motion_profile_plot = self.motion_profile_plotter.create_plot()

    @staticmethod
    def _list_json_files() -> list[str]:
        return sorted([
            f.name.removesuffix(".signals.json")
            for f in SAVE_DIRECTORY.glob("*.signals.json")
        ])

    def _set_selected_file(self, event_args):
        self.selected_file = event_args.value

    def _load_selected_json(self) -> None:
        if not self.selected_file:
            self.ui.notify("No file selected.", color="red")
            return

        self.signals_path = SAVE_DIRECTORY / f"{self.selected_file}.signals.json"
        self.meta_path = SAVE_DIRECTORY / f"{self.selected_file}.meta.json"

        if not self.signals_path.exists() or not self.meta_path.exists():
            self.ui.notify(f"File not found: {self.selected_file}", color="red")
            return

        self._load_stepper_signals()
        self._load_trajectory()
        self.ui.notify(f"Loaded {self.selected_file}", color="green")

    def _load_stepper_signals(self):
        with open(self.signals_path, 'r') as f:
            self.stepper_signals = json.load(f)

    def _load_trajectory(self):
        with open(self.meta_path, "r") as f:
            metadata = json.load(f)

        cfg_bytes = (BASE_DIR / "motor_config.toml").read_bytes()
        current_hash = hashlib.sha256(cfg_bytes).hexdigest()
        if current_hash != metadata["motor_config_sha256"]:
            self.ui.notify(
                "Warning: motor_config differs from planner_config.",
                color="orange"
            )

        try:
            profile_type = SCurvedProfile if metadata["profile_type"] == "SCurvedProfile" else None
            self.trajectory = PointToPointTrajectory(
                n_axes=2,
                profile_type=profile_type,
                pitch=self.motor_config["pitch"],
                rdir_ref=self.motor_config["rdir_ref"],
                alpha_max=tuple(metadata["alpha_max"]),
                omega_max=tuple(metadata["omega_max"]),
                full_steps_per_rev=self.motor_config["full_steps_per_rev"],
                microstep_factor=self.motor_config["microstep_factor"]
            )
            self.points_mm = metadata["points"]
            points_m = [(x / 1000, y / 1000) for x, y in self.points_mm]
            self.trajectory(*points_m, continuous=metadata["continuous_motion"])
            self.pos_profile = self.trajectory.position_profiles
            self.vel_profile = self.trajectory.velocity_profiles
            self.acc_profile = self.trajectory.acceleration_profiles

            self.selected_profile = "velocity"
            self._show_trajectory()
            self._show_motion_profile()

        except Exception as e:
            self.ui.notify(f"Failed to load: {e}", color="red")

    def _show_trajectory(self):
        if not self.points_mm:
            self.ui.notify("No trajectory loaded.", color="red")
            return

        x_vals, y_vals = zip(*self.points_mm)
        if self.trajectory:
            self.trajectory_plotter.draw_figure(
                x_vals, y_vals,
                self.trajectory.get_coordinates()
            )
        else:
            self.trajectory_plotter.draw_figure(x_vals, y_vals, None)
        self.trajectory_plot.update()

    def _show_motion_profile(self) -> None:
        if not self.vel_profile:
            self.ui.notify("No motion profile available.", color="red")
            return

        if self.selected_profile == "position":
            t_x = self.pos_profile["x"][0]
            y_x = self.pos_profile["x"][1] * 1000  # mm
            t_y = self.pos_profile["y"][0]
            y_y = self.pos_profile["y"][1] * 1000
            y_label = "position, mm"
        elif self.selected_profile == "acceleration":
            t_x = self.acc_profile["x"][0]
            y_x = self.acc_profile["x"][1] * 1000  # mm/s²
            t_y = self.acc_profile["y"][0]
            y_y = self.acc_profile["y"][1] * 1000
            y_label = "acceleration, mm/s²"
        else:
            t_x = self.vel_profile["x"][0]
            y_x = self.vel_profile["x"][1] * 1000  # mm/s
            t_y = self.vel_profile["y"][0]
            y_y = self.vel_profile["y"][1] * 1000
            y_label = "velocity, mm/s"

        self.motion_profile_plotter.draw_figure(x=(t_x, t_y), y=(y_x, y_y))
        self.motion_profile_plotter.set_figure_layout(y_label)
        self.motion_profile_plot.update()

    @staticmethod
    def _read_motor_configurations() -> dict:
        data = tomllib.load(open(BASE_DIR / "motor_config.toml", "rb"))

        x_motor_data = data["x_motor"]
        y_motor_data = data["y_motor"]

        x_full_steps_per_rev = x_motor_data["microstepping"]["full_steps_per_rev"]
        y_full_steps_per_rev = y_motor_data["microstepping"]["full_steps_per_rev"]

        try:
            x_microstep_factor = int(x_motor_data["microstepping"]["resolution"].split("/")[-1])
        except (IndexError, ValueError):
            x_microstep_factor = 1
        try:
            y_microstep_factor = int(y_motor_data["microstepping"]["resolution"].split("/")[-1])
        except (IndexError, ValueError):
            y_microstep_factor = 1

        x_pitch = x_motor_data["pitch"]
        y_pitch = y_motor_data["pitch"]

        x_rdir_ref = RotationDirection.CW if x_motor_data["rdir_ref"] == "clockwise" else RotationDirection.CCW
        y_rdir_ref = RotationDirection.CW if y_motor_data["rdir_ref"] == "clockwise" else RotationDirection.CCW

        return {
            "full_steps_per_rev": (x_full_steps_per_rev, y_full_steps_per_rev),
            "microstep_factor": (x_microstep_factor, y_microstep_factor),
            "pitch": (x_pitch, y_pitch),
            "rdir_ref": (x_rdir_ref, y_rdir_ref),
        }

    def _on_tab_change(self, event_args) -> None:
        self.selected_profile = event_args.value.lower()
        self._show_motion_profile()

    def _send_to_plc(self) -> None:
        if not self.stepper_signals:
            self.ui.notify("No trajectory to send.", color="red")
            return
        self.parent.hmi_data.data["trajectory"] = self.stepper_signals
        self.parent.hmi_data.buttons["auto_start"] = True
        self.ui.notify("Trajectory sent to PLC.")

    def _raise_emergency(self) -> None:
        self.parent.hmi_data.buttons["emergency"] = True


class JogModePanel:

    def __init__(self, parent: 'CNCHMI'):
        self.parent = parent
        self.ui = parent.ui

    @classmethod
    def build(cls, parent):
        self = cls(parent)

        with self.ui.tab_panel("Jog Mode"):
            with self.ui.column():
                with self.ui.column().classes("border rounded p-4"):
                    self.ui.label().bind_text_from(
                        self.parent.hmi_data.data,
                        "jog_speed",
                        lambda v: f"Current jog speed: {v.capitalize()}"
                    )

                    for axis in ["X", "Y", "Z"]:
                        with self.ui.row().classes("gap-2 mb-2"):
                            self.ui.button(f"{axis} -") \
                                .on("mousedown", lambda e, a=axis: self._start_jog(a, "-")) \
                                .on("mouseup", lambda e, a=axis: self._stop_jog(a, "-"))

                            self.ui.button(f"{axis} +") \
                                .on("mousedown", lambda e, a=axis: self._start_jog(a, "+")) \
                                .on("mouseup", lambda e, a=axis: self._stop_jog(a, "+"))

                    self.ui.select(
                        ["Slow", "Medium", "Fast"],
                        value="Medium",
                        label="Jog speed",
                        on_change=lambda e: self._set_jog_speed(e.value)
                    ).classes("w-40")

        return self

    def _start_jog(self, axis: str, direction: str):
        btn_name = f"jog_{axis.lower()}{direction}"
        self.parent.hmi_data.buttons["jog_start"] = True
        self.parent.hmi_data.switches[btn_name] = True

    def _stop_jog(self, axis: str, direction: str):
        btn_name = f"jog_{axis.lower()}{direction}"
        self.parent.hmi_data.switches[btn_name] = False

    def _set_jog_speed(self, value: str):
        self.parent.hmi_data.data["jog_speed"] = value.lower()


class CNCHMI(AbstractHMI):

    def __init__(self, app, ui):
        self.aut_panel = None
        self.jog_panel = None
        self.hmi_data = hmi_data
        self.spinner_overlay = None

        super().__init__(
            title="CNC HMI",
            app=app,
            ui=ui,
            shared_data=self.hmi_data,
            plc_app=MotionPLC,
            logger=init_logger(console=False)
        )

    def build_gui(self) -> None:
        self._create_spinner_overlay()
        self.ui.label("CNC HMI - RATTMMOTOR 1610 Pro").classes("text-2xl mb-4")

        tabs = self.ui.tabs(on_change=self._on_tab_change).classes("mb-4")
        with tabs:
            self.ui.tab("Automatic Mode")
            self.ui.tab("Jog Mode")

        with self.ui.tab_panels(tabs, value="Automatic Mode").classes("w-full"):
            self.aut_panel = AutomaticPanel.build(self)
            self.jog_panel = JogModePanel.build(self)

    def _on_tab_change(self, event_args):
        plc_state = self.hmi_data.data["plc_state"]

        if plc_state != "idle":
            self.ui.notify(f"Cannot switch mode (PLC is {plc_state})", color="red")
            current_mode = self.hmi_data.data["mode"]
            # Force back to current tab mode.
            event_args.sender.value = "Automatic Mode" if current_mode == "auto" else "Jog Mode"
            return

        selected_tab = event_args.value
        mode = "auto" if selected_tab == "Automatic Mode" else "jog"
        self.hmi_data.data["mode"] = mode

    def update_status(self):
        plc_state = self.hmi_data.data["plc_state"]
        mode = self.hmi_data.data["mode"]

        status_msg = f"<b>{mode.upper()} | {plc_state.upper()}</b>"

        self.status_html.set_content(status_msg)

    def _create_spinner_overlay(self) -> None:
        self.spinner_overlay = self.ui.dialog()
        with self.spinner_overlay.props("persistent"):
            with self.ui.card().classes("w-full items-center"):
                self.ui.spinner(size="lg")
                self.ui.label("Processing trajectory... Please wait.")
        self.spinner_overlay.close()
