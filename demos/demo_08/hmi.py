import io
import csv
import tomllib

import plotly.graph_objects as go

from pyberryplc.hmi import AbstractHMI
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.motion import PointToPointTrajectory, SCurvedProfile, RotationDirection
from pyberryplc.core.memory import HMISharedData

from plc import MotionPLC


hmi_data = HMISharedData(
    buttons={
        "auto_start": False,
        "jog_start": False
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
        "x_travel_time": float("nan"),
        "y_travel_time": float("nan"),
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
                title=None,
                tickfont=dict(family="Roboto, sans-serif", size=14),
                scaleanchor="y",
                scaleratio=1
            ),
            yaxis=dict(
                title=None,
                tickfont=dict(family="Roboto, sans-serif", size=14),
                scaleanchor="x",
                scaleratio=1
            ),
            margin=dict(l=40, r=40, t=40, b=40)
        )

    def create_plot(self):
        return self.ui.plotly(self.fig).classes(f"w-[{self.w}] h-[{self.h}]")

    def draw_figure(self, x, y):
        self.fig.data = []  # Clear previous plot
        self.fig.add_trace(go.Scatter(
            x=x,
            y=y,
            mode="lines+markers",
            name="Trajectory"
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

    def __init__(self, parent):
        self.parent = parent
        self.ui = parent.ui
        self.trajectory_plotter = TrajectoryPlotlyPlotter(self.ui)
        self.points: list[tuple[float, ...]] = []

    @classmethod
    def build(cls, parent):
        self = cls(parent)

        with self.ui.tab_panel("Automatic Mode"):
            with self.ui.column().classes("border rounded p-4 w-full"):
                self.ui.label("Automatic Mode").classes("text-xl mb-2")
                # Trajectory CSV file upload widget
                self._build_upload_widget()
                # Trajectory plot
                self._build_trajectory_plot()

                with self.ui.column():
                    self.ui.label("Maximum acceleration (°/s²)")
                    alpha_max_slider = self.ui.slider(
                        min=100.0, max=5000.0, step=100.0, value=1500.0,
                    )
                    self.ui.label().bind_text_from(alpha_max_slider, "value")

                    self.ui.label("Maximum speed (°/s)")
                    omega_max_slider = self.ui.slider(
                        min=100.0, max=5000.0, step=100.0, value=1500.0,
                    )
                    self.ui.label().bind_text_from(omega_max_slider, "value")

                self.ui.separator()
                with self.ui.row():
                    self.ui.button(
                        "Verify Motion Profiles",
                        on_click=lambda: self.parent.profile_dialog.open(
                            self.points,
                            (alpha_max_slider.value, alpha_max_slider.value),
                            (omega_max_slider.value, omega_max_slider.value)
                        )
                    )
                    self.parent.send_button = self.ui.button(
                        "Send to PLC",
                        on_click=self._send_to_plc
                    )
                    self.parent.set_button_state(self.parent.send_button, enabled=False)
        return self

    def _build_upload_widget(self):
        self.ui.upload(
            label="Select Trajectory CSV",
            auto_upload=True,
            max_files=1,
            on_upload=self._on_upload
        ).props("accept=.csv").classes("mb-4 w-full")

    def _build_trajectory_plot(self):
        self.ui.label("Planned trajectory").classes("text-xl font-bold")
        with self.ui.row().classes("items-center"):
            self.ui.label("Y [mm]") \
                .classes("text-base mr-1") \
                .style("width: 2rem; text-align: center;")
            with self.ui.column().classes("items-center"):
                self.trajectory_plot = self.trajectory_plotter.create_plot()
                self.ui.label("X [mm]").classes("text-base mt-1")

    def _on_upload(self, event_args):
        try:
            self.points.clear()
            text_stream = io.TextIOWrapper(event_args.content, encoding="utf-8")
            reader = csv.reader(text_stream)
            for row in reader:
                if len(row) >= 2:
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        self.points.append((x, y))
                    except ValueError:
                        continue

            if len(self.points) < 2:
                self.ui.notify("Not enough valid points in file.", color="red")
                return

            self.ui.notify(f"Loaded {len(self.points)} points from file.")
            self._show_trajectory()

        except Exception as ex:
            self.ui.notify(f"Error while reading file: {ex}", color="red")

    def _show_trajectory(self):
        if not self.points:
            self.ui.notify("No points loaded.", color="red")
            return

        x_vals, y_vals = zip(*self.points)
        self.trajectory_plotter.draw_figure(x_vals, y_vals)
        self.trajectory_plot.update()
        self.ui.notify("Trajectory plot updated.")

    def _send_to_plc(self) -> None:
        if not self.parent.trajectory:
            self.ui.notify("No trajectory to send.", color="red")
            return
        self.parent.hmi_data.data["trajectory"] = self.parent.trajectory.get_stepper_signals()
        self.parent.hmi_data.buttons["auto_start"] = True
        self.ui.notify("Trajectory sent to PLC.")


class MotionProfileDialog:

    def __init__(self, parent) -> None:
        self.parent = parent
        self.ui = self.parent.ui
        self.selected_profile = "velocity"
        self.pos = None
        self.vel = None
        self.acc = None
        self.points = []
        self.motion_profile_plotter = MotionProfilePlotlyPlotter(self.ui)
        self.motor_config: dict = {}

        self._get_motor_configurations()
        self._create_overlay()
        self._create_dialog()

    def _get_motor_configurations(self):
        data = tomllib.load(open("motor_config.toml", "rb"))
        x_motor_data = data["x_motor"]
        y_motor_data = data["y_motor"]
        x_full_steps_per_rev = x_motor_data["microstepping"]["full_steps_per_rev"]
        y_full_steps_per_rev = y_motor_data["microstepping"]["full_steps_per_rev"]
        x_microstep_factor = int(x_motor_data["microstepping"]["resolution"].split("/")[-1])
        y_microstep_factor = int(y_motor_data["microstepping"]["resolution"].split("/")[-1])
        x_pitch = x_motor_data["pitch"]
        y_pitch = y_motor_data["pitch"]
        x_rdir_ref = RotationDirection.CW if x_motor_data["rdir_ref"] == "clockwise" else RotationDirection.CCW
        y_rdir_ref = RotationDirection.CW if y_motor_data["rdir_ref"] == "clockwise" else RotationDirection.CCW
        self.motor_config = {
            "full_steps_per_rev": (x_full_steps_per_rev, y_full_steps_per_rev),
            "microstep_factor": (x_microstep_factor, y_microstep_factor),
            "pitch": (x_pitch, y_pitch),
            "rdir_ref": (x_rdir_ref, y_rdir_ref),
        }

    def _create_dialog(self) -> None:
        self._dialog = self.ui.dialog()
        with self._dialog:
            with self.ui.card().classes("w-full"):
                self.ui.label("Motion Profile Viewer").classes("text-lg font-bold")

                with self.ui.tabs(value="Velocity", on_change=self._on_tab_change).props("dense"):
                    self.ui.tab("Velocity")
                    self.ui.tab("Position")
                    self.ui.tab("Acceleration")

                self.motion_profile_plot = self.motion_profile_plotter.create_plot()

                self.ui.button("Close", on_click=self._dialog.close)
        self._dialog.close()

    def _create_overlay(self) -> None:
        self._overlay = self.ui.dialog()
        with self._overlay.props("persistent"):
            with self.ui.card().classes("w-full items-center"):
                self.ui.spinner(size="lg")
                self.ui.label("Processing trajectory... Please wait.")
        self._overlay.close()

    def _on_tab_change(self, event_args) -> None:
        # select profile: velocity, position, or acceleration
        self.selected_profile = event_args.value.lower()
        self._show_motion_profile()

    def _show_motion_profile(self) -> None:
        if not self.vel:
            self.ui.notify("No motion profile available.", color="red")
            return

        if self.selected_profile == "position":
            t_x = self.pos["x"][0]
            y_x = self.pos["x"][1] * 1000  # mm
            t_y = self.pos["y"][0]
            y_y = self.pos["y"][1] * 1000
            y_label = "position, mm"
        elif self.selected_profile == "acceleration":
            t_x = self.acc["x"][0]
            y_x = self.acc["x"][1] * 1000  # mm/s²
            t_y = self.acc["y"][0]
            y_y = self.acc["y"][1] * 1000
            y_label = "acceleration, mm/s²"
        else:
            t_x = self.vel["x"][0]
            y_x = self.vel["x"][1] * 1000  # mm/s
            t_y = self.vel["y"][0]
            y_y = self.vel["y"][1] * 1000
            y_label = "velocity, mm/s"

        self.motion_profile_plotter.draw_figure(x=(t_x, t_y), y=(y_x, y_y))
        self.motion_profile_plotter.set_figure_layout(y_label)
        self.motion_profile_plot.update()

    async def open(
        self,
        points: list[tuple[float, float]],
        alpha_max: tuple[float, float],
        omega_max: tuple[float, float]
    ):
        # first show the overlay with spinner during calculation of motion
        # profiles
        self._overlay.open()
        try:
            await self._compute_trajectory(points, alpha_max, omega_max)
        except Exception as ex:
            self.ui.notify(
                f"Failed to generate motion profiles: {ex}",
                color="red"
            )
        self._overlay.close()

        # when calculations are done, show motion profile dialog...
        self.selected_profile = "velocity"
        self._show_motion_profile()
        self._dialog.open()

    async def _compute_trajectory(
        self,
        points: list[tuple[float, float]],
        alpha_max: tuple[float, float],
        omega_max: tuple[float, float]
    ):
        points = [(x / 1000, y / 1000) for x, y in points]  # mm -> m

        self.parent.trajectory = PointToPointTrajectory(
            n_axes=2,
            profile_type=SCurvedProfile,
            pitch=self.motor_config["pitch"],
            rdir_ref=self.motor_config["rdir_ref"],
            alpha_max=alpha_max,
            omega_max=omega_max,
            full_steps_per_rev=self.motor_config["full_steps_per_rev"],
            microstep_factor=self.motor_config["microstep_factor"]
        )
        self.parent.trajectory(*points)
        self.pos = self.parent.trajectory.position_profiles
        self.vel = self.parent.trajectory.velocity_profiles
        self.acc = self.parent.trajectory.acceleration_profiles

        self.parent.set_button_state(self.parent.send_button, enabled=True)


class JogModePanel:

    def __init__(self, parent):
        self.parent = parent
        self.ui = parent.ui

    @classmethod
    def build(cls, parent):
        self = cls(parent)

        with self.ui.tab_panel("Jog Mode"):
            with self.ui.column().classes("border rounded p-4 w-full"):
                self.ui.label("Jog Mode").classes("text-xl mb-2")

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
        self.ui.notify(f"Jog speed set to {value}")


class CNCHMI(AbstractHMI):

    def __init__(self, app, ui):
        self.aut_panel = None
        self.jog_panel = None
        self.profile_dialog = None
        self.trajectory = None
        self.hmi_data = hmi_data
        self.send_button = None

        super().__init__(
            title="CNC HMI",
            app=app,
            ui=ui,
            shared_data=self.hmi_data,
            plc_app=MotionPLC,
            logger=init_logger(console=False)
        )

    def build_gui(self) -> None:
        self.profile_dialog = MotionProfileDialog(self)

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
        self.ui.notify(f"Mode switched to {mode.capitalize()}")

    def update_status(self):
        plc_state = self.hmi_data.data["plc_state"]
        mode = self.hmi_data.data["mode"]
        self.status_html.set_content(f"<b>{mode.upper()} | {plc_state.upper()}</b>")
