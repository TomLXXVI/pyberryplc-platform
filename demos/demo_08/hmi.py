import io
import csv
import tomllib
import asyncio

import plotly.graph_objects as go
from nicegui import run

from pyberryplc.hmi import AbstractHMI
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.core.memory import HMISharedData

from plc import MotionPLC


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

        self.trajectory_plotter = TrajectoryPlotlyPlotter(self.ui)
        self.points = []
        self.trajectory = None

        self.motor_config = self._read_motor_configurations()
        self.alpha_max_slider = None
        self.omega_max_slider = None
        self.cont_motion_switch = None
        self._busy_flag = False
        self.pos = None
        self.vel = None
        self.acc = None
        self.stepper_signals = []
        self.motion_profile_plotter = MotionProfilePlotlyPlotter(self.ui)
        self.motion_profile_plot = None
        self.selected_profile = "velocity"

    @classmethod
    def build(cls, parent):
        self = cls(parent)

        with self.ui.tab_panel("Automatic Mode"):
            self.ui.label("Automatic Mode").classes("text-xl mb-2")

            with self.ui.row():
                with self.ui.column():
                    # Trajectory CSV file upload widget
                    self._build_csv_upload_widget()

                    # Slider panel alpha_max and omega_max
                    self._build_motion_profile_sliders()

                    # Switch continuous motion
                    self._build_cont_motion_switch()

                with self.ui.column():
                    with self.ui.row():
                        # Trajectory plot
                        self._build_trajectory_plot()

                        # Motion profile viewer
                        self._build_motion_profile_plot()
                self.ui.separator()

            with self.ui.row():
                self.ui.button("Send to PLC", on_click=self._send_to_plc)
                self.ui.button("EMERGENCY STOP", color="red", on_click=self._raise_emergency)

        return self

    def _build_csv_upload_widget(self):
        with self.ui.column().classes("border rounded p-4"):
            self.ui.upload(
                label="Select Trajectory CSV",
                auto_upload=True,
                max_files=1,
                on_upload=self._on_csv_upload
            ).props("accept=.csv").classes("mb-4 w-full")

    def _build_motion_profile_sliders(self):
        with self.ui.column().classes("border rounded p-4 w-full"):
            self.ui.label("Maximum acceleration (°/s²)")
            self.alpha_max_slider = self.ui.slider(
                min=100.0, max=5000.0, step=100.0, value=1500.0,
                on_change=self._update_contents
            )
            self.ui.label().bind_text_from(self.alpha_max_slider, "value")

            self.ui.label("Maximum speed (°/s)")
            self.omega_max_slider = self.ui.slider(
                min=100.0, max=5000.0, step=100.0, value=1500.0,
                on_change=self._update_contents
            )
            self.ui.label().bind_text_from(self.omega_max_slider, "value")

    def _build_cont_motion_switch(self):
        with self.ui.column().classes("border rounded p-4 w-full"):
            self.cont_motion_switch = self.ui.switch(
                "continuous motion",
                value=False,
                on_change=self._update_contents
            )

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

    async def _on_csv_upload(self, event_args):
        try:
            self.points.clear()
            self.points = self._read_points_from_csv(event_args.content)
            if len(self.points) < 2:
                self.ui.notify("Not enough valid points in file.", color="red")
                return
            await self._update_contents()
        except Exception as ex:
            self.ui.notify(f"Error while reading file: {ex}", color="red")

    def _send_to_plc(self) -> None:
        if not self.trajectory:
            self.ui.notify("No trajectory to send.", color="red")
            return
        self.parent.hmi_data.data["trajectory"] = self.stepper_signals
        self.parent.hmi_data.buttons["auto_start"] = True
        self.ui.notify("Trajectory sent to PLC.")

    def _raise_emergency(self) -> None:
        self.parent.hmi_data.buttons["emergency"] = True

    def _on_tab_change(self, event_args) -> None:
        self.selected_profile = event_args.value.lower()
        self._show_motion_profile()

    @staticmethod
    def _read_points_from_csv(content):
        text_stream = io.TextIOWrapper(content, encoding="utf-8")
        reader = csv.reader(text_stream)
        points = []
        for row in reader:
            if len(row) >= 2:
                try:
                    x = float(row[0])
                    y = float(row[1])
                    points.append((x, y))
                except ValueError:
                    continue
        return points

    @staticmethod
    def _read_motor_configurations() -> dict:
        from pyberryplc.motion import RotationDirection

        data = tomllib.load(open("motor_config.toml", "rb"))
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

    @staticmethod
    def _compute_trajectory(
        points: list[tuple[float, float]],
        alpha_max: tuple[float, float],
        omega_max: tuple[float, float],
        cont_motion: bool,
        motor_config: dict
    ) -> dict:
        from pyberryplc.motion import PointToPointTrajectory, SCurvedProfile

        points = [(x / 1000, y / 1000) for x, y in points]  # mm -> m
        trajectory = PointToPointTrajectory(
            n_axes=2,
            profile_type=SCurvedProfile,
            pitch=motor_config["pitch"],
            rdir_ref=motor_config["rdir_ref"],
            alpha_max=alpha_max,
            omega_max=omega_max,
            full_steps_per_rev=motor_config["full_steps_per_rev"],
            microstep_factor=motor_config["microstep_factor"]
        )
        trajectory(*points, continuous=cont_motion)
        delays = trajectory.get_stepper_signals()

        return {
            "trajectory": trajectory,
            "pos": trajectory.position_profiles,
            "vel": trajectory.velocity_profiles,
            "acc": trajectory.acceleration_profiles,
            "delays": delays
        }

    def _show_trajectory(self):
        if not self.points:
            self.ui.notify("No points loaded.", color="red")
            return

        x_vals, y_vals = zip(*self.points)
        if self.trajectory:
            self.trajectory_plotter.draw_figure(
                x_vals, y_vals,
                self.trajectory.get_coordinates()
            )
        else:
            self.trajectory_plotter.draw_figure(x_vals, y_vals, None)
        self.trajectory_plot.update()

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

    async def _update_contents(self):
        if self._busy_flag:
            return

        self._busy_flag = True

        self.parent.spinner_overlay.open()
        await asyncio.sleep(0)

        try:
            result = await run.cpu_bound(
                self._compute_trajectory,
                self.points,
                (self.alpha_max_slider.value, self.alpha_max_slider.value),
                (self.omega_max_slider.value, self.omega_max_slider.value),
                self.cont_motion_switch.value,
                self.motor_config
            )
        except Exception as e:
            self.ui.notify(f"Error: {e}", color="red")
            return
        finally:
            self.parent.spinner_overlay.close()
            self._busy_flag = False

        self.trajectory = result["trajectory"]
        self.pos = result["pos"]
        self.vel = result["vel"]
        self.acc = result["acc"]
        self.stepper_signals = result["delays"]

        self._show_motion_profile()
        self._show_trajectory()


class JogModePanel:

    def __init__(self, parent: 'CNCHMI'):
        self.parent = parent
        self.ui = parent.ui

    @classmethod
    def build(cls, parent):
        self = cls(parent)

        with self.ui.tab_panel("Jog Mode"):
            with self.ui.column():
                self.ui.label("Jog Mode").classes("text-xl mb-2")

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
