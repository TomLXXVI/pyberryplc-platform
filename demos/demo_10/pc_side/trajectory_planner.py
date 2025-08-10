import io
import csv
import json
from pathlib import Path
import datetime
import asyncio
import tomllib
import tempfile
import hashlib

import plotly.graph_objects as go

from pyberryplc.motion import PointToPointTrajectory, SCurvedProfile, RotationDirection
from pyberryplc.utils.sftp_loader import SFTPUploader, SFTPDownloader


BASE_DIR = Path(__file__).parent.resolve()
SAVE_DIRECTORY = BASE_DIR / 'trajectory_outputs'
SAVE_DIRECTORY.mkdir(parents=True, exist_ok=True)


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
                x=coords[0] * 1000,  # m -> mm
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


class TrajectoryPlanner:

    def __init__(
        self,
        app,
        ui,
    ):
        self.app = app
        self.ui = ui

        self.alpha_max_slider = None
        self.omega_max_slider = None
        self.cont_motion_switch = None
        self.spinner_overlay = None
        self.filename_input = None

        self.points_mm = []
        self.trajectory = None
        self.pos_profile = None
        self.vel_profile = None
        self.acc_profile = None

        self.selected_profile = None

        self.trajectory_plotter = TrajectoryPlotlyPlotter(self.ui)
        self.trajectory_plot = None

        self.motion_profile_plotter = MotionProfilePlotlyPlotter(self.ui)
        self.motion_profile_plot = None

        self.motor_config = self._read_motor_configurations()

    def run(self):
        self._build_gui()
        self.ui.run(title="Trajectory Planner", port=8081, reload=False, show=False)

    def _build_gui(self):
        self._create_spinner_overlay()

        self.ui.label("Trajectory Planner").classes("text-2xl mb-4")
        with self.ui.row():
            with self.ui.column():
                # Trajectory CSV file upload widget
                self._build_csv_upload_widget()

                # Slider panel with alpha_max and omega_max sliders
                self._build_motion_sliders()

                # Switch for continuous motion on/off
                self._build_cont_motion_switch()

                # Save trajectory to JSON panel
                self._build_save_to_json_button()

            with self.ui.column():
                with self.ui.row():
                    # Trajectory viewer
                    self._build_trajectory_viewer()

                    # Motion profile viewer
                    self._build_motion_profile_viewer()

            self.ui.separator()

        with self.ui.row():
            self.ui.button(
                "Exit App",
                on_click=lambda: exit(0),  # self.app.shutdown,
                color="red"
            )

    def _build_csv_upload_widget(self):
        with self.ui.column().classes("border rounded p-4"):
            self.ui.upload(
                label="Upload Trajectory CSV",
                auto_upload=True,
                max_files=1,
                on_upload=self._on_csv_upload
            ).props("accept=.csv").classes("mb-4 w-full")

    def _build_motion_sliders(self):
        with self.ui.column().classes("border rounded p-4 w-full"):
            self.ui.label("Maximum acceleration (°/s²)")
            self.alpha_max_slider = self.ui.slider(
                min=100.0,
                max=5000.0,
                step=100.0,
                value=1500.0,
                on_change=lambda e: self._update_trajectory()
            )
            self.ui.label().bind_text_from(self.alpha_max_slider, "value")

            self.ui.label("Maximum speed (°/s)")
            self.omega_max_slider = self.ui.slider(
                min=100.0,
                max=5000.0,
                step=100.0,
                value=1500.0,
                on_change=lambda e: self._update_trajectory()
            )
            self.ui.label().bind_text_from(self.omega_max_slider, "value")

    def _build_cont_motion_switch(self):
        with self.ui.column().classes("border rounded p-4 w-full"):
            self.cont_motion_switch = self.ui.switch(
                "continuous motion",
                value=False,
                on_change=lambda e: self._update_trajectory()
            )

    def _build_save_to_json_button(self):
        with self.ui.column().classes("border rounded p-4 w-full"):
            self.filename_input = self.ui.input(
                label="Base name (optional)",
                placeholder="e.g. part_A",

            )
            self.ui.button(
                "Save to JSON",
                on_click=self._save_to_json,
                color="primary"
            )

    def _build_trajectory_viewer(self):
        with self.ui.column().classes("border rounded p-4"):
            self.ui.label("Trajectory").classes("text-lg font-bold")
            self.trajectory_plot = self.trajectory_plotter.create_plot()

    def _build_motion_profile_viewer(self):
        with self.ui.column().classes("border rounded p-4"):
            self.ui.label("Motion Profiles").classes("text-lg font-bold")
            with self.ui.tabs(value="Velocity", on_change=self._on_tab_change).props("dense"):
                self.ui.tab("Velocity")
                self.ui.tab("Position")
                self.ui.tab("Acceleration")
            self.motion_profile_plot = self.motion_profile_plotter.create_plot()

    def _on_csv_upload(self, event_args):
        try:
            self.points_mm.clear()
            self.trajectory = None
            self.selected_profile = None
            self.points_mm = self._read_points_from_csv(event_args.content)
            if len(self.points_mm) < 2:
                self.ui.notify("Not enough valid points in file.", color="red")
                return
            self._update_trajectory()
        except Exception as ex:
            self.ui.notify(f"Error while reading file: {ex}", color="red")

    def _read_motor_configurations(self) -> dict:
        sftp_cfg_path = Path(__file__).parent.resolve() / "sftp.toml"
        local_cfg_path = Path(__file__).parent.resolve() / "motor_config.toml"
        remote_motor_cfg_path = None
        data: dict | None = None
        motor_cfg_bytes = None

        try:
            sftp_cfg = tomllib.load(open(sftp_cfg_path, "rb"))
            remote_motor_cfg_path = sftp_cfg.get("remote_motor_config_path")
        except FileNotFoundError:
            pass

        if remote_motor_cfg_path:
            try:
                downloader = SFTPDownloader.from_file(str(sftp_cfg_path))
                with tempfile.TemporaryDirectory() as td:
                    local_tmp = Path(td) / "motor_config.toml"
                    downloader.download_file(remote_motor_cfg_path, str(local_tmp))
                    motor_cfg_bytes = local_tmp.read_bytes()
                    data = tomllib.loads(motor_cfg_bytes.decode("utf-8"))
            except Exception as e:
                self.ui.notify(
                    f"Failed to retrieve remote motor_config.toml: {e}."
                    "Use local motor_config.toml.",
                    color="orange"
                )

        if data is None:
            if not local_cfg_path.exists():
                raise RuntimeError(
                    "No motor_config.toml available (remote download failed and"
                    " local file does not exist)."
                )
            motor_cfg_bytes = local_cfg_path.read_bytes()
            data = tomllib.loads(motor_cfg_bytes.decode("utf-8"))

        self._motor_config_sha256 = hashlib.sha256(motor_cfg_bytes).hexdigest()

        x_motor_data = data["x_motor"]
        y_motor_data = data["y_motor"]

        def _microstep_factor(m):
            try:
                return int(m["microstepping"]["resolution"].split("/")[-1])
            except Exception:
                return 1

        x_full_steps_per_rev = x_motor_data["microstepping"]["full_steps_per_rev"]
        y_full_steps_per_rev = y_motor_data["microstepping"]["full_steps_per_rev"]
        x_microstep_factor = _microstep_factor(x_motor_data)
        y_microstep_factor = _microstep_factor(y_motor_data)
        x_pitch = x_motor_data["pitch"]
        y_pitch = y_motor_data["pitch"]
        rdir_map = {
            "clockwise": RotationDirection.CW,
            "counterclockwise": RotationDirection.CCW
        }
        x_rdir_ref = rdir_map.get(
            x_motor_data.get("rdir_ref", "counterclockwise").lower(),
            RotationDirection.CCW
        )
        y_rdir_ref = rdir_map.get(
            y_motor_data.get("rdir_ref", "counterclockwise").lower(),
            RotationDirection.CCW
        )
        return {
            "full_steps_per_rev": (x_full_steps_per_rev, y_full_steps_per_rev),
            "microstep_factor": (x_microstep_factor, y_microstep_factor),
            "pitch": (x_pitch, y_pitch),
            "rdir_ref": (x_rdir_ref, y_rdir_ref),
        }

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

    def _update_trajectory(self):
        if not self.points_mm:
            self.ui.notify("No points loaded.", color="red")
            return

        self.selected_profile = "velocity"

        alpha_max = (
            self.alpha_max_slider.value,
            self.alpha_max_slider.value
        )
        omega_max = (
            self.omega_max_slider.value,
            self.omega_max_slider.value
        )
        cont_motion = self.cont_motion_switch.value

        self.trajectory = PointToPointTrajectory(
            n_axes=2,
            profile_type=SCurvedProfile,
            pitch=self.motor_config["pitch"],
            rdir_ref=self.motor_config["rdir_ref"],
            alpha_max=alpha_max,
            omega_max=omega_max,
            full_steps_per_rev=self.motor_config["full_steps_per_rev"],
            microstep_factor=self.motor_config["microstep_factor"]
        )
        points = [(x / 1000, y / 1000) for x, y in self.points_mm]  # mm -> m
        self.trajectory(*points, continuous=cont_motion)
        try:
            self.pos_profile = self.trajectory.position_profiles
            self.vel_profile = self.trajectory.velocity_profiles
            self.acc_profile = self.trajectory.acceleration_profiles
        except ValueError:
            self.ui.notify(
                "Motion profile could not be calculated with "
                f"current maximum acceleration {alpha_max} and "
                f"current maximum speed {omega_max}",
                color="red"
            )
        else:
            self._show_trajectory()
            self._show_motion_profile()

    def _show_trajectory(self):
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
        if self.selected_profile is None:
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

    def _on_tab_change(self, event_args) -> None:
        self.selected_profile = event_args.value.lower()
        self._show_motion_profile()

    def _create_spinner_overlay(self) -> None:
        self.spinner_overlay = self.ui.dialog()
        with self.spinner_overlay.props("persistent"):
            with self.ui.card().classes("w-full items-center"):
                self.ui.spinner(size="lg")
                self.ui.label(
                    "Computing stepper motor signals. This "
                    "may take a while if the microstep factor is "
                    "large."
                ).classes("w-3/4")
        self.spinner_overlay.close()

    async def _save_to_json(self):
        if self.trajectory is None:
            self.ui.notify("No trajectory to save.", color="red")
            return

        self.spinner_overlay.open()
        await asyncio.sleep(0)

        base = self.filename_input.value.strip()
        if not base:
            base = f"trajectory_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"

        signals_path = SAVE_DIRECTORY / f"{base}.signals.json"
        meta_path = SAVE_DIRECTORY / f"{base}.meta.json"

        try:
            # Compute signals
            signals = await asyncio.to_thread(self.trajectory.get_stepper_signals)

            # Save signals and metadata locally (backup)
            self._save_signals(signals_path, signals)
            self._save_metadata(meta_path)

            # Upload to RPI
            uploader = SFTPUploader.from_file("sftp.toml")
            remote_paths = await asyncio.to_thread(
                uploader.upload_files,
                [str(signals_path), str(meta_path)]
            )
        except Exception as e:
            self.ui.notify(f"Error: {e}", color="red")
        else:
            ui.notify(
                f"Saved locally and uploaded to Pi:\n"
                + "\n".join(remote_paths),
                color='green'
            )
        finally:
            self.spinner_overlay.close()

    @staticmethod
    def _save_signals(path, signals):
        with open(path, "w") as f:
            json.dump(signals, f, indent=2)

    def _save_metadata(self, path):
        meta = {
            "profile_type": "SCurvedProfile",
            "alpha_max": [self.alpha_max_slider.value] * 2,
            "omega_max": [self.omega_max_slider.value] * 2,
            "continuous_motion": self.cont_motion_switch.value,
            "points": self.points_mm,
            "motor_config_sha256": self._motor_config_sha256
        }
        with open(path, "w") as f:
            json.dump(meta, f, indent=2)


if __name__ in {"__main__", "__mp_main__"}:

    from nicegui import app, ui

    trajectory_planner = TrajectoryPlanner(app, ui)
    trajectory_planner.run()
