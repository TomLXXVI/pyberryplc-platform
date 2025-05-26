import io
import csv
from functools import partial
import asyncio

import plotly.graph_objects as go
from nicegui.events import UploadEventArguments, ValueChangeEventArguments

from pyberryplc.hmi import AbstractHMI
from pyberryplc.core import SharedData
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.motion.motion_profile import ProfileType
from pyberryplc.motion.trajectory import TrajectoryPlanner, Trajectory
from pyberryplc.motion.utils import get_pitch, connect

from plc_app import XYMotionPLC


class XYMotionHMI(AbstractHMI):
    
    def __init__(self, app, ui):
        self.shared_data = SharedData(
            hmi_buttons={
                "start_motion": False,
            },
            hmi_digital_outputs={
                "x_motor_busy": False,
                "x_motor_ready": False,
                "y_motor_busy": False,
                "y_motor_ready": False,
            },
            hmi_analog_outputs={
                "travel_time_x": float('nan'),
                "travel_time_y": float('nan'),
            },
            hmi_data={
                "segments": None,
                "segment_count": 0
            }
        )

        self.points: list[tuple[float, float]] = []
        self.trajectory = Trajectory()
        self.trajectory_fig = go.Figure(go.Scatter(x=[0.0], y=[0.0]))
        self.trajectory_fig.update_layout(
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
        
        self.x_input = None
        self.y_input = None
        self.trajectory_plot = None
        self.profile_dialog = None
        
        super().__init__(
            title="XY Trajectory Demo",
            app=app,
            ui=ui,
            shared_data=self.shared_data,
            plc_app=XYMotionPLC,
            logger=init_logger(console=False),
            port=8081
        )
    
    def build_gui(self):
        self.profile_dialog = MotionProfileDialog(self)
        
        self.ui.label(self.title).classes("text-2xl font-bold mb-4")

        self.ui.label("Upload a .csv file with X,Y coordinates:")
        self.ui.upload(on_upload=self._handle_upload, auto_upload=True, max_files=1)
        
        self.ui.label("Planned XY Trajectory").classes("text-xl font-bold")
        with self.ui.row().classes("items-center"):
            self.ui.label("Y [mm]") \
                .classes("text-base mr-1") \
                .style("width: 2rem; text-align: center;")

            with self.ui.column().classes("items-center"):
                self.trajectory_plot = self.ui.plotly(self.trajectory_fig).classes("w-[500px] h-[500px]")
                self.ui.label("X [mm]").classes("text-base mt-1")
        
        self.ui.button("Verify Motion Profiles", on_click=partial(self.profile_dialog.open, self.points))
        self.ui.button("Send to PLC", on_click=self._send_to_plc)

    def _handle_upload(self, e: UploadEventArguments):
        try:
            self.points.clear()
            text_stream = io.TextIOWrapper(e.content, encoding="utf-8")
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
            self._display_trajectory()
            
        except Exception as ex:
            self.ui.notify(f"Error while reading file: {ex}", color="red")

    def _display_trajectory(self) -> None:
        if not self.points:
            self.ui.notify("No points loaded.", color="red")
            return

        self.trajectory_fig.data = []  # Clear previous plot
        x_vals, y_vals = zip(*self.points)
        self.trajectory_fig.add_trace(go.Scatter(
            x=x_vals,
            y=y_vals,
            mode="lines+markers",
            name="Trajectory"
        ))
        self.trajectory_plot.update()
        self.ui.notify("Trajectory plot updated.")

    def _send_to_plc(self) -> None:
        if not self.trajectory:
            self.ui.notify("No trajectory to send.", color="red")
            return
        self.shared_data.hmi_data["segments"] = self.trajectory
        self.shared_data.hmi_buttons["start_motion"] = True
        self.ui.notify("Trajectory sent to PLC.")

    def update_status(self) -> None:
        x_motor_busy = self.shared_data.hmi_digital_outputs["x_motor_busy"]
        x_motor_ready = self.shared_data.hmi_digital_outputs["x_motor_ready"]
        y_motor_busy = self.shared_data.hmi_digital_outputs["y_motor_busy"]
        y_motor_ready = self.shared_data.hmi_digital_outputs["y_motor_ready"]
        segment_count = self.shared_data.hmi_data["segment_count"]
        
        segment_status = ""
        if segment_count > 0:
            segment_status = f"<b>Segment {segment_count}</b>"
        
        x_status = "<b>X-axis</b>: "
        if x_motor_busy and not x_motor_ready:
            x_status += "running"
        elif x_motor_busy and x_motor_ready:
            x_status += "paused"
        else:
            x_status += "stopped"

        y_status = "<b>Y-axis</b>: "
        if y_motor_busy and not y_motor_ready:
            y_status += "running"
        elif y_motor_busy and y_motor_ready:
            y_status += "paused"
        else:
            y_status += "stopped"
        
        if segment_status:
            status = f"{segment_status} | {x_status} | {y_status}"
        else:
            status = f"{x_status} | {y_status}"
        self.status_html.set_content(status)


class MotionProfileDialog:
    
    def __init__(self, hmi: XYMotionHMI) -> None:
        self.hmi = hmi
        self.selected_profile = "velocity"
        self.pos = None
        self.vel = None
        self.acc = None
        self.points = []
        self.profile_fig = go.Figure()
        self.profile_fig.update_layout(
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
                    text="velocity, mm/s",
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
        self._create_overlay()
        self._create_dialog()
    
    def _create_dialog(self) -> None:
        # create motion profile dialog
        self._dialog = self.hmi.ui.dialog()
        with self._dialog:
            with self.hmi.ui.card().classes("w-full"):
                self.hmi.ui.label("Motion Profile Viewer").classes("text-lg font-bold")
                
                with self.hmi.ui.tabs(value="Velocity", on_change=self._on_tab_change).props("dense"):
                    self.hmi.ui.tab("Velocity")
                    self.hmi.ui.tab("Position")
                    self.hmi.ui.tab("Acceleration")

                self.plot = self.hmi.ui.plotly(self.profile_fig).classes("w-[500px]")
                self.hmi.ui.button("Close", on_click=self._dialog.close)
        self._dialog.close()
    
    def _create_overlay(self) -> None:
        # create dialog with spinner
        self._overlay = self.hmi.ui.dialog()
        with self._overlay.props("persistent"):
            with self.hmi.ui.card().classes("w-full items-center"):
                self.hmi.ui.spinner(size="lg")
                self.hmi.ui.label("Processing trajectory... Please wait.")
        self._overlay.close()

    def _on_tab_change(self, e: ValueChangeEventArguments) -> None:
        # select profile: velocity, position, or acceleration
        self.selected_profile = e.value.lower()
        self._update_profile_plot()
    
    def _update_profile_plot(self) -> None:
        # update figure with selected profile
        if not self.vel:
            self.hmi.ui.notify("No motion profile available.", color="red")
            return

        if self.selected_profile == "position":
            t_x = self.pos["x"]["time"]
            y_x = self.pos["x"]["values"] * 1000  # mm
            t_y = self.pos["y"]["time"]
            y_y = self.pos["y"]["values"] * 1000
            y_label = "position, mm"
        elif self.selected_profile == "acceleration":
            t_x = self.acc["x"]["time"]
            y_x = self.acc["x"]["values"] * 1000  # mm/s²
            t_y = self.acc["y"]["time"]
            y_y = self.acc["y"]["values"] * 1000
            y_label = "acceleration, mm/s²"
        else:
            t_x = self.vel["x"]["time"]
            y_x = self.vel["x"]["values"] * 1000 # mm/s
            t_y = self.vel["y"]["time"]
            y_y = self.vel["y"]["values"] * 1000
            y_label = "velocity, mm/s"

        self.profile_fig.data = []
        self.profile_fig.add_trace(go.Scatter(
            x=t_x,
            y=y_x,
            mode="lines",
            name="X-axis"
        ))
        self.profile_fig.add_trace(go.Scatter(
            x=t_y,
            y=y_y,
            mode="lines",
            name="Y-axis"
        ))
        self.profile_fig.update_layout(
            yaxis=dict(
                title=dict(
                    text=y_label,
                    font=dict(family="Helvetica, Arial, sans-serif", size=13)
                ),
                tickfont=dict(family="Helvetica, Arial, sans-serif", size=13),
            ),
        )
        self.plot.update()
    
    async def open(self, points: list[tuple[float, float]]):
        # first show the overlay with spinner during the calculation of motion 
        # profiles...
        self._overlay.open()
        try:
            await self._compute_trajectory(points)
        except Exception as ex:
            self.hmi.ui.notify(
                f"Failed to generate motion profiles: {ex}", 
                color="red"
            )
        self._overlay.close()
        # when calculations are done, open motion profile dialog... 
        self.selected_profile = "velocity"
        self._update_profile_plot()
        self._dialog.open()
        
    async def _compute_trajectory(self, points: list[tuple[float, float]]):
        
        await asyncio.sleep(1)  # only here to test
        
        trajectory_planner = TrajectoryPlanner(
            pitch=get_pitch(1, 0.01),
            motor_speed=180.0,
            motor_accel=360.0,
            full_steps_per_rev=200,  # must match with motor configuration file
            microstep_factor=2,      # must match with motor configuration file
            profile_type=ProfileType.S_CURVED,
        )
        points = [(x / 1000, y / 1000) for x, y in points]  # mm -> m
        self.hmi.trajectory = trajectory_planner.create_trajectory(*points)
        self.pos, self.vel, self.acc = self.hmi.trajectory.motion_profiles
