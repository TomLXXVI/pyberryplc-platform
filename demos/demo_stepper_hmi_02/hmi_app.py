import plotly.graph_objects as go

from pyberryplc.hmi import AbstractHMI
from pyberryplc.core import HMISharedData
from pyberryplc.utils.log_utils import init_logger
from pyberryplc.motion.single_axis import TrapezoidalProfile, SCurvedProfile

from plc_app import StepperMotorPLC


class StepperHMI(AbstractHMI):
    
    def __init__(self, app, ui):
        # Define the data that will be shared between PLC and HMI.
        shared_data = HMISharedData(
            buttons={"start_motion": False},
            data={"profile": None},
            digital_outputs={"motor_busy": False}
        )
        
        # GUI-widgets that need a reference.
        self.ds_tot_input: ui.number | None = None
        self.dt_tot_input: ui.number | None = None
        self.dt_acc_input: ui.number | None = None
        self.profile_type: ui.select | None = None
        self.plot: ui.plotly | None = None
        self.warning_label: ui.label | None = None
        
        # Initial motion profile.
        self.motion_profile = TrapezoidalProfile(
            ds_tot=720.0,
            dt_tot=2.0,
            dt_i=0.5
        )
        
        # Figure of initial motion profile.
        t_ax, v_ax = self.motion_profile.velocity_profile()
        self.fig = go.Figure(go.Scatter(x=t_ax, y=v_ax))
        self.fig.update_layout(
            template="ggplot2",
            margin=dict(t=40, b=40, l=50, r=10),
            xaxis_title="time, s",
            yaxis_title="velocity, °/s",
            title="Velocity Profile",
            height=400,  # pixels
            width=800
        )
        
        super().__init__(
            title="Stepper Motor Demo",
            app=app,
            ui=ui,
            shared_data=shared_data,
            plc_app=StepperMotorPLC,
            logger=init_logger(console=False),
            port=8081
        )
    
    def build_gui(self) -> None:
        self.ui.label("Stepper Motor Demo").classes("text-2xl font-bold mb-4")
        
        with self.ui.column().style("width: 800px"):
            with self.ui.row().classes("justify-between w-[800px]"):
                self.ds_tot_input = self.ui.number(
                    label="Travel distance [°]",
                    value=self.motion_profile.ds_tot,
                    on_change=self._update_plot
                )
                self.dt_tot_input = self.ui.number(
                    label="Travel time [s]",
                    value=self.motion_profile.dt_tot,
                    on_change=self._update_plot
                )
                self.dt_acc_input = self.ui.number(
                    label="Acceleration time [s]",
                    value=self.motion_profile.dt_i,
                    on_change=self._update_plot
                )
                self.profile_type = self.ui.select(
                    ["trapezoidal", "S-curve"],
                    label="Motion profile type",
                    value="trapezoidal",
                    on_change=self._update_plot
                ).classes("w-52")
    
            with self.ui.row().classes("justify-center"):
                self.plot = self.ui.plotly(self.fig).classes("w-[800px] h-[400px]")
        
        with self.ui.column():
            self.warning_label = self.ui.label().classes("text-red-600 mt-1")
         
        with self.ui.row():
            self.ui.button("Start motion", on_click=self._start_motion)
        
        self.ui.separator().classes("my-4")
    
    def _start_motion(self) -> None:
        # Send data to PLC
        self.shared_data.data["profile"] = self.motion_profile
        self.shared_data.buttons["start_motion"] = True
        
        # Info to the HMI operator            
        self.logger.info(f"Motion profile sent.")
    
    def _update_plot(self):
        try:
            # Create motion profile.
            ds_tot = float(self.ds_tot_input.value)
            dt_tot = float(self.dt_tot_input.value)
            dt_acc = float(self.dt_acc_input.value)
            
            # Check motion profile
            if dt_acc > dt_tot / 2:
                self.warning_label.text = "Warning: acceleration time is more than half the total travel time."
                title_color = "red"
            else:
                self.warning_label.text = ""
                title_color = "black"
            
            profile_params = {
                'ds_tot': ds_tot,
                'dt_tot': dt_tot,
                'dt_i': dt_acc
            }
            self.motion_profile = None
            match self.profile_type.value:
                case "trapezoidal":
                    self.motion_profile = TrapezoidalProfile(**profile_params)
                case "S-curve":
                    self.motion_profile = SCurvedProfile(**profile_params)
            
            # Plot motion profile.
            self._plot_motion_profile(title_color)
        
        except:
            self.warning_label.text = ""
    
    def _plot_motion_profile(self, title_color: str) -> None:
        t_ax, v_ax = self.motion_profile.velocity_profile()
        self.fig.data = []
        self.fig.add_trace(go.Scatter(x=t_ax, y=v_ax))
        self.fig.update_layout(
            title=dict(
                text="Velocity Profile",
                font=dict(color=title_color)
            )
        )
        self.plot.update()
    
    def update_status(self) -> None:
        motor_busy = self.shared_data.digital_outputs["motor_busy"]
        self.status_html.set_content("Motor running" if motor_busy else "Motor stopped")
