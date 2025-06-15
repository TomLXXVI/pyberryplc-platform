from pyberryplc.hmi.base import AbstractHMI, HMISharedData
from pyberryplc.utils.log_utils import init_logger

from plc_app import StepperMotorPLC


class StepperHMI(AbstractHMI):
    
    def __init__(self, app, ui):
        # Define shared data that will be exchanged between PLC and HMI.
        shared = HMISharedData(
            buttons={
                "start_motor_left": False,
                "start_motor_right": False,
                "stop_motor": False
            },
            digital_outputs={
                "motor_running": False
            }
        )
        
        # Set up a logger for the PLC (will be shared with the HMI).
        logger = init_logger(log_file="logs/plc.log", console=False)
                
        # GUI-widgets (that need to have a reference):
        self.btn_left = None
        self.btn_right = None
        self.btn_stop = None
        
        # Instantiate the HMI app.
        super().__init__(
            title="Stepper Motor HMI",
            app=app,
            ui=ui,
            shared_data=shared, 
            plc_app=StepperMotorPLC,
            logger=logger,
            port=8081
        )
    
    def build_gui(self):
        """Build the GUI of this HMI app."""
        self.ui.label("Stepper Motor HMI").classes("text-2xl font-bold")
        
        with self.ui.row():
            self.btn_left = self.ui.button(
                "Start left", 
                on_click=lambda: self.shared_data.buttons.update({"start_motor_left": True})
            )
            self.btn_right = self.ui.button(
                "Start right", 
                on_click=lambda: self.shared_data.buttons.update({"start_motor_right": True})
            )
            self.btn_stop = self.ui.button(
                "Stop", 
                on_click=lambda: self.shared_data.buttons.update({"stop_motor": True})
            )
        
    def update_status(self):
        running = self.shared_data.digital_outputs.get("motor_running", False)
        self.status_html.set_content("Motor is running" if running else "Motor is stopped")

        self.set_button_state(self.btn_left, enabled=not running)
        self.set_button_state(self.btn_right, enabled=not running)
        self.set_button_state(self.btn_stop, enabled=running)
