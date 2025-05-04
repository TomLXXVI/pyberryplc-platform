from nicegui import ui
import threading
from pyberryplc.core.shared_data import SharedData
from pyberryplc.utils.log_utils import init_logger
from plc_app import StepperMotorPLC
from hmi_app import create_ui

# Get logger
logger = init_logger(log_file="logs/plc.log", console=False)

# Define shared data for data exchange between PLC and HMI
shared = SharedData(
    hmi_buttons={
        "start_motor_left": False,
        "start_motor_right": False,
        "stop_motor": False
    },
    hmi_outputs={
        "motor_running": False,
    }
)

# Create HMI GUI
create_ui(shared)


def start_plc_thread(shared, logger):
    def _plc_thread_fn():
        try:
            plc = StepperMotorPLC(shared_data=shared, logger=logger)
            plc.run()
        except Exception as e:
            logger.exception("PLC thread crashed.")
            shared.hmi_outputs["plc_fault"] = True
            if hasattr(shared, "dialog"):
                shared.dialog.open()

    thread = threading.Thread(target=_plc_thread_fn, daemon=True)
    thread.start()
    return thread


def monitor_restart():
    logger.warning("Restart requested from HMI")
    shared.hmi_buttons["restart_plc"] = False
    shared.hmi_outputs["plc_fault"] = False  # clear fault before retry
    shared.hmi_outputs["plc_restarted"] = False

    thread = start_plc_thread(shared, logger)
    
    def _check_thread():
        if thread.is_alive():
            logger.info("PLC thread running.")
            shared.hmi_outputs["plc_restarted"] = True
            if hasattr(shared, "dialog"):
                shared.dialog.close()
            threading.Timer(3.0, lambda: shared.hmi_outputs.update({"plc_restarted": False})).start()
        else:
            logger.error("PLC thread terminated during startup.")
            shared.hmi_outputs["plc_fault"] = True
            if hasattr(shared, "dialog"):
                shared.dialog.open()

    ui.timer(1.0, _check_thread, once=True)

# Start initial PLC thread via monitor
start_plc_thread(shared, logger)

# Make monitor available to HMI
shared.restart_callback = monitor_restart

ui.run(title="Stepper Motor HMI", port=8081, reload=False)
