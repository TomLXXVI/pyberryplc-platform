from nicegui import ui
from pathlib import Path
from pyberryplc.core.shared_data import SharedData

shared_data: SharedData | None = None  # will be injected externally

def create_ui(shared: SharedData):
    global shared_data
    shared_data = shared

    ui.label("Stepper Motor HMI").classes("text-2xl font-bold")

    with ui.row():
        ui.button("Start motor left", on_click=lambda: shared_data.hmi_buttons.update({"start_motor_left": True}))
        ui.button("Start motor right", on_click=lambda: shared_data.hmi_buttons.update({"start_motor_right": True}))
        ui.button("Stop motor", on_click=lambda: shared_data.hmi_buttons.update({"stop_motor": True}))

    status_label = ui.label()
    fault_label = ui.label().classes("text-red-600 font-bold")
    restart_label = ui.label().classes("text-green-600 font-medium")

    log_display = ui.textarea(label='PLC log output').props('readonly').classes('w-full h-80')

    error_dialog = ui.dialog().classes("w-[400px]")
    with error_dialog:
        ui.label("⚠️ A critical PLC error has occurred. Please check the log for details.").classes("text-red-600 font-bold")
        with ui.row().classes("mt-4 justify-end"):
            ui.button("Dismiss", on_click=error_dialog.close)
            ui.button("Restart PLC", color="red", on_click=lambda: shared_data.restart_callback())

    # Expose dialog to the rest of the system
    shared.dialog = error_dialog

    def update_status():
        motor_on = shared_data.hmi_outputs.get("motor_running", False)
        status_label.text = f"Motor is {'running' if motor_on else 'stopped'}"

        if shared_data.hmi_outputs.get("plc_fault"):
            fault_label.text = "⚠️ PLC fault detected"
        else:
            fault_label.text = ""

        if shared_data.hmi_outputs.get("plc_restarted"):
            restart_label.text = "✅ PLC restarted successfully"
        else:
            restart_label.text = ""

    def update_logs():
        log_file = Path("logs/plc.log")
        if log_file.exists():
            lines = log_file.read_text(encoding="utf-8").splitlines()[-20:]
            log_display.value = "\n".join(lines)

    ui.timer(0.5, update_status)
    ui.timer(1.0, update_logs)
