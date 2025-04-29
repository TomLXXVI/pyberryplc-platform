import threading
import time
import asyncio
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pyberryplc.core.plc import AbstractPLC
from pyberryplc.core.shared_data import SharedData


class PLCServer:
    """Framework class to encapsulate PLC threading and FastAPI server."""

    def __init__(self, shared_data: SharedData, plc_instance: AbstractPLC) -> None:
        self.shared_data = shared_data
        self.plc = plc_instance
        self.plc_thread: threading.Thread | None = None
        self.plc_monitor_thread: threading.Thread | None = None
        self.app = FastAPI()
        self.setup_routes()

    def setup_routes(self) -> None:
        """Configure FastAPI routes."""
        self.app.mount("/static", StaticFiles(directory="hmi_client/static"), name="static")

        @self.app.get("/")
        async def get_index():
            return FileResponse("hmi_client/templates/index.html")

        @self.app.post("/set_button")
        async def set_button(payload: dict):
            name = payload["name"]
            self.shared_data.hmi_buttons[name] = True
            return {"status": "Button registered"}

        @self.app.post("/set_switch")
        async def set_switch(payload: dict):
            name = payload["name"]
            value = payload["value"]
            self.shared_data.hmi_switches[name] = value
            return {"status": "Switch updated"}

        @self.app.post("/set_analog_input")
        async def set_analog_input(payload: dict):
            name = payload["name"]
            value = payload["value"]
            self.shared_data.hmi_analog_inputs[name] = value
            return {"status": "Analog input updated"}

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    await websocket.send_json({
                        "motor_running": self.shared_data.hmi_outputs.get("motor_running", False),
                        "alarm_active": self.shared_data.hmi_outputs.get("alarm_active", False),
                        "plc_fault": self.shared_data.hmi_outputs.get("plc_fault", False),
                    })
                    await asyncio.sleep(1)
            except WebSocketDisconnect:
                print("WebSocket disconnected")

    def start_plc(self) -> None:
        """Start PLC scan thread and monitoring thread."""
        self.plc_thread = threading.Thread(target=self.plc.run, daemon=True)
        self.plc_thread.start()

        self.plc_monitor_thread = threading.Thread(target=self.monitor_plc_thread, daemon=True)
        self.plc_monitor_thread.start()

    def monitor_plc_thread(self) -> None:
        """Monitor the PLC thread and raise fault if it dies."""
        while True:
            if not self.plc_thread or not self.plc_thread.is_alive():
                print("[PLC Monitor] PLC thread has crashed!")
                self.shared_data.hmi_outputs["plc_fault"] = True
                break
            time.sleep(2)

    def run_server(self) -> None:
        """Start the FastAPI server using uvicorn."""
        uvicorn.run(self.app, host="0.0.0.0", port=8000, reload=False)
