# Start plc server with: 
# uvicorn plc_server.main:app --host 0.0.0.0 --port 8000 --reload

import os
import asyncio
import logging
import random

from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi import WebSocket, WebSocketDisconnect

logger = logging.getLogger("uvicorn")

app = FastAPI()

# Shared motion profile settings
class MotionProfile(BaseModel):
    dt_acc: float
    dt_tot: float
    ds_tot: float

motion_profile = MotionProfile(dt_acc=0.5, dt_tot=2.0, ds_tot=100.0)
start_requested = False
motor_running = False
alarm_active = False

@app.get("/ping")
async def ping():
    """Simple endpoint to test if server is alive."""
    return {"message": "pong"}

@app.post("/set_profile")
async def set_profile(profile: MotionProfile):
    """Receive motion profile settings from HMI."""
    global motion_profile
    motion_profile = profile
    logger.info(f"Received new profile: {motion_profile}")
    return {"status": "profile updated"}

@app.post("/start_motion")
async def start_motion():
    """Trigger the PLC to start motion."""
    global start_requested, motor_running
    start_requested = True
    motor_running = True
    logger.info("Start motion requested")
    
    async def simulate_motion():
        global motor_running, alarm_active
        await asyncio.sleep(10)
        motor_running = False
        
        if random.random() < 0.5:
            alarm_active = True
            logger.warning("ALARM triggered: Overload!")
        else:
            alarm_active = False
            logger.info("Motion finished normally.")
    
    asyncio.create_task(simulate_motion())
    return {"status": "start requested"}

@app.get("/status")
async def status():
    """Provide full status back to HMI."""
    return {
        "motion_profile": motion_profile,
        "start_requested": start_requested,
        "motor_running": motor_running,
        "alarm_active": alarm_active
    }

# Serve static files
app.mount("/static", StaticFiles(directory="hmi_client/static"), name="static")

@app.get("/")
async def get_index():
    """Serve the index.html page."""
    return FileResponse(os.path.join("hmi_client", "templates", "index.html"))


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            status_message = {
                "motor_running": motor_running,
                "alarm_active": alarm_active
            }
            await websocket.send_json(status_message)
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        logger.info("WebSocket connection closed")
