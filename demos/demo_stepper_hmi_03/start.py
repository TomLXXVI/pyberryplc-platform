from nicegui import app, ui
from hmi_app import XYMotionHMI


hmi = XYMotionHMI(app, ui)
hmi.run()
