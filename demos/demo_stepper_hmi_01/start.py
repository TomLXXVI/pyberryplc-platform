from nicegui import app, ui
from hmi_app import StepperHMI

hmi = StepperHMI(app, ui)
hmi.run()
