from nicegui import app, ui
from hmi_app import XYMotionHMI


if __name__ == '__main__':
    
    hmi = XYMotionHMI(app, ui)
    hmi.run()
