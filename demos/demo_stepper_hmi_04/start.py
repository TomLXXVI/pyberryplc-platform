from pyberryplc.utils.log_utils import init_logger
from pyberryplc.core import SharedData
from plc_app import XYMotionPLC


if __name__ == '__main__':
    p1 = (0.0, 0.0)
    p2 = (0.04, 0.02)
    p3 = (0.06, 0.04)
    p4 = (0.06, 0.06)
    p5 = (0.04, 0.06)
    
    logger = init_logger(name="XYMotionPLC", console=True)
    
    shared_data = SharedData(
        hmi_buttons={
            "start_motion": True,
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
            "segments": [(p1, p2), (p2, p3), (p3, p4), (p4, p5)],
        }
    )
    
    plc = XYMotionPLC(shared_data, logger)
    plc.run()
