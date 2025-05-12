from pyberryplc.motion.control import xy_motion_control, get_pitch
from plc_app import XYMotionPLC, SharedData


if __name__ == '__main__':

    mp_x, mp_y = xy_motion_control(
        p1=(0.0, 0.0), 
        p2=(0.02, 0.02),
        pitch=get_pitch(1, 0.01),
        omega=45.0,
        dt_acc=0.2,
        profile_type="trapezoidal"
    )
    
    print(f"Travel distance: X = {mp_x.ds_tot:.2f} deg, Y = {mp_y.ds_tot:.2f} deg")
    print(f"Travel time: X = {mp_x.dt_tot:.2f} s, Y = {mp_y.dt_tot:.2f} s")
    print(f"Speed: X = {mp_x.v_m:.3f} deg/s, Y = {mp_y.v_m:.3f} deg/s")
    
    shared_data = SharedData(
        hmi_buttons={"start_motion": True},
        hmi_data={
            "motion_profile_x": mp_x,
            "motion_profile_y": mp_y
        },
        hmi_digital_outputs={
            "motor_x_ready": False,
            "motor_x_busy": False,
            "motor_y_ready": False,
            "motor_y_busy": False,
        },
        hmi_analog_outputs={
            "travel_time_x": float('nan'),
            "travel_time_y": float('nan'),
        }
    )

    plc = XYMotionPLC(shared_data=shared_data)
    plc.run()
