from dataclasses import dataclass

from pyberryplc.motion import MotionProfile, TrapezoidalProfile, SCurvedProfile


@dataclass
class Position:
    x: float
    y: float


def get_pitch(revs: int, distance: float) -> float:
    """Returns the pitch of a lead screw.
    
    Pitch is defined as the number of revolutions of the screw to travel the
    nut one meter.
    
    Parameters
    ----------
    revs : int
        Number of revolutions of the screw that corresponds with the given 
        distance.
    distance : float
        Distance in meters travelled by the nut for the given number of 
        revolutions.
    """
    k = 1.0 / distance
    pitch = k * revs
    return pitch


def xy_motion_control(
    p1: Position, 
    p2: Position, 
    pitch: float, 
    omega: float,
    dt_acc: float,
    profile_type: str = "trapezoidal"
) -> tuple[MotionProfile, MotionProfile]:
    """Calculates the required motion profiles along the X- and Y-axis such that
    both axes finish their moves at the same time.
    
    Parameters
    ----------
    p1 : Position
        Start position with (x1, y1) coordinates in meters.
    p2 : Position
        End position with (x2, y2) coordinates in meters.
    pitch:
        Pitch of the lead screw (i.e. the number of revolutions of the screw to
        move the nut one meter).
    omega:
        Angular speed of the motor shaft in degrees per second.
    dt_acc:
        Acceleration time in seconds.
    profile_type : str, ["trapezoidal", "S-curve"]
        Type of motion profile: either a trapezoidal profile (i.e. the default)
        or an S-curved profile.
    
    Returns
    -------
    mp_x : MotionProfile
        Required motion profile along the x-axis.
    mp_y : MotionProfile
        Required motion profile along the y-axis.
    """
    # travel distance along the X-axis and Y-axis
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    # transmission ratio of lead screw (deg/m)
    N = 360.0 * pitch
    # angular travel of X- and Y-motor shaft (deg)
    dtheta_x = N * dx
    dtheta_y = N * dy
    # motion profile of X- and Y-motor.
    if profile_type == "trapezoidal":
        mp = TrapezoidalProfile
    else:
        mp = SCurvedProfile
    if dx >= dy:
        mp_x = mp(dt_acc=dt_acc, ds_tot=dtheta_x, v_m=omega)
        mp_y = mp(dt_acc=dt_acc, ds_tot=dtheta_y, dt_tot=mp_x.dt_tot)
    else:
        mp_y = mp(dt_acc=dt_acc, ds_tot=dtheta_y, v_m=omega)
        mp_x = mp(dt_acc=dt_acc, ds_tot=dtheta_x, dt_tot=mp_y.dt_tot)
    return mp_x, mp_y
    

def main():
    p1 = Position(0.0, 0.0)    
    p2 = Position(0.300, 0.100)  
    pitch = get_pitch(1, 0.01)
    omega = 720.0  # deg / s
    x_motion, y_motion = xy_motion_control(p1, p2, pitch, omega, 0.2, "trapezoidal")
    print(f"X-axis angular speed: {x_motion.v_m:.3f} deg/s")
    print(f"Y-axis angular speed: {y_motion.v_m:.3f} deg/s")
    print(f"X-axis total travel time: {x_motion.dt_tot:.3f} s")
    print(f"Y-axis total travel time: {y_motion.dt_tot:.3f} s")


if __name__ == '__main__':
    main()
    