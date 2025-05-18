import numpy as np
from .multi_axis import MotionProfile


def get_pitch(revs: int, distance: float) -> float:
    """Returns the pitch of a lead screw.

    Pitch is defined as the number of revolutions of the screw to travel the
    nut one meter.

    Parameters
    ----------
    revs : int
        Number of revolutions of the screw.
    distance : float
        Distance in meters travelled by the nut for the given number of 
        revolutions of the screw.
    """
    k = 1.0 / distance
    pitch = k * revs
    return pitch


def connect(*mps: MotionProfile):
    """Connects the motion profiles of segments in a trajectory.

    Returns
    -------
    pos_profile : tuple[float, float]
        Resulting position profile along the segments. First element of the
        tuple is the time array, the second element is the corresponding 
        position array.
    vel_profile : tuple[float, float]
        Resulting velocity profile along the segments. First element of the
        tuple is the time array, the second element is the corresponding 
        velocity array.
    acc_profile : tuple[float, float]
        Resulting acceleration profile along the segments. First element of 
        the tuple is the time array, the second element is the corresponding 
        acceleration array.
    """
    def _connect_vel(*mps: MotionProfile):
        """Connects the velocity profiles.
    
        Returns
        -------
        t_arr:
            Time array.
        v_arr:
            Corresponding velocity array.
        """
        t_arr, v_arr = _connect(*[mp.velocity_profile() for mp in mps])
        return t_arr, v_arr
    
    def _connect_pos(*mps: MotionProfile):
        """Connects the position profiles.
    
        Returns
        -------
        t_arr:
            Time array.
        s_arr:
            Corresponding position array.
        """
        t_arr, s_arr = _connect(*[mp.position_profile() for mp in mps])
        return t_arr, s_arr
    
    def _connect_acc(*mps: MotionProfile):
        """Connects the acceleration profiles.
    
        Returns
        -------
        t_arr:
            Time array.
        a_arr:
            Corresponding acceleration array.
        """
        t_arr, a_arr = _connect(*[mp.acceleration_profile() for mp in mps])
        return t_arr, a_arr
    
    def _connect(*profiles):
        """Connects the profiles (position, velocity, or acceleration) of the 
        segments in the trajectory.
    
        Returns
        -------
        t_arr:
            Time array.
        arr:
            Array with the corresponding profile values.
        """
        t_arr_lst, arr_lst = [], []
        for i, profile in enumerate(profiles):
            t_arr, arr = profile
            if i > 0:
                dt_shift = t_arr_lst[-1][-1]
                t_arr += dt_shift
            t_arr_lst.append(t_arr)
            arr_lst.append(arr)
        t_arr_concat = np.concatenate(tuple(t_arr_lst))
        arr_concat = np.concatenate(tuple(arr_lst))
        return t_arr_concat, arr_concat
    
    mps = tuple(mp for mp in mps if isinstance(mp, MotionProfile))
    pos_profile = _connect_pos(*mps)
    vel_profile = _connect_vel(*mps)
    acc_profile = _connect_acc(*mps)
    
    return pos_profile, vel_profile, acc_profile
