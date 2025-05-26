import numpy as np
from .multi_axis import Direction
from .trajectory import Segment


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


def connect(*segments: Segment) -> tuple[dict, ...]:
    """Connects the segments in a trajectory.

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
    def _connect_vel(*segments: Segment):
        """
        Connects the velocity profiles.
        """
        vel_x_lst, vel_y_lst = zip(*[segment.velocity_profiles for segment in segments])
        vel_x_connected = _connect(*vel_x_lst)
        vel_y_connected = _connect(*vel_y_lst)
        return vel_x_connected, vel_y_connected
    
    def _connect_pos(*segments: Segment):
        """
        Connects the position profiles.
        """
        pos_x_lst, pos_y_lst = zip(*[segment.position_profiles for segment in segments])
        pos_x_connected = _connect(*pos_x_lst)
        pos_y_connected = _connect(*pos_y_lst)
        return pos_x_connected, pos_y_connected
    
    def _connect_acc(*segments: Segment):
        """
        Connects the acceleration profiles.
        """
        acc_x_lst, acc_y_lst = zip(*[segment.acceleration_profiles for segment in segments])
        acc_x_connected = _connect(*acc_x_lst)
        acc_y_connected = _connect(*acc_y_lst)
        return acc_x_connected, acc_y_connected
    
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
    
    pos_profile = _connect_pos(*segments)
    vel_profile = _connect_vel(*segments)
    acc_profile = _connect_acc(*segments)
        
    pos_profile = {
        "x": {
            "time": pos_profile[0][0],
            "values": pos_profile[0][1]
        },
        "y": {
            "time": pos_profile[1][0],
            "values": pos_profile[1][1]
        }
    }
    vel_profile = {
        "x": {
            "time": vel_profile[0][0],
            "values": vel_profile[0][1]
        },
        "y": {
            "time": vel_profile[1][0],
            "values": vel_profile[1][1]
        }
    }
    acc_profile = {
        "x": {
            "time": acc_profile[0][0],
            "values": acc_profile[0][1]
        },
        "y": {
            "time": acc_profile[1][0],
            "values": acc_profile[1][1]
        }
    }
    return pos_profile, vel_profile, acc_profile
