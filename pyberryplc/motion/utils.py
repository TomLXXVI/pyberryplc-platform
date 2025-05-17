import numpy as np


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


def connect(mp1, mp2):
    """Connects the motion profile `mp1` of the preceding segment with the
    motion profile `mp2` of the next segment.

    Returns
    -------
    pos_profile : tuple[float, float]
        Resulting position profile along the two segments. First element of the
        tuple is the time array, the second element is the corresponding 
        position array.
    vel_profile : tuple[float, float]
        Resulting velocity profile along the two segments. First element of the
        tuple is the time array, the second element is the corresponding 
        velocity array.
    acc_profile : tuple[float, float]
        Resulting acceleration profile along the two segments. First element of 
        the tuple is the time array, the second element is the corresponding 
        acceleration array.
    """
    def _connect_vel(mp1, mp2):
        """Connects the velocity profile of motion profile `mp1` of the preceding 
        segment with the velocity profile of motion profile `mp2` of the next 
        segment.
    
        Returns
        -------
        t_arr:
            Time array.
        v_arr:
            Corresponding velocity array.
        """
        profile1 = mp1.velocity_profile()
        profile2 = mp2.velocity_profile()
        t_arr, v_arr = _connect(profile1, profile2)
        return t_arr, v_arr
    
    def _connect_pos(mp1, mp2):
        """Connects the position profile of motion profile `mp1` of the preceding 
        segment with the position profile of motion profile `mp2` of the next 
        segment.
    
        Returns
        -------
        t_arr:
            Time array.
        s_arr:
            Corresponding position array.
        """
        profile1 = mp1.position_profile()
        profile2 = mp2.position_profile()
        t_arr, s_arr = _connect(profile1, profile2)
        return t_arr, s_arr
    
    def _connect_acc(mp1, mp2):
        """Connects the acceleration profile of motion profile `mp1` of the preceding 
        segment with the acceleration profile of motion profile `mp2` of the next 
        segment.
    
        Returns
        -------
        t_arr:
            Time array.
        a_arr:
            Corresponding acceleration array.
        """
        profile1 = mp1.acceleration_profile()
        profile2 = mp2.acceleration_profile()
        t_arr, a_arr = _connect(profile1, profile2)
        return t_arr, a_arr
    
    def _connect(profile1, profile2):
        """Connects the profile (position, velocity, or acceleration) of the 
        preceding profile 1 with the next profile 2.
    
        Returns
        -------
        t_arr:
            Time array.
        arr:
            Array with the corresponding profile values.
        """
        t_arr1, arr1 = profile1
        t_arr2, arr2 = profile2
        dt_shift = t_arr1[-1]
        t_arr2 += dt_shift
        t_arr = np.concatenate((t_arr1, t_arr2))
        arr = np.concatenate((arr1, arr2))
        return t_arr, arr

    pos_profile = _connect_pos(mp1, mp2)
    vel_profile = _connect_vel(mp1, mp2)
    acc_profile = _connect_acc(mp1, mp2)
    return pos_profile, vel_profile, acc_profile
