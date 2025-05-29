

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
