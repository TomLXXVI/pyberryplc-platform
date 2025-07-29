# PARABOLIC TRAJECTORY - CONSTANT ACCELERATION

from typing import Any
import numpy as np


def position(t: float | np.ndarray, **conds: float | Any) -> float | np.ndarray:
    """
    Position vs. time during acceleration/deceleration.
    
    Parameters
    ----------
    t:
        Time moment or array of time moments (> t0).
    **conds:
        Initial conditions:
        t0:
            Initial time. Default value is 0.0.
        s0:
            Initial position at t = t0. Default value is 0.0.
        v0:
            Initial velocity at t = t0. Default value is 0.0.
        a0:
            Constant acceleration. Default value is 0.0.
    
    Returns
    -------
    Single position at time moment t or array of positions corresponding with
    the values in time array.
    """
    t0 = conds.get("t0", 0.0)
    s0 = conds.get("s0", 0.0)
    v0 = conds.get("v0", 0.0)
    a0 = conds.get("a0", 0.0)
    
    s = s0 + v0 * (t - t0) + 0.5 * a0 * (t - t0)**2
    return s


def velocity(t: float | np.ndarray, **conds: float | Any) -> float | np.ndarray:
    """
    Velocity vs. time during acceleration/deceleration.
    
    Parameters
    ----------
    t:
        Time moment or array of time moments (> t0).
    **conds:
        Initial conditions:
        t0:
            Initial time. Default value is 0.0.
        v0:
            Initial velocity at t = t0. Default value is 0.0.
        a0:
            Constant acceleration. Default value is 0.0.

    Returns
    -------
    Single velocity at time t or array of velocities corresponding with values
    in time array.
    """
    t0 = conds.get("t0", 0.0)
    v0 = conds.get("v0", 0.0)
    a0 = conds.get("a0", 0.0)
    
    v = v0 + a0 * (t - t0)
    return v


def acceleration(t: float | np.ndarray, **conds: float | Any) -> float | np.ndarray:
    """
    Acceleration vs. time during acceleration/deceleration
    
    Parameters
    ----------
    t:
        Time moment or array of time moments (> t0).
    **conds:
        Initial conditions:
        a0:
            Constant acceleration.

    Returns
    -------
    Constant acceleration at any time moment t or array with equal length as 
    time array wherein each value is equal to the constant acceleration.
    """
    a0 = conds.get("a0", 0.0)
    
    if isinstance(t, np.ndarray):
        return np.full_like(t, a0)
    return a0


def get_constant_acceleration(
    vf: float, 
    qf: float | None = None, 
    tf: float | None = None,
    **conds: float | Any
) -> float:
    """
    Returns the constant acceleration derived from the velocity profile.
    
    Parameters
    ----------
    vf:
        Final velocity at the end of the acceleration phase.
    qf:
        Final position at the end of the acceleration phase.
    tf:
        Final time moment of the acceleration phase.
    **conds:
        Initial conditions:
        t0:
            Initial time. Default value is 0.0.
        s0:
            Initial position at t = t0. Default value is 0.0.
        v0:
            Initial velocity at t = t0. Default value is 0.0.
            
    Returns
    -------
    Constant acceleration.
    """
    t0 = conds.get("t0", 0.0)
    s0 = conds.get("s0", 0.0)
    v0 = conds.get("v0", 0.0)
    
    if tf is not None:
        a0 = (vf - v0) / (tf - t0)
        return a0
    elif qf is not None:
        a0 = (vf - v0)**2 / (2 * (qf - s0))
        return a0
    else:
        raise ValueError("Either `tf` or `qf` must be specified.")


if __name__ == '__main__':
    
    from pyberryplc.charts import LineChart

    def main():
        a0 = 0.5
        t_arr = np.linspace(0.0, 4.0)
        s_arr = position(t_arr, a0=a0)
        v_arr = velocity(t_arr, a0=a0)

        qchart = LineChart()
        qchart.add_xy_data(
            label="s(t)",
            x1_values=t_arr,
            y1_values=s_arr
        )
        qchart.x1.add_title("t")
        qchart.y1.add_title("s")
        qchart.show()

        vchart = LineChart()
        vchart.add_xy_data(
            label="v(t)",
            x1_values=t_arr,
            y1_values=v_arr
        )
        vchart.x1.add_title("t")
        vchart.y1.add_title("v")
        vchart.show()

    main()
