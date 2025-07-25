# CUBIC TRAJECTORY - CONSTANT JERK

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
            Initial acceleration. Default value is 0.0.
        j0:
            Constant jerk. Default value is 0.0.

    Returns
    -------
    Single position at time moment `t` or array of positions corresponding with
    the values in time array `t`.
    """
    t0 = conds.get("t0", 0.0)
    s0 = conds.get("s0", 0.0)
    v0 = conds.get("v0", 0.0)
    a0 = conds.get("a0", 0.0)
    j0 = conds.get("j0", 0.0)

    s = s0 + v0 * (t - t0) + 1/2 * a0 * (t - t0)**2 + 1/6 * j0 * (t - t0)**3
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
            Initial acceleration. Default value is 0.0.
        j0:
            Constant jerk. Default value is 0.0.

    Returns
    -------
    Single velocity at time `t` or array of velocities corresponding with the
    values in time array `t`.
    """
    t0 = conds.get("t0", 0.0)
    v0 = conds.get("v0", 0.0)
    a0 = conds.get("a0", 0.0)
    j0 = conds.get("j0", 0.0)
    
    v = v0 + a0 * (t - t0) + 1/2 * j0 * (t - t0)**2
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
        t0:
            Initial time. Default value is 0.0.
        a0:
            Initial acceleration. Default value is 0.0.
        j0:
            Constant jerk. Default value is 0.0.

    Returns
    -------
    Single acceleration at time `t` or array of accelerations corresponding with
    the values in time array `t`.
    """
    t0 = conds.get("t0", 0.0)
    a0 = conds.get("a0", 0.0)
    j0 = conds.get("j0", 0.0)
    
    a = a0 + j0 * (t - t0)
    return a


def jerk(t: float | np.ndarray, **conds: float | Any) -> float | np.ndarray:
    """
    Jerk vs. time during acceleration/deceleration.

    Parameters
    ----------
    t:
        Time moment or array of time moments (> t0).
    **conds:
        Initial conditions:
        j0:
            Constant jerk. Default value is 0.0.

    Returns
    -------
    Constant jerk at any time moment `t` or array with equal length as time
    array wherein each value is equal to the constant jerk.
    """
    j0 = conds.get("j0", 0.0)
    
    if isinstance(t, np.ndarray):
        return np.full_like(t, j0)
    return j0


def get_constant_jerk(
    af: float,
    vf: float | None = None,
    tf: float | None = None,
    **conds: float | Any
) -> float:
    """
    Returns the constant jerk derived from the acceleration profile.

    Parameters
    ----------
    af:
        Final acceleration at the end of the acceleration phase.
    vf:
        Final velocity at the end of the acceleration phase.
    tf:
        Final time moment of the acceleration phase.
    **conds:
        Initial conditions:
        t0:
            Initial time. Default value is 0.0.
        a0:
            Initial acceleration. Default value is 0.0.

    Returns
    -------
    Constant jerk.
    """
    t0 = conds.get("t0", 0.0)
    v0 = conds.get("v0", 0.0)
    a0 = conds.get("a0", 0.0)

    if tf is not None:
        j0 = (af - a0) / (tf - t0)
        return j0
    elif vf is not None:
        j0 = (af - a0) ** 2 / (2 * (vf - v0))
        return j0
    else:
        raise ValueError("Either `tf` or `vf` must be specified.")


def solve_motion(
    t0: float = 0.0,
    t1: float = 0.0,
    s0: float = 0.0,
    s1: float = 0.0,
    v0: float = 0.0,
    v1: float = 0.0
) -> tuple[float | Any, float | Any]:
    """
    Solves for the constant jerk `j0` and initial acceleration `a0` when the
    following boundary conditions are given:
    
    Parameters
    ----------
    t0:
        Initial time of movement.
    t1:
        Final time of movement.
    s0:
        Initial position at `t0`.
    s1:
        Final position at `t1`.
    v0:
        Initial velocity at `t0`.
    v1:
        Final velocity at `t1`.
    """
    dt = t1 - t0
    ds = s1 - s0
    dv = v1 - v0
    A = np.array([
        [dt, 0.5 * dt**2],
        [0.5 * dt**2, 1/6 * dt**3]
    ])
    B = np.array([
        [dv],
        [ds - v0 * dt]
    ])
    X = np.linalg.solve(A, B)
    a0, j0 = X[0, 0], X[1, 0]
    return a0, j0


def get_motion_profile(
    t0: float,
    t1: float,
    ini_conds: tuple[float, float],
    fin_conds: tuple[float, float]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Calculates the position profile, velocity profile, and acceleration
    profile of the motion between time moments `t0` and `t1`.

    Parameters
    ----------
    t0:
        Initial moment of the observation of the motion.
    t1:
        Final moment of the observation of the motion.
    ini_conds:
        Initial position and velocity at `t0`.
    fin_conds:
        Final position and velocity at `t1`.

    Returns
    -------
    t_arr:
        Numpy array with time moments between `t0` and `t1` (included).
    s_arr:
        Numpy array with corresponding positions.
    v_arr:
        Numpy array with corresponding velocities.
    a_arr:
        Numpy array with corresponding accelerations.
    """
    t_arr = np.linspace(t0, t1)
    s0, v0 = ini_conds
    s1, v1 = fin_conds
    a0, j0 = solve_motion(t0, t1, s0, s1, v0, v1)
    s_arr = position(t_arr, t0=t0, s0=s0, v0=v0, a0=a0, j0=j0)
    v_arr = velocity(t_arr, t0=t0, v0=v0, a0=a0, j0=j0)
    a_arr = acceleration(t_arr, t0=t0, a0=a0, j0=j0)
    return t_arr, s_arr, v_arr, a_arr


if __name__ == '__main__':

    from pyberryplc.charts import LineChart

    def main():
        j0 = 2.0
        a0 = 0.0
        v0 = 0.0
        s0 = 0.0

        t_arr = np.linspace(0.0, 4.0)
        s_arr = position(t_arr, j0=j0, a0=a0, v0=v0, s0=s0)
        v_arr = velocity(t_arr, j0=j0, a0=a0, v0=v0)
        a_arr = acceleration(t_arr, j0=j0, a0=a0)
        j_arr = jerk(t_arr, j0=j0)

        schart = LineChart()
        schart.add_xy_data(
            label="s(t)",
            x1_values=t_arr,
            y1_values=s_arr
        )
        schart.x1.add_title("t")
        schart.y1.add_title("s")
        schart.show()

        vchart = LineChart()
        vchart.add_xy_data(
            label="v(t)",
            x1_values=t_arr,
            y1_values=v_arr
        )
        vchart.x1.add_title("t")
        vchart.y1.add_title("v")
        vchart.show()

        achart = LineChart()
        achart.add_xy_data(
            label="a(t)",
            x1_values=t_arr,
            y1_values=a_arr
        )
        achart.x1.add_title("t")
        achart.y1.add_title("a")
        achart.show()

        jchart = LineChart()
        jchart.add_xy_data(
            label="j(t)",
            x1_values=t_arr,
            y1_values=j_arr
        )
        jchart.x1.add_title("t")
        jchart.y1.add_title("j")
        jchart.show()
