from typing import Callable
import numpy as np
from scipy.integrate import solve_ivp


def velocity(
    t: float,
    a: Callable[[float, bool], float],
    t0: float = 0.0,
    v0: float = 0.0,
    abs_time: bool = True
) -> tuple[np.ndarray, np.ndarray]:
    """Calculates values of the velocity between time moments `t0` and `t`
    (where `t` > `t0`) when the acceleration `a` is given as a function of time
    (i.e. the function `a` should be called as `a(t)`). `v0` is the known
    velocity at time moment `t0`.

    Returns
    -------
    Tuple of two Numpy arrays. The first array contains the time values at which
    the velocity is determined. The second array contains the corresponding
    values of the velocity.
    """
    # noinspection PyUnusedLocal
    def fun(t: float, v: np.ndarray, abs_time: bool) -> np.ndarray:
        v_dot = np.zeros(1)
        v_dot[0] = a(t, abs_time)
        return v_dot

    sol = solve_ivp(
        fun, (t0, t), [v0], 
        method='LSODA', 
        t_eval=np.linspace(t0, t, endpoint=True), 
        args=(abs_time,)
    )
    # noinspection PyUnresolvedReferences
    return sol.t, sol.y[0]


def position(
    t: float,
    a: Callable[[float, bool], float],
    t0: float = 0.0,
    v0: float = 0.0,
    s0: float = 0.0,
    abs_time: bool = True
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculates values of position between time moments `t0` and `t`
    (where `t` > `t0`) when the acceleration `a` is given as a function of time
    (i.e. the function `a` should be called as `a(t)`). `v0` and `s0` are
    respectively the known velocity and the known position at time moment `t0`.

    Returns
    -------
    Tuple of three Numpy arrays. The first array contains the time values at
    which the position is determined. The second array contains the corresponding
    values of the position. In the third array, the corresponding values of the
    velocity are also returned.
    """
    def fun(t: float, s: np.ndarray, abs_time) -> np.ndarray:
        s_dot = np.zeros(2)
        s_dot[0] = s[1]
        s_dot[1] = a(t, abs_time)
        return s_dot

    sol = solve_ivp(
        fun, (t0, t), [s0, v0], 
        method='LSODA', 
        t_eval=np.linspace(t0, t, endpoint=True), 
        args=(abs_time,)
    )
    # noinspection PyUnresolvedReferences
    return sol.t, sol.y[0], sol.y[1]
