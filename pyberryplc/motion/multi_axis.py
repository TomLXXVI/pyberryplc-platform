from typing import Callable
from abc import ABC, abstractmethod
from enum import StrEnum

from numpy.typing import NDArray
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import root_scalar
from scipy.interpolate import interp1d


def velocity(
    t: float,
    accel_fn: Callable[[float, float, float], float],
    t0: float = 0.0,
    v0: float = 0.0,
    dv: float = 0.0
) -> tuple[NDArray[np.float64], ...]:
    """
    Calculates velocity values between time moments `t0` and `t` (`t` > `t0`).

    Parameters
    ----------
    t: float
        Final time moment in seconds.
    accel_fn:
        Function that returns the acceleration at any time moment between `t0`
        and `t`.
    t0: float
        Initial time moment the movement is observed in seconds.
    v0: float
        Velocity of the movement at the initial time moment.
    dv: float:
        Speed change during the movement. This helps to determine whether
        acceleration or deceleration is wanted.

    Returns
    -------
    t_arr:
        Numpy array with time values between `t0` and `t`.
    v_arr:
        Numpy array with the velocity values that correspond with the time
        values in `t_arr`.
    """
    # noinspection PyUnusedLocal
    def fn(t: float, v: NDArray[np.float64], dv: float, t0: float) -> NDArray[np.float64]:
        v_dot = np.zeros(1)
        v_dot[0] = accel_fn(t, dv, t0)
        return v_dot

    sol = solve_ivp(
        fn, (t0, t), [v0], 
        method='LSODA', 
        t_eval=np.linspace(t0, t, 100, endpoint=True),
        args=(dv, t0)
    )
    # noinspection PyUnresolvedReferences
    return sol.t, sol.y[0]


def position(
    t: float,
    accel_fn: Callable[[float, float, float], float],
    t0: float = 0.0,
    v0: float = 0.0,
    s0: float = 0.0,
    dv: float = 0.0
) -> tuple[NDArray[np.floating], ...]:
    """
    Calculates position values between time moments `t0` and `t` (`t` > `t0`).

    Parameters
    ----------
    t: float
        Final time moment in seconds.
    accel_fn:
        Function that returns the acceleration at any time moment between `t0`
        and `t`.
    t0: float
        Initial time moment the movement is observed in seconds.
    v0: float
        Velocity of the movement at the initial time moment.
    s0: float
        Position at the intial time moment.
    dv: float:
        Speed change during the movement. This helps to determine whether
        acceleration or deceleration is wanted.

    Returns
    -------
    t_arr:
        Numpy array with time values between `t0` and `t`.
    s_arr:
        Numpy array with the position values that correspond with the time
        values in `t_arr`.
    v_arr:
        Numpy array with the velocity values that correspond with the time
        values in `t_arr`.
    """

    def fn(t: float, s: NDArray[np.float64], dv: float, t0: float) -> NDArray[np.float64]:
        s_dot = np.zeros(2)
        s_dot[0] = s[1]
        s_dot[1] = accel_fn(t, dv, t0)
        return s_dot

    sol = solve_ivp(
        fn, (t0, t), [s0, v0], 
        method='LSODA', 
        t_eval=np.linspace(t0, t, endpoint=True),
        args=(dv, t0)
    )
    # noinspection PyUnresolvedReferences
    return sol.t, sol.y[0], sol.y[1]


class MotionProfile(ABC):
    
    def __init__(
        self, 
        ds_tot: float,
        a_m: float, 
        v_m: float,
        v_i: float = 0.0,
        s_i: float = 0.0,
        v_f: float | None = None,
        dt_tot: float | None = None
    ) -> None:
        """
        Creates a `MotionProfile` object.

        Parameters
        ----------
        ds_tot : float
            Total travel distance.
        v_m : float
            Maximum (allowable) velocity of the motor.
        a_m : float
            Maximum (allowable) acceleration of the motor.
        dt_tot : float, optional
            Total travel time (if known already).
        v_i : float, optional
            Initial velocity or start velocity. Default is 0.0.
        v_f : float, optional
            Final velocity, i.e. at the end of the travel distance. Default is
            0.0. If `None` the final velocity will be set equal to the top
            velocity of the motion profile.
        s_i : float, optional
            Initial position or start position. Default is 0.0.

        A motion profile can be instantiated in two different ways:

        1.  The travel distance `ds_tot` of the movement, the acceleration `a_m`
            of the motor, the initial velocity `v_i` and final velocity `v_f` of
            the movement are known.
            The top velocity `v_top` will initially be taken as the maximum
            velocity `v_m` of the motor. The total travel time `dt_tot` is then
            still to be determined.

        2.  The total travel distance `ds_tot` and total travel time `dt_tot` of
            the movement, the acceleration `a_m` of the motor, the initial
            velocity `v_ini` and final velocity `v_fin` of the movement are
            known. The top velocity `v_top` of the movement then still needs to
            be determined, but cannot exceed the maximum velocity `v_m` of the
            motor.
        """
        self.ds_tot = ds_tot
        self.a_m = a_m
        self.v_m = v_m
        self.v_i = v_i
        self._v_f = v_f
        self.s_i = s_i
        self._dt_tot = dt_tot
        self._v_top = self._get_top_velocity(self.v_m)
        self._dt_cov: float | None = None

    @property
    def dv_i(self) -> float:
        """
        Returns the required speed change in the initial acceleration phase
        of the motion profile.
        """
        if self.ds_tot > 0.0:
            return self._v_top - self.v_i
        return 0.0

    @property
    def dv_f(self) -> float:
        """
        Returns the required speed change in the final acceleration phase of the
        motion profile.
        """
        if self.ds_tot > 0.0 and self._v_f is not None:
            return self._v_f - self._v_top
        return 0.0
    
    @property
    def ds_i(self) -> float:
        """
        Returns the acceleration distance in the initial acceleration phase of
        the motion profile.
        """
        if self.ds_tot > 0.0 and self.dt_i > 0.0:
            t_arr, s_arr, _ = position(
                self.dt_i, self.accel_fn,
                t0=0.0,
                v0=self.v_i,
                s0=0.0,
                dv=self.dv_i
            )
            return float(s_arr[-1])
        return 0.0
    
    @property
    def ds_f(self) -> float:
        """
        Returns the acceleration distance in the final acceleration phase of
        the motion profile.
        """
        if self.ds_tot > 0.0 and self._v_f is not None and self.dt_f > 0.0:
            t_arr, s_arr, _ = position(
                self.dt_f, self.accel_fn,
                t0=0.0,
                v0=self._v_top,
                s0=0.0,
                dv=self.dv_f
            )
            return float(s_arr[-1])
        return 0.0
    
    @property
    def ds_cov(self) -> float:
        """
        Returns the travel distance at constant top velocity (acceleration is
        zero).
        """
        if self.ds_tot > 0.0:
            return self.ds_tot - (self.ds_i + self.ds_f)
        return 0.0

    def _get_top_velocity(self, v: float) -> float:
        """
        Returns the top velocity for the movement.
        """
        if self.ds_tot > 0.0:
            self._v_top = v
            if self.ds_cov < 0.0:
                self._v_top = self._search_top_velocity()
            return self._v_top
        return 0.0
    
    def _search_top_velocity(self) -> float:
        """
        Searches for the top velocity for which `ds_cov` equals zero (i.e.
        movement without constant-velocity phase).
        """

        def fn(v: float) -> float:
            self._v_top = v
            return self.ds_cov
        
        try:
            sol = root_scalar(fn, bracket=[1.e-6, self.v_m])
        except ValueError:
            raise ValueError("Failed to find a suitable top velocity") from None
        return sol.root

    @property
    def v_top(self) -> float:
        return self._v_top

    @v_top.setter
    def v_top(self, v: float) -> None:
        """
        Sets the top velocity for the movement.
        """
        self._v_top = v

    @property
    def v_f(self) -> float:
        """
        Returns the final velocity at the end of the movement.
        """
        if self._v_f is None:
            return self.v_top
        return self._v_f

    @property
    def dt_cov(self) -> float | None:
        """
        Returns the travel time at constant top velocity.
        """
        if self.ds_tot > 0.0:
            return round(self.ds_cov / self.v_top, 6)
        elif self._dt_cov is not None:
            return self._dt_cov
        return 0.0

    @property
    def dt_tot(self) -> float:
        """
        Returns the total travel time of the movement.
        """
        if self._dt_tot is not None:
            return self._dt_tot
        if self.ds_tot > 0.0:
            return self.dt_i + self.dt_cov + self.dt_f
        return 0.0

    @dt_tot.setter
    def dt_tot(self, v: float) -> None:
        """Sets the total travel time. Only to be used with multi-axis motion
        when the current axis is not moving.
        """
        self._dt_tot = v
        self._dt_cov = v

    @property
    @abstractmethod
    def dt_i(self) -> float:
        """
        Returns the initial acceleration time, i.e. the time duration of the
        initial acceleration phase.
        """
        ...

    @property
    @abstractmethod
    def dt_f(self) -> float:
        """
        Returns the final acceleration time, i.e. the time duration of the
        final acceleration phase.
        """
        ...
    
    @abstractmethod
    def accel_fn(self, t: float, dv: float, t0: float = 0.0) -> float:
        """
        Returns the acceleration at time moment `t`.

        Parameters
        ----------
        t: float
            Time moment for which the acceleration needs to be determined.
        t0: float
            Initial time moment, i.e. the time moment the acceleration phase
            starts.
        dv: float
            Required speed change. This helps to determine whether acceleration
            or deceleration is needed.
        """
        ...

    def velocity_profile(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the velocity profile of the movement.

        Returns
        -------
        t_arr :
            Numpy array with values on the time-axis.
        v_arr:
            Numpy array with the corresponding values on the velocity-axis.
        """
        # initial acceleration phase
        t0, v0 = 0.0, self.v_i
        t1 = t0 + self.dt_i
        if self.dt_i > 0.0:
            t1_arr, v1_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_i)
        else:
            t1_arr, v1_arr = np.array([t0]), np.array([v0])

        # constant-velocity phase
        t1, v1 = float(t1_arr[-1]), float(v1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            # noinspection PyTypeChecker
            t2_arr, v2_arr = velocity(t2, lambda *_: 0.0, t1, v1)
            t2, v2 = float(t2_arr[-1]), float(v2_arr[-1])
        else:
            t2_arr, v2_arr = None, None
            t2, v2 = float(t1_arr[-1]), float(v1_arr[-1])

        # final acceleration phase
        if self.dt_f > 0:
            t3 = t2 + self.dt_f
            t3_arr, v3_arr = velocity(t3, self.accel_fn, t2, v2, self.dv_f)
        else:
            t3_arr, v3_arr = None, None

        if t2_arr is None and t3_arr is not None:
            t_arr = np.concatenate((t1_arr[:-1], t3_arr))
            v_arr = np.concatenate((v1_arr[:-1], v3_arr))
        elif t2_arr is not None and t3_arr is None:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr))
            v_arr = np.concatenate((v1_arr[:-1], v2_arr))
        elif t2_arr is None and t3_arr is None:
            t_arr = t1_arr
            v_arr = v1_arr
        else:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr[:-1], t3_arr))
            v_arr = np.concatenate((v1_arr[:-1], v2_arr[:-1], v3_arr))

        return t_arr, v_arr

    def position_profile(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the position profile of the movement.

        Returns
        -------
        t_arr :
            Numpy array with values on the time-axis.
        s_arr:
            Numpy array with the corresponding values on the position-axis.
        """
        # initial acceleration phase
        t0, v0, s0 = 0.0, self.v_i, self.s_i
        t1 = t0 + self.dt_i
        if self.dt_i > 0.0:
            t1_arr, s1_arr, v1_arr = position(t1, self.accel_fn, t0, v0, s0, self.dv_i)
        else:
            t1_arr, s1_arr, v1_arr = np.array([t0]), np.array([s0]), np.array([v0])

        # constant-velocity phase
        t1, v1, s1 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            # noinspection PyTypeChecker
            t2_arr, s2_arr, v2_arr = position(t2, lambda *_: 0.0, t1, v1, s1)
            t2, v2, s2 = float(t2_arr[-1]), float(v2_arr[-1]), float(s2_arr[-1])
        else:
            t2_arr, s2_arr, v2_arr = None, None, None
            t2, v2, s2 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])

        # final acceleration phase
        if self.dt_f > 0.0:
            t3 = t2 + self.dt_f
            t3_arr, s3_arr, _ = position(t3, self.accel_fn, t2, v2, s2, self.dv_f)
        else:
            t3_arr, s3_arr = None, None

        if t2_arr is None and t3_arr is not None:
            t_arr = np.concatenate((t1_arr[:-1], t3_arr))
            s_arr = np.concatenate((s1_arr[:-1], s3_arr))
        elif t2_arr is not None and t3_arr is None:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr))
            s_arr = np.concatenate((s1_arr[:-1], s2_arr))
        elif t2_arr is None and t3_arr is None:
            t_arr = t1_arr
            s_arr = s1_arr
        else:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr[:-1], t3_arr))
            s_arr = np.concatenate((s1_arr[:-1], s2_arr[:-1], s3_arr))

        return t_arr, s_arr
    
    def acceleration_profile(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the acceleration profile of the movement.

        Returns
        -------
        t_arr :
            Numpy array with values on the time-axis.
        a_arr:
            Numpy array with the corresponding values on the acceleration-axis.
        """
        # initial acceleration phase
        t0, a0 = 0.0, 0.0
        t1 = t0 + self.dt_i
        t1_arr = np.linspace(t0, t1, endpoint=True)
        a1_arr = np.array([self.accel_fn(t, self.dv_i, t0) for t in t1_arr])

        # constant-velocity phase
        t1, a1 = float(t1_arr[-1]), float(a1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr = np.linspace(t1, t2, endpoint=True)
            a2_arr = np.array([0.0 for _ in t2_arr])
            t2, a2 = float(t2_arr[-1]), float(a2_arr[-1])
        else:
            t2_arr = None
            a2_arr = None
            t2, a2 = float(t1_arr[-1]), float(a1_arr[-1])

        # final acceleration phase
        if self.dt_f > 0:
            t3 = t2 + self.dt_f
            t3_arr = np.linspace(t2, t3, endpoint=True)
            a3_arr = np.array([self.accel_fn(t, self.dv_f, t2) for t in t3_arr])
        else:
            t3_arr, a3_arr = None, None

        if t2_arr is None and t3_arr is not None:
            t_arr = np.concatenate((t1_arr[:-1], t3_arr))
            a_arr = np.concatenate((a1_arr[:-1], a3_arr))
        elif t2_arr is not None and t3_arr is None:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr))
            a_arr = np.concatenate((a1_arr[:-1], a2_arr))
        elif t2_arr is None and t3_arr is None:
            t_arr = t1_arr
            a_arr = a1_arr
        else:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr[:-1], t3_arr))
            a_arr = np.concatenate((a1_arr[:-1], a2_arr[:-1], a3_arr))

        return t_arr, a_arr

    def get_velocity_time_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a time moment `t` and returns the
        velocity `v` at that time moment (`0 <= t <= dt_tot`).
        """
        t_ax, v_ax = self.velocity_profile()
        interp = interp1d(t_ax, v_ax)

        def f(t: float) -> float:
            try:
                v = interp(t)
            except ValueError:
                return 0.0
            return v

        return f

    def get_position_time_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a time moment `t` and returns the
        position `s` at that time moment in the movement (`0 <= t <= dt_tot`).
        """
        t_ax, s_ax = self.position_profile()
        interp = interp1d(t_ax, s_ax)

        def f(t: float) -> float:
            try:
                s = interp(t)
            except ValueError:
                s = s_ax[0]
                if t > t_ax[-1]:
                    s = s_ax[-1]
            return s

        return f

    def get_time_position_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a position `s` and returns the time
        moment `t` this position is reached in the movement (`0 <= s <= ds_tot`).
        """
        t_ax, s_ax = self.position_profile()
        interp = interp1d(s_ax, t_ax)

        def f(s: float) -> float:
            try:
                t = interp(s)
            except ValueError:
                t = t_ax[0]
                if s > s_ax[-1]:
                    t = t_ax[-1]
            return t

        return f

    def get_ini_velocity_time_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a time moment `t` in seconds and
        returns the velocity `v` at that moment during the initial acceleration
        phase of the movement (`0 <= t <= dt_i`).

        If `t > dt_i`, the velocity at `dt_i` is returned (i.e. the top velocity
        during the constant-velocity phase).
        """
        t0, v0 = 0.0, 0.0
        t1 = t0 + self.dt_i
        t_arr, v_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_i)
        v_max = v_arr[-1]
        interp = interp1d(t_arr, v_arr)

        def f(t: float) -> float:
            if t0 <= t <= t1:
                v = interp(t)
            elif t < t0:
                v = 0.0
            else:
                v = v_max
            return v

        return f

    def get_fin_velocity_time_fn(
        self,
        t0: float,
        v0: float
    ) -> Callable[[float], float]:
        """
        Returns a function that takes a time moment `t` in seconds and returns
        the velocity `v` at that moment during the final acceleration phase of
        the movement (`t0 <= t <= t0 + dt_f`).

        If `t > t0 + dt_f`, a velocity of zero is returned.
        If `t < t0`, the initial velocity `v0` is returned.

        Parameters
        ----------
        t0 : float
            Time moment the final acceleration phase begins.
        v0: float
            Initial velocity at the start of the final acceleration phase.
        """
        t1 = t0 + self.dt_f
        t_arr, v_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_f)
        interp = interp1d(t_arr, v_arr)

        def f(t: float) -> float:
            if t0 <= t <= t1:
                v = interp(t)
                if v < 0.0: v = 0.0
            elif t > t1:
                v = 0.0
            else:
                v = v0
            return v

        return f

    def get_ini_time_position_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a position `s` during the initial
        acceleration phase and returns the time moment `t` in seconds when this
        position is reached.

        If `s` is greater than the final acceleration position, the time moment
        is returned when the final acceleration position is reached.
        """
        t0, v0, s0 = 0.0, 0.0, 0.0
        t1 = t0 + self.dt_i
        t_arr, s_arr, v_arr = position(t1, self.accel_fn, t0, v0, s0, self.dv_i)
        t_min = t_arr[0]
        t_max = t_arr[-1]
        s_max = s_arr[-1]
        interp_s = interp1d(s_arr, t_arr)

        def f(s: float) -> float:
            if s0 <= s <= s_max:
                t = interp_s(s)
            elif s > s_max:
                t = t_max
            else:
                t = t_min
            return t

        return f

    def get_fin_time_position_fn(
        self,
        t0: float,
        s0: float,
        v0: float
    ) -> Callable[[float], float]:
        """
        Returns a function that takes a position `s` during the final
        acceleration phase and returns the time moment `t` in seconds when this
        position is reached.

        If `s` is greater than the final acceleration position, the time moment
        is returned when the final acceleration position is reached.
        If `s < s0`, the time moment `t0` is returned.

        Parameters
        ----------
        t0 : float
            Time moment the final acceleration phase begins.
        s0 : float
            Initial position at the start of the final acceleration phase.
        v0: float
            Initial velocity at the start of the final acceleration phase.
        """
        t1 = t0 + self.dt_f
        t_arr, v_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_f)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions.
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = interp1d(t_arr, v_arr)
            sol = root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self.accel_fn, t0, v0, s0)
        t_min = t_arr[0]
        t_max = t_arr[-1]
        s_max = s_arr[-1]

        interp_s = interp1d(s_arr, t_arr)

        def f(s: float) -> float:
            if s0 <= s <= s_max:
                t = interp_s(s)
            elif s > s_max:
                t = t_max
            else:
                t = t_min
            return t

        return f


class TrapezoidalProfile(MotionProfile):
    
    @property
    def dt_i(self) -> float:
        return abs(self.dv_i) / self.a_m
    
    @property
    def dt_f(self) -> float:
        return abs(self.dv_f) / self.a_m
    
    def accel_fn(self, t: float, dv: float, t0: float = 0.0) -> float:
        if dv < 0.0:
            return -self.a_m
        elif dv > 0.0:
            return self.a_m
        else:
            return 0.0
 

class SCurvedProfile(MotionProfile):
    
    @property
    def dt_i(self) -> float:
        return round(2 * abs(self.dv_i) / self.a_m, 6)
    
    @property
    def dt_f(self) -> float:
        return round(2 * abs(self.dv_f) / self.a_m, 6)

    def accel_fn(self, t: float, dv: float, t0: float = 0.0) -> float:
        if dv != 0.0:
            k = self.a_m ** 2 / abs(dv)
            if dv < 0.0: k *= -1
            dt = 2 * abs(dv) / self.a_m
            t1 = t0 + dt / 2
            t2 = t0 + dt
            if t0 <= t < t1:
                return k * (t - t0)
            elif t1 <= t <= t2:
                return k * (t2 - t)
            else:
                return 0.0
        return 0.0


class RotationDirection(StrEnum):
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    def to_bool(self) -> bool:
        """Returns True if direction is counterclockwise, False otherwise."""
        return self == RotationDirection.COUNTERCLOCKWISE

    def to_int(self) -> int:
        """Returns 1 for counterclockwise, -1 for clockwise."""
        if self == RotationDirection.CLOCKWISE:
            return -1
        return 1

    def __int__(self) -> int:
        return self.to_int()

    def __bool__(self) -> bool:
        raise TypeError("Use .to_bool() for explicit conversion.")


class MotionProfileType(StrEnum):
    TRAPEZOIDAL = "trapezoidal"
    S_CURVED = "S-curved"

    @classmethod
    def get_types(cls) -> list[str]:
        return [cls.TRAPEZOIDAL, cls.S_CURVED]
