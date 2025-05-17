"""
Definition of motion profiles for use in multi-axis motion.

Two types of profiles are implemented in this module:
- Class `TrapezoidalProfile` defines a trapezoidal motion profile.
- Class `SCurvedProfile` defines a pure S-curve motion profile.

References
----------
Gürocak, H. (2016), Industrial Motion Control, John Wiley & Sons.
"""
from typing import Callable
from abc import ABC, abstractmethod
from enum import StrEnum
import numpy as np
import scipy
from .kinematics import position, velocity


class Direction(StrEnum):
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"

    def to_bool(self) -> bool:
        """Returns True if direction is counterclockwise, False otherwise."""
        return self == Direction.COUNTERCLOCKWISE

    def to_int(self) -> int:
        """Returns 1 for counterclockwise, 0 for clockwise."""
        return int(self.to_bool())

    def __int__(self) -> int:
        return self.to_int()

    def __bool__(self) -> bool:
        raise TypeError("Use .to_bool() for explicit conversion.")


class MotionProfile(ABC):
    """
    Base class for defining a single-axis motion profile.

    After instantiation of this class, the following attributes of the motion
    profile are available:

    Attributes
    ----------
    `v_m`:
        Top velocity.
    `a_m`:
        Maximum acceleration.
    `ds_tot`:
        Total travel distance.
    `dt_tot`:
        Total travel time.
    `dt_ini`:
        Initial acceleration time at the beginning of the motion.
    `ds_ini`:
        Initial acceleration distance at the beginning of the motion.
    `dt_fin`:
        Final acceleration time at the end of the motion.
    `ds_fin`:
        Final acceleration distance at the end of the motion.
    `dt_cov`:
        Constant velocity time, i.e. the time between the initial acceleration
        time and the final acceleration time when the axis moves at constant
        speed.
    `ds_cov`:
        Constant velocity distance.

    After instantiation, a time profile of velocity, position, and acceleration
    can be retrieved through methods `velocity_profile()`, `position_profile()`,
    and `acceleration_profile()`.

    Notes
    -----
    The choice of units of measurement is free, but they must be consistent.
    For example, if time is measured in seconds and motor shaft position (being
    an angle) is measured in degrees, then velocity must be in degrees per
    second, and acceleration must be in degrees per square seconds.
    """
    def __init__(
        self,
        ds_tot: float,
        v_m: float,
        a_m: float | None = None,
        dt_tot: float | None = None,
        v_ini: float = 0.0,
        v_fin: float | None = 0.0,
        s_ini: float = 0.0,
    ) -> None:
        """Initializes a `MotionProfile` object.

        Parameters
        ----------
        ds_tot : float
            Total travel distance.
        v_m : float
            Top velocity of single-axis motion.
        a_m : float, optional
            Maximum (allowable) acceleration during the motion.
        dt_tot : float, optional
            Total travel time.
        v_ini : float, optional
            Initial velocity or start velocity. Default is 0.0.
        v_fin : float, optional
            Final velocity, i.e. at the end of the travel distance. Default is
            0.0.
        s_ini : float, optional
            Initial position or start position. Default is 0.0.

        A motion profile can be instantiated in two different ways:
        1.  The travel distance `ds_tot`, the top velocity `v_m`, the
            acceleration `a_m`, and the initial velocity `v_ini` and final
            velocity `v_fin` are given.
        2.  The travel distance `ds_tot`, the top velocity `v_m`, the total
            travel time `dt_tot`, and the initial velocity `v_ini` and final
            velocity `v_fin` are given.

        In multi-axis motion, the motion profile of the axis with the largest
        displacement can be calculated the first way. The motion profiles of the
        other axes are then calculated the second way using the same travel
        time `dt_tot` as the axis with the largest displacement.
        """
        self.ds_tot = ds_tot
        self.a_m = a_m
        self.v_m = v_m
        self.dt_tot = dt_tot
        self.v_ini = v_ini
        self.v_fin = v_fin
        self.s_ini = s_ini
        self._solve_motion_profile()

    def _solve_motion_profile(self) -> None:
        if self.dt_tot is None:
            self._solve_with_method1()
        else:
            self._solve_with_method2()

    def _solve_with_method1(self):
        """Solves for the attributes of the motion profile when the travel
        distance `ds_tot`, the top velocity `v_m`, the acceleration `a_m`, and
        the initial velocity `v_ini` and final velocity `v_fin` are given.
        """
        self.dv_ini = self.v_m - self.v_ini
        if self.v_fin is not None:
            self.dv_fin = self.v_fin - self.v_m
        else:
            self.dv_fin = 0.0
            self.v_fin = self.v_m
        self.dt_ini = self._calc_ini_accel_duration()
        self.dt_fin = self._calc_fin_accel_duration()
        self.ds_ini = self._calc_ini_accel_distance()
        self.ds_fin = self._calc_fin_accel_distance()
        self.ds_cov = self.ds_tot - (self.ds_ini + self.ds_fin)
        self.dt_cov = self._calc_cst_veloc_duration()
        self.dt_tot = self.dt_ini + self.dt_cov + self.dt_fin

    def _solve_with_method2(self, v_m: float | None = None) -> None:
        """Solves for the attributes of the motion profile when the travel
        distance `ds_tot`, the top velocity `v_m`, the total travel time
        `dt_tot`, and the initial velocity `v_ini` and final velocity `v_fin`
        are given.
        """
        if v_m is not None: self.v_m = v_m
        self.dv_ini = self.v_m - self.v_ini
        self.dv_fin = self.v_fin - self.v_m if self.v_fin is not None else 0.0

        # Find acceleration time for which the calculated total travel distance
        # equals the given total travel distance.
        def f(dt_ini: float) -> float:
            self.dt_ini = dt_ini
            self.a_m = self._calc_ini_accel()
            self.dt_fin = self._calc_fin_accel_duration()
            self.dt_cov = self.dt_tot - (self.dt_ini + self.dt_fin)
            self.ds_ini = self._calc_ini_accel_distance()
            self.ds_fin = self._calc_fin_accel_distance()
            self.ds_cov = self.v_m * self.dt_cov
            ds_tot = self.ds_ini + self.ds_cov + self.ds_fin
            dev = ds_tot - self.ds_tot
            return dev

        try:
            sol = scipy.optimize.root_scalar(
                f,
                bracket=[0.05 * self.dt_tot, 0.5 * self.dt_tot]
            )
        except ValueError:
            # If no solution can be found at the current speed, try with a
            # lower speed.
            self._solve_with_method2(v_m=0.95 * self.v_m)
        else:
            f(sol.root)
            self.v_fin = self.v_m

    @abstractmethod
    def _ini_accel_fun(self, t: float, abs_time: bool) -> float:
        ...

    @abstractmethod
    def _fin_accel_fun(self, t: float, abs_time: bool) -> float:
        ...

    @abstractmethod
    def _calc_ini_accel_duration(self) -> float:
        ...

    @abstractmethod
    def _calc_fin_accel_duration(self) -> float:
        ...

    @abstractmethod
    def _calc_ini_accel(self) -> float:
        ...

    def _calc_ini_accel_distance(self) -> float:
        t0 = 0.0
        t1 = t0 + self.dt_ini
        v0 = self.v_ini
        s0 = 0.0
        if t1 > t0:
            _, s, _ = position(t1, self._ini_accel_fun, t0, v0, s0, abs_time=False)
            s1 = float(s[-1])
            ds_ini = s1 - s0
            return ds_ini
        else:
            return 0.0

    def _calc_fin_accel_distance(self) -> float:
        t0 = 0.0
        t1 = t0 + self.dt_fin
        v0 = self.v_m
        s0 = 0.0
        if t1 > t0:
            _, s, _ = position(t1, self._fin_accel_fun, t0, v0, s0, abs_time=False)
            s1 = float(s[-1])
            ds_fin = s1 - s0
            return ds_fin
        else:
            return 0.0
    
    def _calc_cst_veloc_duration(self) -> float:
        dt_cov = self.ds_cov / self.v_m
        return dt_cov

    def velocity_profile(self) -> tuple[np.ndarray, np.ndarray]:
        """Calculates the velocity profile of the motion.

        Returns
        -------
        t_arr :
            Numpy array with values on the time-axis.
        v_arr:
            Numpy array with the corresponding values on the velocity-axis.
        """
        # initial ramp phase
        t0, v0 = 0.0, self.v_ini
        t1 = t0 + self.dt_ini
        if self.dt_ini > 0.0:
            t1_arr, v1_arr = velocity(t1, self._ini_accel_fun, t0, v0)
        else:
            t1_arr, v1_arr = np.array([t0]), np.array([v0])
        
        # constant velocity phase
        t1, v1 = float(t1_arr[-1]), float(v1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr, v2_arr = velocity(t2, lambda t, _: 0.0, t1, v1)
            t2, v2 = float(t2_arr[-1]), float(v2_arr[-1])
        else:
            t2_arr, v2_arr = None, None
            t2, v2 = float(t1_arr[-1]), float(v1_arr[-1])

        # final ramp phase
        if self.dt_fin > 0:
            t3 = t2 + self.dt_fin
            t3_arr, v3_arr = velocity(t3, self._fin_accel_fun, t2, v2)
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
        """Calculates the position profile of the motion.

        Returns
        -------
        t_arr :
            Numpy array with values on the time-axis.
        s_arr:
            Numpy array with the corresponding values on the position-axis.
        """
        # initial ramp phase
        t0, v0, s0 = 0.0, self.v_ini, self.s_ini
        t1 = t0 + self.dt_ini
        if self.dt_ini > 0.0:
            t1_arr, s1_arr, v1_arr = position(t1, self._ini_accel_fun, t0, v0, s0)
        else:
            t1_arr, s1_arr, v1_arr = np.array([t0]), np.array([s0]), np.array([v0])
        
        # constant velocity phase
        t1, v1, s1 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr, s2_arr, v2_arr = position(t2, lambda t, _: 0.0, t1, v1, s1)
            t2, v2, s2 = float(t2_arr[-1]), float(v2_arr[-1]), float(s2_arr[-1])
        else:
            t2_arr, s2_arr, v2_arr = None, None, None
            t2, v2, s2 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])

        # final ramp phase
        if self.dt_fin > 0.0:
            t3 = t2 + self.dt_fin
            t3_arr, s3_arr, _ = position(t3, self._fin_accel_fun, t2, v2, s2)
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
        """Calculates the acceleration profile of the motion.

        Returns
        -------
        t_arr :
            Numpy array with values on the time-axis.
        a_arr:
            Numpy array with the corresponding values on the acceleration-axis.
        """
        # initial ramp phase
        t0, a0 = 0.0, 0.0
        t1 = t0 + self.dt_ini
        t1_arr = np.linspace(t0, t1, endpoint=True)
        a1_arr = np.array([self._ini_accel_fun(t, abs_time=True) for t in t1_arr])

        # constant velocity phase
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

        # final ramp phase
        if self.dt_fin > 0:
            t3 = t2 + self.dt_fin
            t3_arr = np.linspace(t2, t3, endpoint=True)
            a3_arr = np.array([self._fin_accel_fun(t, abs_time=True) for t in t3_arr])
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

    def get_velocity_time_fn(
        self,
        N: float | None = None
    ) -> Callable[[float], float]:
        """Returns a function that takes a time moment `t` and returns the 
        velocity `v` at that time moment (`0 <= t <= dt_tot`).

        Parameters
        ----------
        N: float, optional
            Transmission ratio (the ratio of motor speed to load speed).
        """
        t_ax, v_ax = self.velocity_profile()

        if N is not None:
            v_ax = N * v_ax

        interp = scipy.interpolate.interp1d(t_ax, v_ax)

        def f(t: float) -> float:
            try:
                v = interp(t)
            except ValueError:
                return 0.0
            return v

        return f

    def get_position_time_fn(
        self,
        N: float | None = None
    ) -> Callable[[float], float]:
        """Returns a function that takes a time moment `t` and returns the 
        position `s` at that time moment in the movement (`0 <= t <= dt_tot`).

        Parameters
        ----------
        N: float, optional
            Transmission ratio (the ratio of motor speed to load speed).
        """
        t_ax, s_ax = self.position_profile()

        if N is not None:
            s_ax = N * s_ax

        interp = scipy.interpolate.interp1d(t_ax, s_ax)

        def f(t: float) -> float:
            try:
                s = interp(t)
            except ValueError:
                s = s_ax[0]
                if t > t_ax[-1]:
                    s = s_ax[-1]
            return s

        return f

    def get_time_position_fn(
        self,
        N: float | None = None
    ) -> Callable[[float], float]:
        """Returns a function that takes a position `s` and returns the time 
        moment `t` this position is reached in the movement (`0 <= s <= ds_tot`).

        Parameters
        ----------
        N:
            Transmission ratio (ratio of motor speed to load speed).
        """
        t_ax, s_ax = self.position_profile()

        if N is not None:
            s_ax = N * s_ax

        interp = scipy.interpolate.interp1d(s_ax, t_ax)

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
        """Returns a function that takes a time moment `t` in seconds and 
        returns the velocity `v` at that moment during the initial ramp phase
        of the movement (`0 <= t <= dt_ini`).
        If `t > dt_ini`, the velocity at `dt_ini` is returned, i.e. also the
        velocity during the constant-velocity phase.
        """
        t0, v0 = 0.0, 0.0
        t1 = t0 + self.dt_ini
        t_arr, v_arr = velocity(t1, self._ini_accel_fun, t0, v0)
        v_max = v_arr[-1]

        interp = scipy.interpolate.interp1d(t_arr, v_arr)

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
        """Returns a function that takes a time moment `t` in seconds and 
        returns the velocity `v` at that moment during the final ramp phase
        of the movement (`t0 <= t <= t0 + dt_fin`).

        Parameters
        ----------
        t0 : float
            Time moment the final ramp phase begins.
        v0: float
            Initial velocity at the start of the final ramp phase.
        """
        t1 = t0 + self.dt_fin
        t_arr, v_arr = velocity(t1, self._fin_accel_fun, t0, v0)

        interp = scipy.interpolate.interp1d(t_arr, v_arr)

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
        """Returns a function that takes a position `s` and returns the time 
        moment `t` in seconds when this position is reached.
        """
        t0, v0, s0 = 0.0, 0.0, 0.0
        t1 = t0 + self.dt_ini
        t_arr, s_arr, v_arr = position(t1, self._ini_accel_fun, t0, v0, s0)
        t_min = t_arr[0]
        t_max = t_arr[-1]
        s_max = s_arr[-1]

        interp_s = scipy.interpolate.interp1d(s_arr, t_arr)

        def f(s: float) -> float:
            if s0 <= s <= s_max:
                t = interp_s(s)
            elif s > s_max:
                t = t_max
            else:
                t = t_min
            return t

        return f

    def get_ini_time_velocity_fn(self) -> Callable[[float], float]:
        """Returns a function that takes a velocity `v` and returns the 
        time moment `t` in seconds when this velocity is reached.
        """
        t0, v0, s0 = 0.0, 0.0, 0.0
        t1 = t0 + self.dt_ini
        t_arr, s_arr, v_arr = position(t1, self._ini_accel_fun, t0, v0, s0)
        t_min = t_arr[0]
        t_max = t_arr[-1]
        v_max = v_arr[-1]

        interp_v = scipy.interpolate.interp1d(v_arr, t_arr)

        def f(v: float) -> float:
            if v0 <= v <= v_max:
                t = interp_v(v)
            elif v > v_max:
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
        """Returns a function that takes a position `s` and returns the time 
        moment `t` in seconds when this position is reached.

        Parameters
        ----------
        t0 : float
            Time moment the final ramp phase begins.
        s0 : float
            Initial position at the start of the final ramp phase.
        v0: float
            Initial velocity at the start of the final ramp phase.
        """
        t1 = t0 + self.dt_fin
        t_arr, v_arr = velocity(t1, self._fin_accel_fun, t0, v0)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions. 
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = scipy.interpolate.interp1d(t_arr, v_arr)
            sol = scipy.optimize.root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self._fin_accel_fun, t0, v0, s0)
        t_min = t_arr[0]
        t_max = t_arr[-1]
        s_max = s_arr[-1]

        interp_s = scipy.interpolate.interp1d(s_arr, t_arr)

        def f(s: float) -> float:
            if s0 <= s <= s_max:
                t = interp_s(s)
            elif s > s_max:
                t = t_max
            else:
                t = t_min
            return t

        return f

    def get_fin_time_velocity_fn(
        self,
        t0: float,
        s0: float,
        v0: float
    ) -> Callable[[float], float]:
        """Returns a function that takes a velocity `v` and returns the 
        time moment `t` in seconds when this velocity is reached.

        Parameters
        ----------
        t0 : float
            Time moment the final ramp phase begins.
        s0 : float
            Initial position at the start of the final ramp phase.
        v0: float
            Initial velocity at the start of the final ramp phase.
        """
        t1 = t0 + self.dt_fin
        t_arr, v_arr = velocity(t1, self._fin_accel_fun, t0, v0)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions. 
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = scipy.interpolate.interp1d(t_arr, v_arr)
            sol = scipy.optimize.root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self._fin_accel_fun, t0, v0, s0)
        t_min = t_arr[0]
        t_max = t_arr[-1]

        interp_v = scipy.interpolate.interp1d(v_arr, t_arr)

        def f(v: float) -> float:
            if v0 >= v >= 0.0:
                t = interp_v(v)
            elif 0.0 > v:
                t = t_max
            else:
                t = t_min
            return t

        return f


class TrapezoidalProfile(MotionProfile):
    """
    Use this class to define a trapezoidal motion profile.
    """

    def _ini_accel_fun(self, t: float, abs_time: bool = False) -> float:
        if self.dv_ini != 0.0:
            return self.a_m
        else:
            return 0.0

    def _fin_accel_fun(self, t: float, abs_time: bool = False) -> float:
        if self.dv_fin != 0:
            return -self.a_m
        else:
            return 0.0

    def _calc_ini_accel_duration(self) -> float:
        dt_ini = abs(self.dv_ini / self.a_m)
        return dt_ini

    def _calc_fin_accel_duration(self) -> float:
        dt_fin = abs(self.dv_fin / self.a_m)
        return dt_fin

    def _calc_ini_accel(self) -> float:
        return self.dv_ini / self.dt_ini


class SCurvedProfile(MotionProfile):
    """
    Use this class to define a pure S-curve motion profile.
    """

    def _ini_accel_fun(self, t: float, abs_time: bool = False) -> float:
        if self.dv_ini != 0.0:
            c1 = self.a_m ** 2 / abs(self.dv_ini)
            if self.dv_ini < 0: 
                c1 *= -1
        else:
            c1 = 0.0
        t0 = 0.0
        t1 = t0 + self.dt_ini / 2
        t2 = t1 + self.dt_ini / 2
        if t0 <= t <= t1:
            return c1 * t
        elif t1 < t <= t2:
            return -c1 * (t - t2)
        else:
            return 0.0

    def _fin_accel_fun(self, t: float, abs_time: bool = False) -> float:
        if self.dv_fin != 0.0:
            c1 = self.a_m ** 2 / abs(self.dv_fin)
            if self.dv_fin > 0: 
                c1 *= -1
        else:
            c1 = 0.0
        if abs_time:
            t0 = self.dt_ini + self.dt_cov
        else:
            t0 = 0.0
        t1 = t0 + self.dt_fin / 2
        t2 = t1 + self.dt_fin / 2
        if t0 <= t <= t1:
            return -c1 * (t - t0)
        elif t1 < t <= t2:
            return c1 * (t - t2)
        else:
            return 0.0

    def _calc_ini_accel_duration(self) -> float:
        dt_ini = abs(2 * self.dv_ini / self.a_m)
        return dt_ini

    def _calc_fin_accel_duration(self) -> float:
        dt_fin = abs(2 * self.dv_fin / self.a_m)
        return dt_fin

    def _calc_ini_accel(self) -> float:
        return 2 * self.dv_ini / self.dt_ini


class ProfileType(StrEnum):
    TRAPEZOIDAL = "trapezoidal"
    S_CURVED = "S-curved"

    @classmethod
    def get_types(cls) -> list[str]:
        return [cls.TRAPEZOIDAL, cls.S_CURVED]


Point = tuple[float, float]


class XYMotionControl:
    """
    Calculates the motion profiles along the X-axis and along the Y-axis to
    move in a straight line from one point to another point (synchronised X- and
    Y-axis motion).
    """
    def __init__(
        self,
        pitch: float,
        motor_speed: float,
        motor_accel: float,
        profile_type: ProfileType = ProfileType.TRAPEZOIDAL
    ) -> None:
        """Initializes the `XYMotionControl` object.

        Parameters
        ----------
        pitch: float
            Pitch of the lead screw (i.e. number of revolutions of the screw to
            move the nut one meter).
        motor_speed:
            Available angular speed of the motor (deg/s).
        motor_accel:
            Available angular acceleration of the motor (deg/s²). This will
            depend on the torque capabilities of the motor and on the total 
            inertia moment of the motor and the connected load.
        profile_type : ProfileType
            The type of motion profile. Either a trapezoidal (default) or 
            S-curved motion profile.
        """
        if profile_type in ProfileType.get_types():
            self.profile_type = profile_type
        else:
            raise TypeError(f"Profile type '{profile_type}' does not exist.")
        self.pitch = pitch
        self.motor_speed = motor_speed
        self.motor_accel = motor_accel

        self._N = 360.0 * self.pitch  # convert revs/m -> deg/m
        self._pt_start: Point = (0.0, 0.0)
        self._pt_end: Point = (0.0, 0.0)
        self._angle_ini_x: float = 0.0
        self._angle_ini_y: float = 0.0
        self._speed_ini_x: float = 0.0
        self._speed_ini_y: float = 0.0
        self._speed_fin_x: float | None = None
        self._speed_fin_y: float | None = None
        self._direction_x: Direction = Direction.COUNTERCLOCKWISE
        self._direction_y: Direction = Direction.COUNTERCLOCKWISE
        self._dx: float = 0.0
        self._dy: float = 0.0
        self._dtheta_x: float = 0.0
        self._dtheta_y: float = 0.0
        self._mp_x: MotionProfile | None = None
        self._mp_y: MotionProfile | None = None

    def set_points(self, pt_start: Point, pt_end: Point) -> None:
        """Sets the start and end point of the linear motion."""
        self._pt_start = pt_start
        self._pt_end = pt_end
        
        self._angle_ini_x = self._N * pt_start[0]
        self._angle_ini_y = self._N * pt_start[1]
        
        self._dx = self._pt_end[0] - self._pt_start[0]
        self._dy = self._pt_end[1] - self._pt_start[1]
        self._dtheta_x = self._N * self._dx
        self._dtheta_y = self._N * self._dy
        
        self._direction_x = Direction.COUNTERCLOCKWISE
        self._direction_y = Direction.COUNTERCLOCKWISE
        if self._dx < 0.0:
            self._direction_x = Direction.CLOCKWISE
        if self._dy < 0.0:
            self._direction_y = Direction.CLOCKWISE
        
        # Erase existing motion profiles
        self._mp_x, self._mp_y = None, None
 
    def set_boundary_velocities(
        self,
        speed_ini_x: float = 0.0,
        speed_ini_y: float = 0.0,
        speed_fin_x: float | None = None,
        speed_fin_y: float | None = None
    ) -> None:
        """Sets the initial and final velocities of the linear motion in x and y.

        The final velocity can be `None`, meaning that it is unknown. In that
        case the final velocity will be equal to the top velocity of the 
        motion.
        """
        self._speed_ini_x = speed_ini_x
        self._speed_ini_y = speed_ini_y
        self._speed_fin_x = speed_fin_x
        self._speed_fin_y = speed_fin_y

        # Erase existing motion profiles
        self._mp_x, self._mp_y = None, None
   
    def _create_motion_profile(self, **kwargs) -> MotionProfile:
        # noinspection PyUnreachableCode
        match self.profile_type:
            case ProfileType.TRAPEZOIDAL:
                MP = TrapezoidalProfile
            case ProfileType.S_CURVED:
                MP = SCurvedProfile
            case _:
                MP = TrapezoidalProfile
        return MP(**kwargs)
    
    def _calc_motion_profiles(self) -> None:
        """Calculates the synchronised profiles of X- and Y-axis motion.

        The X- and Y-axis motion will have the same start and ending time,
        independent of their displacement.
        """
        # Motion starts from rest: the axis with the largest move determines
        # the total travel time of both axis moves.
        if self._speed_ini_x == 0.0 and self._speed_ini_y == 0.0:
            dev = round(self._dtheta_x - self._dtheta_y, 6)
            if dev >= 0:
                mp_x = self._create_motion_profile(
                    ds_tot=self._dtheta_x,
                    v_m=self.motor_speed,
                    a_m=self.motor_accel,
                    v_ini=self._speed_ini_x,
                    v_fin=self._speed_fin_x,
                    s_ini=self._angle_ini_x
                )
                mp_y = self._create_motion_profile(
                    ds_tot=self._dtheta_y,
                    v_m=self.motor_speed,
                    a_m=self.motor_accel,
                    dt_tot=mp_x.dt_tot,
                    v_ini=self._speed_ini_y,
                    v_fin=self._speed_fin_y,
                    s_ini=self._angle_ini_y
                )
            else:
                mp_y = self._create_motion_profile(
                    ds_tot=self._dtheta_y,
                    v_m=self.motor_speed,
                    a_m=self.motor_accel,
                    v_ini=self._speed_ini_y,
                    v_fin=self._speed_fin_y,
                    s_ini=self._angle_ini_y
                )
                mp_x = self._create_motion_profile(
                    ds_tot=self._dtheta_x,
                    v_m=self.motor_speed,
                    a_m=self.motor_accel,
                    dt_tot=mp_y.dt_tot,
                    v_ini=self._speed_ini_x,
                    v_fin=self._speed_fin_x,
                    s_ini=self._angle_ini_x
                )
        # Both axes are moving at the start of an intermediate segment with
        # known initial velocity.
        else:
            dt_tot_x = self._dtheta_x / self._speed_ini_x
            dt_tot_y = self._dtheta_y / self._speed_ini_y
            dev = round(dt_tot_x - dt_tot_y, 6)
            if dev >= 0.0:
                mp_x = self._create_motion_profile(
                    ds_tot=self._dtheta_x,
                    v_m=self._speed_ini_x,
                    a_m=self.motor_accel,
                    v_ini=self._speed_ini_x,
                    v_fin=self._speed_fin_x,
                    s_ini=self._angle_ini_x
                )
                mp_y = self._create_motion_profile(
                    ds_tot=self._dtheta_y,
                    v_m=self._speed_ini_x,
                    a_m=self.motor_accel,
                    dt_tot=mp_x.dt_tot,
                    v_ini=self._speed_ini_y,
                    v_fin=self._speed_fin_y,
                    s_ini=self._angle_ini_y
                )
            else:
                mp_y = self._create_motion_profile(
                    ds_tot=self._dtheta_y,
                    v_m=self._speed_ini_y,
                    a_m=self.motor_accel,
                    v_ini=self._speed_ini_y,
                    v_fin=self._speed_fin_y,
                    s_ini=self._angle_ini_y
                )
                mp_x = self._create_motion_profile(
                    ds_tot=self._dtheta_x,
                    v_m=self._speed_ini_y,
                    a_m=self.motor_accel,
                    dt_tot=mp_y.dt_tot,
                    v_ini=self._speed_ini_x,
                    v_fin=self._speed_fin_x,
                    s_ini=self._angle_ini_x
                )
        self._mp_x = mp_x
        self._mp_y = mp_y
    
    @property
    def x_motion(self) -> tuple[MotionProfile, Direction]:
        """Returns the motion profile and rotation direction for the X-axis."""
        if self._mp_x is None: self._calc_motion_profiles()
        return self._mp_x, self._direction_x
    
    @property
    def y_motion(self) -> tuple[MotionProfile, Direction]:
        """Returns the motion profile and rotation direction for the Y-axis."""
        if self._mp_y is None: self._calc_motion_profiles()
        return self._mp_y, self._direction_y
    