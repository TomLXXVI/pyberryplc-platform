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

    After instantiation of this class, following attributes of the motion
    profile are available:

    Attributes
    ----------
    `v_m`:
        Maximum velocity of the motor.
    `v_top`:
        Top velocity of the motion profile.
    `a_m`:
        Maximum acceleration of the motor.
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
    The choice of units of measurement is free, but must be consistent.
    For example, if time is measured in seconds and motor shaft position (being
    an angle) is measured in degrees, then velocity must be in degrees per
    second, and acceleration must be in degrees per square seconds.
    """
    def __init__(
        self,
        ds_tot: float,
        v_m: float,
        a_m: float,
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
            Maximum (allowable) velocity of the motor.
        a_m : float
            Maximum (allowable) acceleration of the motor.
        dt_tot : float, optional
            Total travel time.
        v_ini : float, optional
            Initial velocity or start velocity. Default is 0.0.
        v_fin : float, optional
            Final velocity, i.e. at the end of the travel distance. Default is
            0.0. If `None` the final velocity will be set equal to the top
            velocity of the motion profile.
        s_ini : float, optional
            Initial position or start position. Default is 0.0.

        A motion profile can be instantiated in two different ways:

        1.  The travel distance `ds_tot` of the movement, the acceleration `a_m`
            of the motor, and the initial velocity `v_ini` and final velocity
            `v_fin` of the movement are known.
            The top velocity `v_top` will then be taken as the maximum
            velocity `v_m` of the motor. The total travel time `dt_tot` is then
            still to be determined.

        2.  The total travel distance `ds_tot` and total travel time `dt_tot` of
            the movement, the acceleration `a_m` of the motor, and the initial
            velocity `v_ini` and final velocity `v_fin` of the movement are
            known. The top velocity `v_top` of the movement still needs to be
            determined, but cannot exceed the maximum velocity `v_m` of the
            motor.
        """
        self.ds_tot = ds_tot
        self.a_m = a_m
        self.v_m = v_m
        self.v_top = v_m
        self.dt_tot = dt_tot
        self.v_ini = v_ini
        self.v_fin = v_fin
        self.s_ini = s_ini
        
        if self.ds_tot > 0.0:
            self._solve_motion_profile()
        else:
            self._set_no_movement()

    def _solve_motion_profile(self) -> None:
        if self.dt_tot is None:
            self._solve_with_method1()
        else:
            self._solve_with_method2()

    def _solve_with_method1(self):
        """Solves for the attributes of the motion profile when travel distance
        `ds_tot`, top velocity `v_top` = `v_m`, maximum acceleration `a_m`, and
        initial and final velocity `v_ini` and `v_fin` are given.
        """
        self.dv_ini = self.v_top - self.v_ini
        if self.v_fin is not None:
            self.dv_fin = self.v_fin - self.v_top
        else:
            self.dv_fin = 0.0
            self.v_fin = self.v_top
        self.dt_ini = self._calc_ini_accel_duration()
        self.dt_fin = self._calc_fin_accel_duration()
        self.ds_ini = self._calc_ini_accel_distance()
        self.ds_fin = self._calc_fin_accel_distance()
        self.ds_cov = self.ds_tot - (self.ds_ini + self.ds_fin)
        self.dt_cov = self._calc_cst_veloc_duration()
        self.dt_tot = self.dt_ini + self.dt_cov + self.dt_fin
    
    def _solve_with_method2(self) -> None:
        """Solves for the attributes of the motion profile when travel distance
        `ds_tot`, travel time `dt_tot`, acceleration `a_m`, and initial and
        final velocity `v_ini` and `v_fin` are given. Looks for the top velocity
        `v_top` for which the total travel distance `ds_tot` is finished within 
        the specified total travel time `dt_tot`.
        """
        def fn(v):
            self.v_top = v
            self.dv_ini = self.v_top - self.v_ini
            if self.v_fin is not None:
                self.dv_fin = self.v_fin - self.v_top
            else:
                self.dv_fin = 0.0
            self.dt_ini = self._calc_ini_accel_duration()
            self.dt_fin = self._calc_fin_accel_duration()
            self.ds_ini = self._calc_ini_accel_distance()
            self.ds_fin = self._calc_fin_accel_distance()
            self.ds_cov = self.ds_tot - (self.ds_ini + self.ds_fin)
            self.dt_cov = self.dt_tot - (self.dt_ini + self.dt_fin)
            v_new = self.ds_cov / self.dt_cov
            dev = v_new - v
            return dev

        # Find top velocity (between 0 and maximum motor speed) so that the
        # displacement `ds_tot` will be finished after `dt_tot` time.
        # If no solution is found within the bracket, `root_scalar()` raises a
        # `ValueError`.
        sol = scipy.optimize.root_scalar(fn, bracket=[0.0, self.v_m])
        self.v_top = sol.root
        self.v_fin = self.v_top if self.v_fin is None else self.v_fin
        fn(self.v_top)
    
    def _set_no_movement(self):
        self.v_ini = 0.0
        self.dv_ini = 0.0
        self.v_top = 0.0
        self.v_fin = 0.0
        self.dv_fin = 0.0
        self.ds_ini = 0.0
        self.ds_cov = 0.0
        self.ds_fin = 0.0
        self.ds_tot = 0.0
        self.dt_ini = 0.0
        self.dt_cov = self.dt_tot
        self.dt_fin = 0.0
        self.a_m = 0.0
    
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
        v0 = self.v_top
        s0 = 0.0
        if t1 > t0:
            _, s, _ = position(t1, self._fin_accel_fun, t0, v0, s0, abs_time=False)
            s1 = float(s[-1])
            ds_fin = s1 - s0
            return ds_fin
        else:
            return 0.0
    
    def _calc_cst_veloc_duration(self) -> float:
        dt_cov = self.ds_cov / self.v_top
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

        if N is not None: v_ax *= N

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

        if N is not None: s_ax *= N

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

        if N is not None: s_ax *= N

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
        t0 = 0.0
        t1 = t0 + self.dt_ini

        if t0 <= t <= t1:
            if self.dv_ini > 0.0:
                return self.a_m
            elif self.dv_ini < 0.0:
                return -self.a_m
            else:
                return 0.0
        else:
            return 0.0

    def _fin_accel_fun(self, t: float, abs_time: bool = False) -> float:
        t0 = self.dt_ini + self.dt_cov if abs_time else 0.0
        t1 = t0 + self.dt_fin

        if t0 <= t <= t1:
            if self.dv_fin > 0.0:
                return self.a_m
            elif self.dv_fin < 0.0:
                return -self.a_m
            else:
                return 0.0
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
            if self.dv_fin > 0: c1 *= -1
        else:
            c1 = 0.0

        t0 = self.dt_ini + self.dt_cov if abs_time else 0.0
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
