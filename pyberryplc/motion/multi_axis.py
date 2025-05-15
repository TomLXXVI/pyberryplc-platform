"""
Definition of motion profiles for single-axis motion.

Two types of profiles are implemented in this module:
- Class `TrapezoidalProfile` defines a trapezoidal motion profile.
- Class `SCurvedProfile` defines a pure S-curve motion profile.

References
----------
Gürocak, H. (2016), Industrial Motion Control, John Wiley & Sons.
"""
from typing import Callable
from abc import ABC, abstractmethod

import numpy as np
import scipy

from .kinematics import position, velocity


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
    """
    def __init__(
        self,
        ds_tot: float,
        v_m: float,
        a_m: float | None = None,
        dt_tot: float | None = None,
        v_ini: float = 0.0,
        v_fin: float | None = 0.0
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

        self.dv_ini = self.v_m - self.v_ini
        if self.v_fin is not None:
            self.dv_fin = self.v_fin - self.v_m
        else:
            self.dv_fin = 0.0
            self.v_fin = self.v_m
        
        self._solve_motion_profile()            

    @abstractmethod
    def _ini_ramp_fun(self, t: float) -> float:
        ...

    @abstractmethod
    def _fin_ramp_fun(self, t: float) -> float:
        ...

    @abstractmethod
    def _calc_ini_ramp_duration(self) -> float:
        ...

    @abstractmethod
    def _calc_fin_ramp_duration(self) -> float:
        ...

    @abstractmethod
    def _calc_ini_ramp_accel(self) -> float:
        ...

    def _calc_ini_ramp_distance(self) -> float:
        t0 = 0.0
        t1 = t0 + self.dt_ini
        v0 = 0.0
        s0 = 0.0
        _, s, _ = position(t1, self._ini_ramp_fun, t0, v0, s0)
        s1 = float(s[-1])
        ds_ini = s1 - s0
        return ds_ini

    def _calc_fin_ramp_distance(self) -> float:
        t0 = 0.0
        t1 = t0 + self.dt_fin
        v0 = self.v_m
        s0 = 0.0
        if t1 > t0:
            _, s, _ = position(t1, self._fin_ramp_fun, t0, v0, s0)
            s1 = float(s[-1])
            ds_fin = s1 - s0
            return ds_fin
        else:
            return 0.0
    
    def _calc_cst_veloc_duration(self) -> float:
        dt_cov = self.ds_cov / self.v_m
        return dt_cov

    def _solve_motion_profile(self) -> None:
        if self.dt_tot is None:
            self.dt_ini = self._calc_ini_ramp_duration()
            self.dt_fin = self._calc_fin_ramp_duration()
            self.ds_ini = self._calc_ini_ramp_distance()
            self.ds_fin = self._calc_fin_ramp_distance()
            self.ds_cov = self.ds_tot - (self.ds_ini + self.ds_fin)
            self.dt_cov = self._calc_cst_veloc_duration()
            self.dt_tot = self.dt_ini + self.dt_cov + self.dt_fin
        else:

            def f(dt_ini: float) -> float:
                self.dt_ini = dt_ini
                self.a_m = self._calc_ini_ramp_accel()
                self.dt_fin = self._calc_fin_ramp_duration()
                self.dt_cov = self.dt_tot - (self.dt_ini + self.dt_fin)
                self.ds_ini = self._calc_ini_ramp_distance()
                self.ds_fin = self._calc_fin_ramp_distance()
                self.ds_cov = self.v_m * self.dt_cov
                ds_tot = self.ds_ini + self.ds_cov + self.ds_fin
                dev = ds_tot - self.ds_tot
                return dev
            
            try:
                sol = scipy.optimize.root_scalar(f, bracket=[0.05 * self.dt_tot, 0.5 * self.dt_tot])
            except ValueError:
                pass
            else:
                f(sol.root)

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
        t1_arr, v1_arr = velocity(t1, self._ini_ramp_fun, t0, v0)

        # constant velocity phase
        t1, v1 = float(t1_arr[-1]), float(v1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr, v2_arr = velocity(t2, lambda t: 0.0, t1, v1)
            t2, v2 = float(t2_arr[-1]), float(v2_arr[-1])
        else:
            t2_arr, v2_arr = None, None
            t2, v2 = float(t1_arr[-1]), float(v1_arr[-1])

        # final ramp phase
        if self.dt_fin > 0:
            t3 = t2 + self.dt_fin
            t3_arr, v3_arr = velocity(t3, self._fin_ramp_fun, t2, v2)
        else:
            t3_arr = np.array([])
            v3_arr = np.array([])
        
        if t2_arr is None:
            t_arr = np.concatenate((t1_arr[:-1], t3_arr))
        else:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr[:-1], t3_arr))

        if v2_arr is None:
            v_arr = np.concatenate((v1_arr[:-1], v3_arr))
        else:
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
        t0, v0, s0 = 0.0, self.v_ini, 0.0
        t1 = t0 + self.dt_ini
        t1_arr, s1_arr, v1_arr = position(t1, self._ini_ramp_fun, t0, v0, s0)

        # constant velocity phase
        t1, v1, s1 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr, s2_arr, v2_arr = position(t2, lambda t: 0.0, t1, v1, s1)
            t2, v2, s2 = float(t2_arr[-1]), float(v2_arr[-1]), float(s2_arr[-1])
        else:
            t2_arr, s2_arr, v2_arr = None, None, None
            t2, v2, s2 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])

        # final ramp phase
        if self.dt_fin > 0.0:
            t3 = t2 + self.dt_fin
            t3_arr, s3_arr, _ = position(t3, self._fin_ramp_fun, t2, v2, s2)
        else:
            t3_arr = np.array([])
            s3_arr = np.array([])
        
        if t2_arr is None:
            t_arr = np.concatenate((t1_arr[:-1], t3_arr))
        else:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr[:-1], t3_arr))

        if s2_arr is None:
            s_arr = np.concatenate((s1_arr[:-1], s3_arr))
        else:
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
        a1_arr = np.array([self._ini_ramp_fun(t) for t in t1_arr])

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
            a3_arr = np.array([self._fin_ramp_fun(t) for t in t3_arr])
        else:
            t3_arr = np.array([])
            a3_arr = np.array([])
        
        if t2_arr is None:
            t_arr = np.concatenate((t1_arr[:-1], t3_arr))
        else:
            t_arr = np.concatenate((t1_arr[:-1], t2_arr[:-1], t3_arr))

        if a2_arr is None:
            a_arr = np.concatenate((a1_arr[:-1], a3_arr))
        else:
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
        t_arr, v_arr = velocity(t1, self._ini_ramp_fun, t0, v0)
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
        t_arr, v_arr = velocity(t1, self._fin_ramp_fun, t0, v0)

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
        t_arr, s_arr, v_arr = position(t1, self._ini_ramp_fun, t0, v0, s0)
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
        t_arr, s_arr, v_arr = position(t1, self._ini_ramp_fun, t0, v0, s0)
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
        t_arr, v_arr = velocity(t1, self._fin_ramp_fun, t0, v0)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions. 
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = scipy.interpolate.interp1d(t_arr, v_arr)
            sol = scipy.optimize.root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self._fin_ramp_fun, t0, v0, s0)
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
        t_arr, v_arr = velocity(t1, self._fin_ramp_fun, t0, v0)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions. 
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = scipy.interpolate.interp1d(t_arr, v_arr)
            sol = scipy.optimize.root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self._fin_ramp_fun, t0, v0, s0)
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

    def _ini_ramp_fun(self, t: float) -> float:
        if self.dv_ini > 0:
            return self.a_m
        else:
            return -self.a_m

    def _fin_ramp_fun(self, t: float) -> float:
        if self.dv_fin > 0:
            return self.a_m
        else:
            return -self.a_m

    def _calc_ini_ramp_duration(self) -> float:
        dt_ini = abs(self.dv_ini / self.a_m)
        return dt_ini

    def _calc_fin_ramp_duration(self) -> float:
        dt_fin = abs(self.dv_fin / self.a_m)
        return dt_fin

    def _calc_ini_ramp_accel(self) -> float:
        return self.v_m / self.dt_ini


class SCurvedProfile(MotionProfile):
    """
    Use this class to define a pure S-curve motion profile.
    """

    def _ini_ramp_fun(self, t: float) -> float:
        c1 = self.a_m ** 2 / abs(self.dv_ini)
        if self.dv_ini < 0: c1 *= -1
        t0 = 0.0
        t1 = t0 + self.dt_ini / 2
        t2 = t1 + self.dt_ini / 2
        if t0 <= t <= t1:
            return c1 * t
        elif t1 < t <= t2:
            return -c1 * (t - t2)
        else:
            return 0.0

    def _fin_ramp_fun(self, t: float) -> float:
        c1 = self.a_m ** 2 / abs(self.dv_ini)
        if self.dv_ini > 0: c1 *= -1
        t0 = self.dt_ini + self.dt_cov
        t1 = t0 + self.dt_fin / 2
        t2 = t1 + self.dt_fin / 2
        if t0 <= t <= t1:
            return -c1 * (t - t0)
        elif t1 < t <= t2:
            return c1 * (t - t2)
        else:
            return 0.0

    def _calc_ini_ramp_duration(self) -> float:
        dt_ini = abs(2 * self.dv_ini / self.a_m)
        return dt_ini

    def _calc_fin_ramp_duration(self) -> float:
        dt_fin = abs(2 * self.dv_fin / self.a_m)
        return dt_fin

    def _calc_ini_ramp_accel(self) -> float:
        return 2 * self.v_m / self.dt_ini


def xy_motion_control(
    p1: tuple[float, float],
    p2: tuple[float, float],
    pitch: float,
    omega: float,
    alpha: float,
    profile_type: str = "trapezoidal"
) -> tuple[MotionProfile, MotionProfile]:
    """Calculates the required motion profiles along the X- and Y-axis such that
    both axes finish their moves at the same time.

    Parameters
    ----------
    p1 : Position
        Start position with (x1, y1) coordinates in meters.
    p2 : Position
        End position with (x2, y2) coordinates in meters.
    pitch:
        Pitch of the lead screw (i.e. the number of revolutions of the screw to
        move the nut one meter).
    omega:
        Angular speed of the motor shaft in degrees per second.
    alpha:
        Angular acceleration in deg/s².
    profile_type : str, ["trapezoidal", "S-curve"]
        Type of motion profile: either a trapezoidal profile (i.e. the default)
        or an S-curved profile.

    Returns
    -------
    mp_x : MotionProfile
        Required motion profile along the x-axis.
    mp_y : MotionProfile
        Required motion profile along the y-axis.
    """
    # travel distance along the X-axis and Y-axis
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    # transmission ratio of lead screw (deg/m)
    N = 360.0 * pitch
    # angular travel of X- and Y-motor shaft (deg)
    dtheta_x = N * dx
    dtheta_y = N * dy
    # motion profile of X- and Y-motor.
    if profile_type == "trapezoidal":
        mp = TrapezoidalProfile
    else:
        mp = SCurvedProfile
    if dx >= dy:
        mp_x = mp(a_m=alpha, ds_tot=dtheta_x, v_m=omega)
        mp_y = mp(a_m=alpha, v_m=omega, ds_tot=dtheta_y, dt_tot=mp_x.dt_tot)
    else:
        mp_y = mp(a_m=alpha, ds_tot=dtheta_y, v_m=omega)
        mp_x = mp(a_m=alpha, v_m=omega, ds_tot=dtheta_x, dt_tot=mp_y.dt_tot)
    return mp_x, mp_y
