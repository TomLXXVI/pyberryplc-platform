"""
Definition of motion profiles for single-axis motion.

Two types of profiles are implemented in this module:
- Class `TrapezoidalProfile` defines a trapezoidal motion profile.
- Class `SCurvedProfile` defines a pure S-curve motion profile.

References
----------
Gürocak, H. (2016), Industrial Motion Control, John Wiley & Sons.
"""
from __future__ import annotations
from typing import Callable
from abc import ABC, abstractmethod

import numpy as np
import scipy

from .kinematics import position, velocity



class ConfigError(Exception):
    pass


class DistanceError(Exception):
    pass


class TimingError(Exception):
    pass



class MotionProfile(ABC):
    """
    Base class that defines a symmetrical single-axis motion profile.
    The acceleration and deceleration region of the velocity profile have the
    same shape and dimensions.

    After instantiation of this class, the following properties of the motion 
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
    `dt_acc`:
        Acceleration time.
    `ds_acc`:
        Acceleration distance.
    `dt_dec`:
        Deceleration time (always equal to acceleration time).
    `ds_dec`:
        Deceleration distance (always equal to acceleration distance).
    `dt_cov`:
        Constant velocity time, i.e. the time between the acceleration time
        and the deceleration time when the axis moves at constant speed.
    `ds_cov`:
        Constant velocity distance.

    Also, a time profile of the velocity, position, or acceleration can be
    calculated using the methods `velocity_profile()`, `position_profile()`, or
    `acceleration_profile()`.
    """

    # noinspection PyUnresolvedReferences
    def __init__(
        self,
        v_m: float | None = None,
        a_m: float | None = None,
        dt_tot: float | None = None,
        ds_tot: float | None = None,
        dt_acc: float | None = None
    ) -> None:
        """Creates a `MotionProfile` object.

        Parameters
        ----------
        v_m:
            Top velocity of the single-axis motion.
        a_m:
            Maximum (allowable) acceleration during the motion.
        dt_tot:
            Total travel time (motion time).
        ds_tot:
            Total travel distance.
        dt_acc:
            Acceleration time.

        A motion profile can be defined in four different ways:
        1.  Either `v_m`, `a_m` (or `dt_acc`), and `ds_tot` are specified (other
            parameters left to `None`);
        2.  Either `v_m`, `a_m` (or `dt_acc`), and `dt_tot` are specified;
        3.  Either `dt_acc`, `dt_tot`, and `ds_tot` are specified;
        4.  Or only `ds_tot` and `dt_tot` are specified.

        In the 1st case, the total travel time can be determined. In the 2nd
        case, the total travel distance can be determined. In the 3rd case,
        the required acceleration can be determined. In the 4th case, the
        minimum possible acceleration can be determined (motion profile without
        a constant-velocity phase, i.e. a triangular velocity profile).
        """
        self.v_m = v_m
        self.a_m = a_m
        self.ds_tot = ds_tot
        self.dt_tot = dt_tot
        self.ds_acc = None
        self.dt_acc = dt_acc
        self.ds_dec = None
        self.dt_dec = None
        self.ds_cov = None
        self.dt_cov = None
        
        # Given: v_m and dt_acc -> determine a_m 
        if (self.a_m is None) and (self.v_m is not None) and (self.dt_acc is not None):
            self.a_m = self._calc_accel()
        
        # Given: a_m -> determine dt_acc
        if (self.dt_acc is None) and (self.a_m is not None):
            self.dt_acc = self._calc_accel_duration()

        # Use case 1 - Determine total travel time.
        # Given: v_m, a_m, and ds_tot.
        if all([self.v_m, self.a_m, self.ds_tot]):
            self._calc_total_travel_time()
        
        # Use case 2 - Determine total travel distance.
        # Given: v_m, a_m, and dt_tot. 
        elif all([self.v_m, self.a_m, self.dt_tot]):
            self._calc_total_travel_distance()
        
        # Use case 3 - Determine required acceleration.
        elif all([self.dt_acc, self.dt_tot, self.ds_tot]):
            self._calc_required_acceleration()
        
        # Use case 4 - Determine minimum acceleration and corresponding top 
        # velocity. Given: ds_tot, dt_tot (no constant velocity region)
        elif all([self.dt_tot, self.ds_tot]):
            self._calc_minimum_acceleration()
        
        else:
            raise ConfigError(
                "Cannot create the motion profile. Parameters are missing or "
                "wrong parameters are given."
            )
    
    @abstractmethod
    def _accel_fun(self, t: float) -> float:
        ...

    @abstractmethod
    def _decel_fun(self, t: float) -> float:
        ...

    @abstractmethod
    def _calc_accel_duration(self) -> float:
        ...

    @abstractmethod
    def _calc_decel_duration(self) -> float:
        ...

    @abstractmethod
    def _calc_accel(self) -> float:
        ...

    def _calc_total_travel_time(self) -> None:
        # acceleration phase
        self.ds_acc = self._calc_accel_distance()
        
        # deceleration phase
        self.dt_dec = self._calc_decel_duration()
        self.ds_dec = self.ds_acc
        
        # constant velocity phase
        self.ds_cov = self.ds_tot - self.ds_acc - self.ds_dec
        if self.ds_cov < 0:
            raise DistanceError(
                "Not enough travel distance available for acceleration "
                "and deceleration."
            )
        self.dt_cov = self._calc_cst_veloc_duration()
        
        # total travel time
        self.dt_tot = self.dt_acc + self.dt_cov + self.dt_dec

    def _calc_total_travel_distance(self) -> None:
        # acceleration phase
        self.ds_acc = self._calc_accel_distance()
        
        # deceleration phase
        self.dt_dec = self.dt_acc
        self.ds_dec = self.ds_acc
        
        # constant velocity phase
        self.dt_cov = self.dt_tot - self.dt_acc - self.dt_dec
        if self.dt_cov < 0:
            raise TimingError(
                "Not enough travel time available for acceleration and "
                "deceleration."
            )
        self.ds_cov = self._calc_cst_veloc_distance()
        
        # total travel distance
        self.ds_tot = self.ds_acc + self.ds_cov + self.ds_dec

    def _calc_minimum_acceleration(self) -> None:
        self.v_m = 2 * self.ds_tot / self.dt_tot
        self.a_m = 2 * self.v_m / self.dt_tot
        
        # acceleration phase
        self.ds_acc = self._calc_accel_distance()
        
        # deceleration phase
        self.dt_dec = self.dt_acc
        self.ds_dec = self.ds_acc
        
        # constant velocity phase
        self.dt_cov = 0.0
        self.ds_cov = 0.0
    
    def _calc_required_acceleration(self):
        self.dt_dec = self.dt_acc
        self.dt_cov = self.dt_tot - self.dt_acc - self.dt_dec
        self.v_m = self.ds_tot / (self.dt_cov + self.dt_acc)
        self.ds_cov = self.v_m * self.dt_cov
        self.ds_acc = (self.ds_tot - self.ds_cov) / 2
        self.ds_dec = self.ds_acc
        self.a_m = self._calc_accel()

    def _calc_accel_distance(self) -> float:
        t0 = 0.0
        t1 = t0 + self.dt_acc
        v0 = 0.0
        s0 = 0.0
        _, s, _ = position(t1, self._accel_fun, t0, v0, s0)
        s1 = float(s[-1])
        ds_acc = s1 - s0
        return ds_acc

    def _calc_cst_veloc_duration(self) -> float:
        dt_cov = self.ds_cov / self.v_m
        return dt_cov

    def _calc_cst_veloc_distance(self) -> float:
        ds_cov = self.v_m * self.dt_cov
        return ds_cov
        
    def velocity_profile(self) -> tuple[np.ndarray, np.ndarray]:
        """Calculates the velocity profile.

        Returns
        -------
        A tuple with two Numpy arrays. The first array are time values.
        The second array are the corresponding values of velocity.
        """
        # acceleration phase
        t0, v0 = 0.0, 0.0
        t1 = t0 + self.dt_acc
        t1_arr, v1_arr = velocity(t1, self._accel_fun, t0, v0)
        
        # constant velocity phase
        t1, v1 = float(t1_arr[-1]), float(v1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr, v2_arr = velocity(t2, lambda t: 0.0, t1, v1)
            t2, v2 = float(t2_arr[-1]), float(v2_arr[-1])
        else:
            t2_arr, v2_arr = None, None
            t2, v2 = float(t1_arr[-1]), float(v1_arr[-1])
        
        # deceleration phase
        t3 = t2 + self.dt_dec
        t3_arr, v3_arr = velocity(t3, self._decel_fun, t2, v2)
        
        if t2_arr is None:
            t_arr = np.concatenate((t1_arr, t3_arr))
        else:
            t_arr = np.concatenate((t1_arr, t2_arr, t3_arr))
        
        if v2_arr is None:
            v_arr = np.concatenate((v1_arr, v3_arr))
        else:
            v_arr = np.concatenate((v1_arr, v2_arr, v3_arr))
        
        return t_arr, v_arr

    def position_profile(self) -> tuple[np.ndarray, np.ndarray]:
        """Calculates the position profile.

        Returns
        -------
        A tuple with two Numpy arrays. The first array are the time values.
        The second array are the corresponding values of position.
        """
        # acceleration phase
        t0, v0, s0 = 0.0, 0.0, 0.0
        t1 = t0 + self.dt_acc
        t1_arr, s1_arr, v1_arr = position(t1, self._accel_fun, t0, v0, s0)
        
        # constant velocity phase
        t1, v1, s1 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr, s2_arr, v2_arr = position(t2, lambda t: 0.0, t1, v1, s1)
            t2, v2, s2 = float(t2_arr[-1]), float(v2_arr[-1]), float(s2_arr[-1])
        else:
            t2_arr, s2_arr, v2_arr = None, None, None
            t2, v2, s2 = float(t1_arr[-1]), float(v1_arr[-1]), float(s1_arr[-1])
        
        # deceleration phase
        t3 = t2 + self.dt_dec
        t3_arr, s3_arr, _ = position(t3, self._decel_fun, t2, v2, s2)
        
        if t2_arr is None:
            t_arr = np.concatenate((t1_arr, t3_arr))
        else:
            t_arr = np.concatenate((t1_arr, t2_arr, t3_arr))
        
        if s2_arr is None:
            s_arr = np.concatenate((s1_arr, s3_arr))
        else:
            s_arr = np.concatenate((s1_arr, s2_arr, s3_arr))
        
        return t_arr, s_arr

    def acceleration_profile(self) -> tuple[np.ndarray, np.ndarray]:
        """Calculates the acceleration profile.

        Returns
        -------
        A tuple with two Numpy arrays. The first array are the time values.
        The second array are the corresponding values of acceleration.
        """
        # acceleration phase
        t0, a0 = 0.0, 0.0
        t1 = t0 + self.dt_acc
        t1_arr = np.linspace(t0, t1, endpoint=True)
        a1_arr = np.array([self._accel_fun(t) for t in t1_arr])
        
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
        
        # deceleration phase
        t3 = t2 + self.dt_dec
        t3_arr = np.linspace(t2, t3, endpoint=True)
        a3_arr = np.array([self._decel_fun(t) for t in t3_arr])
        
        if t2_arr is None:
            t_arr = np.concatenate((t1_arr, t3_arr))
        else:
            t_arr = np.concatenate((t1_arr, t2_arr, t3_arr))
            
        if a2_arr is None:
            a_arr = np.concatenate((a1_arr, a2_arr, a3_arr))
        else:
            a_arr = np.concatenate((a1_arr, a2_arr, a3_arr))
            
        return t_arr, a_arr

    def get_fn_velocity_from_time(
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

    def get_fn_position_from_time(
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
                # noinspection PyTypeChecker
                return s_ax[-1]
            return s

        return f
    
    def get_fn_time_from_position(
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
                # noinspection PyTypeChecker
                return t_ax[-1]
            return t
        
        return f
    
    def get_accel_fn_velocity_from_time(self) -> Callable[[float], float]:
        """Returns a function that takes a time moment `t` in seconds and 
        returns the velocity `v` at that moment during the acceleration phase
        of the movement (`0 <= t <= dt_acc`).
        If `t > dt_acc`, the velocity at `dt_acc` is returned, i.e. also the
        velocity during the constant-velocity phase.
        """
        t0, v0 = 0.0, 0.0
        t1 = t0 + self.dt_acc
        t_arr, v_arr = velocity(t1, self._accel_fun, t0, v0)
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

    def get_decel_fn_velocity_from_time(
        self, 
        t0: float, 
        v0: float
    ) -> Callable[[float], float]:
        """Returns a function that takes a time moment `t` in seconds and 
        returns the velocity `v` at that moment during the deceleration phase
        of the movement (`t0 <= t <= t0 + dt_dec`).
        
        Parameters
        ----------
        t0 : float
            Time moment the deceleration phase begins.
        v0: float
            Initial velocity at the start of the deceleration phase.
        """
        t1 = t0 + self.dt_dec
        t_arr, v_arr = velocity(t1, self._decel_fun, t0, v0)

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
    
    def get_accel_fn_time_from_position(self) -> Callable[[float], float]:
        """Returns a function that takes a position `s` and returns the time 
        moment `t` in seconds when this position is reached.
        """
        t0, v0, s0 = 0.0, 0.0, 0.0
        t1 = t0 + self.dt_acc
        t_arr, s_arr, v_arr = position(t1, self._accel_fun, t0, v0, s0)
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
    
    def get_accel_fn_time_from_velocity(self) -> Callable[[float], float]:
        """Returns a function that takes a velocity `v` and returns the 
        time moment `t` in seconds when this velocity is reached.
        """
        t0, v0, s0 = 0.0, 0.0, 0.0
        t1 = t0 + self.dt_acc
        t_arr, s_arr, v_arr = position(t1, self._accel_fun, t0, v0, s0)
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
    
    def get_decel_fn_time_from_position(
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
            Time moment the deceleration phase begins.
        s0 : float
            Initial position at the start of the deceleration phase.
        v0: float
            Initial velocity at the start of the deceleration phase.
        """
        t1 = t0 + self.dt_dec
        t_arr, v_arr = velocity(t1, self._decel_fun, t0, v0)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions. 
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = scipy.interpolate.interp1d(t_arr, v_arr)
            sol = scipy.optimize.root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self._decel_fun, t0, v0, s0)
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

    def get_decel_fn_time_from_velocity(
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
            Time moment the deceleration phase begins.
        s0 : float
            Initial position at the start of the deceleration phase.
        v0: float
            Initial velocity at the start of the deceleration phase.
        """
        t1 = t0 + self.dt_dec
        t_arr, v_arr = velocity(t1, self._decel_fun, t0, v0)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions. 
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = scipy.interpolate.interp1d(t_arr, v_arr)
            sol = scipy.optimize.root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self._decel_fun, t0, v0, s0)
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
    def _accel_fun(self, t: float) -> float:
        return self.a_m

    def _decel_fun(self, t: float) -> float:
        return -self.a_m

    def _calc_accel_duration(self) -> float:
        dt_acc = self.v_m / self.a_m
        return dt_acc

    def _calc_decel_duration(self) -> float:
        return self.dt_acc

    def _calc_accel(self) -> float:
        return self.v_m / self.dt_acc


class SCurvedProfile(MotionProfile):
    """
    Use this class to define a pure S-curve motion profile.
    """
    def _accel_fun(self, t: float) -> float:
        c1 = self.a_m ** 2 / self.v_m
        t0 = 0.0
        t1 = t0 + self.dt_acc / 2
        t2 = t1 + self.dt_acc / 2
        if t0 <= t <= t1:
            return c1 * t
        elif t1 < t <= t2:
            return -c1 * (t - t2)
        else:
            return 0.0

    def _decel_fun(self, t: float) -> float:
        c1 = self.a_m ** 2 / self.v_m
        t0 = self.dt_acc + self.dt_cov
        t1 = t0 + self.dt_dec / 2
        t2 = t1 + self.dt_dec / 2
        if t0 <= t <= t1:
            return -c1 * (t - t0)
        elif t1 < t <= t2:
            return c1 * (t - t2)
        else:
            return 0.0

    def _calc_accel_duration(self) -> float:
        dt_acc = 2 * self.v_m / self.a_m
        return dt_acc

    def _calc_decel_duration(self) -> float:
        return self.dt_acc

    def _calc_accel(self) -> float:
        return 2 * self.v_m / self.dt_acc
