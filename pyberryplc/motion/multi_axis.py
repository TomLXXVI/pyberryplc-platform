from typing import Callable
from abc import ABC, abstractmethod
from enum import StrEnum

from numpy.typing import NDArray
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import root_scalar, minimize
from scipy.interpolate import interp1d

from pyberryplc.charts import LineChart


def velocity(
    t: float,
    accel_fn: Callable[[float, float, float, float], float],
    t0: float = 0.0,
    v0: float = 0.0,
    dv: float = 0.0,
    a_top: float = 0.0
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
    a_top:
        Maximum value of the acceleration profile.

    Returns
    -------
    t_arr:
        Numpy array with time values between `t0` and `t`.
    v_arr:
        Numpy array with the velocity values that correspond with the time
        values in `t_arr`.
    """
    # noinspection PyUnusedLocal
    def fn(t: float, v: NDArray[np.float64], a_top: float, dv: float, t0: float) -> NDArray[np.float64]:
        v_dot = np.zeros(1)
        v_dot[0] = accel_fn(t, a_top, dv, t0)
        return v_dot

    sol = solve_ivp(
        fn, (t0, t), [v0], 
        method='LSODA', 
        t_eval=np.linspace(t0, t, 100, endpoint=True),
        args=(a_top, dv, t0)
    )
    # noinspection PyUnresolvedReferences
    return sol.t, sol.y[0]


def position(
    t: float,
    accel_fn: Callable[[float, float, float, float], float],
    t0: float = 0.0,
    v0: float = 0.0,
    s0: float = 0.0,
    dv: float = 0.0,
    a_top: float = 0.0
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
        Position at the initial time moment.
    dv: float:
        Speed change during the movement. This helps to determine whether
        acceleration or deceleration is wanted.
    a_top:
        Maximum value of the acceleration profile.
    
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

    def fn(t: float, s: NDArray[np.float64], a_top: float, dv: float, t0: float) -> NDArray[np.float64]:
        s_dot = np.zeros(2)
        s_dot[0] = s[1]
        s_dot[1] = accel_fn(t, a_top, dv, t0)
        return s_dot

    sol = solve_ivp(
        fn, (t0, t), [s0, v0], 
        method='LSODA', 
        t_eval=np.linspace(t0, t, endpoint=True),
        args=(a_top, dv, t0)
    )
    # noinspection PyUnresolvedReferences
    return sol.t, sol.y[0], sol.y[1]


class MotionProfile(ABC):
    
    def __init__(
        self, 
        ds_tot: float,
        a_m: float,
        v_m: float,
        v_i: float | None = None,
        v_f: float | None = None,
        s_i: float = 0.0,
        dt_tot: float | None = None
    ) -> None:
        # Given by the user:
        self.v_m = v_m
        self.a_m = a_m
        self.ds_tot = ds_tot
        self.s_i = s_i
        self.v_i = v_i
        self.v_f = v_f
        
        # To be calculated:
        self.v_top: float = 0.0
        self.a_top: float = 0.0
        self.dt_i: float = 0.0
        self.dt_f: float = 0.0
        self.dv_i: float = 0.0
        self.dv_f: float = 0.0
        self.ds_i: float = 0.0
        self.ds_f: float = 0.0
        self.dt_cov: float = 0.0
        self.ds_cov: float = 0.0
        self.dt_tot: float = 0.0

        if ds_tot > 0.0:
            if dt_tot is None:
                # method 1: without time constraint
                v_top = v_m
                if v_f and v_i is not None:
                    dv = v_f - v_i
                    if dv > 0.0: v_top = v_f
                    if dv < 0.0: v_top = v_i
                self.a_top = self.a_m
                self.v_top = self._calc_v_top1(v_top)
            else:
                # method 2: with time constraint (for time synchronisation of axes)
                # self.a_top = self.a_m
                # self.v_top = self._calc_v_top2(dt_tot, self.a_m)
                self.a_top, self.v_top = self._calc_v_top3(dt_tot)
    
            self.dv_i = self._calc_dv_i(self.v_top)
            self.dt_i = self.get_dt_acc(self.dv_i, self.a_top)
            self.ds_i = self._calc_ds_i(self.a_top, self.dv_i, self.dt_i)
            self.dv_f = self._calc_dv_f(self.v_top)
            self.dt_f = self.get_dt_acc(self.dv_f, self.a_top)
            self.ds_f = self._calc_ds_f(self.v_top, self.a_top, self.dv_f, self.dt_f)
            self.dt_cov = self._calc_dt_cov(self.v_top, self.a_top)
            self.ds_cov = self._calc_ds_cov(self.v_top, self.a_top)
            self.dt_tot = self._calc_dt_tot(self.v_top, self.a_top)
    
    def _calc_dv_i(self, v_top: float) -> float:
        if self.ds_tot == 0.0: return 0.0
        return round(v_top - self.v_i, 12) if self.v_i is not None else 0.0
    
    def _calc_dv_f(self, v_top: float) -> float:
        if self.ds_tot == 0.0: return 0.0
        return round(self.v_f - v_top, 12) if self.v_f is not None else 0.0 
    
    def _calc_ds_i(
        self, 
        a_top: float, 
        dv_i: float,
        dt_i: float
    ) -> float:
        if self.ds_tot == 0.0: return 0.0
        if dt_i >= 0.0:
            if dt_i == 0.0: return 0.0
            t_arr, s_arr, _ = position(
                dt_i, self.accel_fn,
                t0=0.0,
                v0=self.v_i,
                s0=0.0,
                dv=dv_i,
                a_top=a_top
            )
            return float(s_arr[-1])
        raise ValueError("ds_i cannot be determined.")
    
    def _calc_ds_f(
        self, 
        v_top: float, 
        a_top: float, 
        dv_f: float,
        dt_f: float
    ) -> float:
        if self.ds_tot == 0.0: return 0.0
        if dt_f >= 0.0:
            if dt_f == 0.0: return 0.0
            t_arr, s_arr, _ = position(
                dt_f, self.accel_fn,
                t0=0.0,
                v0=v_top,
                s0=0.0,
                dv=dv_f,
                a_top=a_top
            )
            return float(s_arr[-1])
        raise ValueError("ds_f cannot be determined.")
    
    def _calc_ds_cov(self, v_top: float, a_top: float) -> float:
        if self.ds_tot == 0.0: return 0.0
        dv_i = self._calc_dv_i(v_top)
        dt_i = self.get_dt_acc(dv_i, a_top)
        ds_i = self._calc_ds_i(a_top, dv_i, dt_i)
        dv_f = self._calc_dv_f(v_top)
        dt_f = self.get_dt_acc(dv_f, a_top)
        ds_f = self._calc_ds_f(v_top, a_top, dv_f, dt_f)
        return self.ds_tot - (ds_i + ds_f)
    
    def _calc_dt_cov(self, v_top: float, a_top: float) -> float:
        if self.ds_tot == 0.0: return 0.0
        ds_cov = self._calc_ds_cov(v_top, a_top)
        return ds_cov / v_top
    
    def _calc_dt_tot(self, v_top: float, a_top: float) -> float:
        dv_i = self._calc_dv_i(v_top)
        dt_i = self.get_dt_acc(dv_i, a_top)
        dv_f = self._calc_dv_f(v_top)
        dt_f = self.get_dt_acc(dv_f, a_top)
        dt_cov = self._calc_dt_cov(v_top, a_top)
        return dt_i + dt_f + dt_cov
    
    def _calc_v_top1(self, v_top_ini: float) -> float:
        """
        Searches for the top velocity for which `ds_cov` equals zero (i.e. a
        movement without constant-velocity phase).
        """
        def _search_top_velocity1(a_top: float) -> float:
            def fn(v: float) -> float:
                ds_cov = self._calc_ds_cov(v, a_top)
                return ds_cov

            try:
                sol = root_scalar(fn, bracket=[1.e-12, self.v_m])
                return sol.root
            except ValueError:
                raise ValueError("Could not find a suitable top velocity.")
        
        if self.ds_tot == 0.0: return 0.0
        ds_cov = self._calc_ds_cov(v_top_ini, self.a_top)
        if ds_cov < 0.0:
            v_top = _search_top_velocity1(self.a_top)
            return v_top
        return v_top_ini
    
    def _calc_v_top2(self, dt_tot: float, a_top: float) -> float:
        """
        Searches the top velocity `v_top` of the movement for which the required
        total travel distance `self.ds_tot` will be finished after `dt_tot` 
        seconds when the acceleration is `a_top`.
        """
        dev = np.nan

        def fn(v_top: float) -> float:
            nonlocal dev
            # Determine the required velocity change in the initial and final
            # acceleration phase of the movement.
            dv_i = v_top - self.v_i if self.v_i is not None else 0.0
            dv_f = self.v_f - v_top if self.v_f is not None else 0.0
            # Based on the value `a_top` for the top acceleration, get the 
            # required acceleration time in the initial and final acceleration
            # phase of the movement.
            dt_i = self.get_dt_acc(dv_i, a_top)
            dt_f = self.get_dt_acc(dv_f, a_top)
            # Determine the remaining time in the constant-velocity phase
            # of the movement.
            dt_cov = dt_tot - (dt_i + dt_f)
            # A negative time duration of the constant-velocity phase, means
            # that the required time duration of the initial and final 
            # acceleration phase exceeds the available total travel time `dt_tot`
            # and so the current value of `v_top` is unsuitable. 
            if dt_cov < 0.0:
                if np.isnan(dev): return dev  # --> will cause an exception
                return np.inf if dev < 0.0 else -np.inf
            # Find the travelled distance in the constant-velocity phase, the
            # initial acceleration phase and final acceleration phase.
            ds_cov = v_top * dt_cov
            ds_i = self._calc_ds_i(a_top, dv_i, dt_i)
            ds_f = self._calc_ds_f(v_top, a_top, dv_f, dt_f)
            # Determine the total travel distance of the movement with the 
            # current value for `v_top`.
            ds_tot = ds_i + ds_cov + ds_f
            # Determine the deviation with the required total travel distance.
            dev = self.ds_tot - ds_tot
            return dev

        try:
            # Searches the top velocity `v_top` for which the required total
            # travel distance `self.ds_tot` is finished within the available
            # time `dt_tot` with acceleration `a_top`.
            sol = root_scalar(fn, bracket=[1.e-12, self.v_m], method='brentq')
            return sol.root
        except ValueError:
            raise ValueError("Could not find a suitable top velocity.")

    def _calc_v_top3(
        self,
        dt_tot: float
    ) -> tuple[float, float]:
        """
        Searches the top velocity `v_top` of the movement for which the required
        total travel distance `self.ds_tot` will be finished after `dt_tot` 
        seconds.
        """
        def fn(x: NDArray[np.float64]) -> float:
            try:
                v_top, a_top = float(x[0]), float(x[1])
                dv_i = self._calc_dv_i(v_top)
                dv_f = self._calc_dv_f(v_top)
                
                dt_i = self.get_dt_acc(dv_i, a_top)
                dt_f = self.get_dt_acc(dv_f, a_top)
                dt_cov = dt_tot - (dt_i + dt_f)
                if dt_cov < 0.0: raise ValueError
         
                ds_i = self._calc_ds_i(a_top, dv_i, dt_i)
                ds_f = self._calc_ds_f(v_top, a_top, dv_f, dt_f)
                ds_cov1 = self.ds_tot - (ds_i + ds_f)
                if ds_cov1 < 0.0: raise ValueError
         
                # the constant-velocity travel distance must also satisfy: 
                ds_cov2 = v_top * dt_cov
                dev = ds_cov2 - ds_cov1
                return dev ** 2
            except ValueError:
                return 1e10
        
        v_guess = self.v_m / 2
        a_guess = 2 * self.v_m
        initial_guess = np.array([v_guess, a_guess])
        bounds = [(0.0, self.v_m), (0.0, self.a_m)]
        result = minimize(fn, initial_guess, bounds=bounds, method="Nelder-Mead")
        if result.success:
            v_top, a_top = result.x[0], result.x[1]
            return a_top, v_top
        raise ValueError("Could not find suitable values for top velocity and top acceleration.")
    
    @staticmethod
    @abstractmethod
    def accel_fn(t: float, a_top: float, dv: float, t0: float = 0.0):
        ...
    
    @staticmethod
    @abstractmethod
    def get_dt_acc(dv: float, a_top: float) -> float:
        ...
        
    @staticmethod
    @abstractmethod
    def get_dv(a_top: float, dt_acc: float) -> float:
        ...
    
    @staticmethod
    @abstractmethod
    def get_a_top(dv: float, dt_acc: float) -> float:
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
        t0, v0 = 0.0, self.v_i if self.v_i is not None else self.v_top
        t1 = t0 + self.dt_i
        if self.dt_i > 0.0:
            t1_arr, v1_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_i, self.a_top)
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
            t3_arr, v3_arr = velocity(t3, self.accel_fn, t2, v2, self.dv_f, self.a_top)
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
        t0, v0, s0 = 0.0, self.v_i if self.v_i is not None else self.v_top, self.s_i
        t1 = t0 + self.dt_i
        if self.dt_i > 0.0:
            t1_arr, s1_arr, v1_arr = position(t1, self.accel_fn, t0, v0, s0, self.dv_i, self.a_top)
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
            t3_arr, s3_arr, _ = position(t3, self.accel_fn, t2, v2, s2, self.dv_f, self.a_top)
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
        a1_arr = np.array([self.accel_fn(t, self.a_top, self.dv_i, t0) for t in t1_arr])

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
            a3_arr = np.array([self.accel_fn(t, self.a_top, self.dv_f, t2) for t in t3_arr])
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
    
    @staticmethod
    def plot_position_profiles(*mps: 'MotionProfile') -> LineChart:
        """
        Returns a `LineChart` with the position profiles of the motion profiles.
        Call `show()` on the `LineChart` object to display the plot. 
        """
        chart = LineChart()
        for i, mp in enumerate(mps):
            t_arr, s_arr = mp.position_profile()
            chart.add_xy_data(
                label=f"position, MP{i + 1}",
                x1_values=t_arr,
                y1_values=s_arr
            )
        chart.x1.add_title("time, s")
        chart.y1.add_title("position")
        chart.add_legend()
        return chart

    @staticmethod
    def plot_velocity_profiles(*mps: 'MotionProfile') -> LineChart:
        """
        Returns a `LineChart` with the velocity profiles of the motion profiles.
        Call `show()` on the `LineChart` object to display the plot. 
        """
        chart = LineChart()
        for i, mp in enumerate(mps):
            t_arr, v_arr = mp.velocity_profile()
            chart.add_xy_data(
                label=f"velocity, MP{i + 1}",
                x1_values=t_arr,
                y1_values=v_arr
            )
        chart.x1.add_title("time, s")
        chart.y1.add_title("velocity")
        chart.add_legend()
        return chart

    @staticmethod
    def plot_acceleration_profiles(*mps: 'MotionProfile') -> LineChart:
        """
        Returns a `LineChart` with the acceleration profiles of the motion 
        profiles. Call `show()` on the `LineChart` object to display the plot. 
        """
        chart = LineChart()
        for i, mp in enumerate(mps):
            t_arr, a_arr = mp.acceleration_profile()
            chart.add_xy_data(
                label=f"acceleration, MP{i + 1}",
                x1_values=t_arr,
                y1_values=a_arr
            )
        chart.x1.add_title("time, s")
        chart.y1.add_title("acceleration")
        chart.add_legend()
        return chart
    
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
        t_arr, v_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_i, self.a_top)
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
        t_arr, v_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_f, self.a_top)
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
        t_arr, s_arr, v_arr = position(t1, self.accel_fn, t0, v0, s0, self.dv_i, self.a_top)
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
        t_arr, v_arr = velocity(t1, self.accel_fn, t0, v0, self.dv_f, self.a_top)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions.
        if v_arr[-1] < 0.0:
            # Find time `t1` where `v = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = interp1d(t_arr, v_arr)
            sol = root_scalar(interp_t, bracket=[t_pos, t_neg])
            t1 = sol.root

        t_arr, s_arr, v_arr = position(t1, self.accel_fn, t0, v0, s0, self.a_top)
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
        
    @staticmethod
    def accel_fn(t: float, a_top: float, dv: float, t0: float = 0.0) -> float:
        if dv < 0.0:
            return -a_top
        elif dv > 0.0:
            return a_top
        else:
            return 0.0

    @staticmethod
    def get_dt_acc(dv: float, a_top: float) -> float:
        return round(abs(dv) / a_top, 12)
    
    @staticmethod
    def get_dv(a_top: float, dt_acc: float) -> float:
        return a_top * dt_acc
    
    @staticmethod
    def get_a_top(dv: float, dt_acc: float) -> float:
        return abs(dv) / dt_acc


class SCurvedProfile(MotionProfile):
    
    @staticmethod
    def accel_fn(t: float, a_top: float, dv: float, t0: float = 0.0) -> float:
        if dv != 0.0:
            k = a_top ** 2 / abs(dv)
            if dv < 0.0: k *= -1
            dt = 2 * abs(dv) / a_top
            t1 = t0 + dt / 2
            t2 = t0 + dt
            if t0 <= t < t1:
                return k * (t - t0)
            elif t1 <= t <= t2:
                return k * (t2 - t)
            else:
                return 0.0
        return 0.0

    @staticmethod
    def get_dt_acc(dv: float, a_top: float) -> float:
        return round(2 * abs(dv) / a_top, 12)

    @staticmethod
    def get_dv(a_top: float, dt_acc: float) -> float:
        return (a_top * dt_acc) / 2.0

    @staticmethod
    def get_a_top(dv: float, dt_acc: float) -> float:
        return 2.0 * abs(dv) / dt_acc


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
