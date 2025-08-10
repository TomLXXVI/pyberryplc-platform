import warnings
from typing import Callable
from abc import ABC, abstractmethod
from enum import StrEnum

import numpy as np
from scipy.optimize import root_scalar, minimize_scalar
from scipy.interpolate import interp1d

from pyberryplc.charts import LineChart

from .elementary import cubic as cj, parabolic as ca


class MotionProfile(ABC):
    
    def __init__(
        self,
        ds_tot: float,
        a_max: float,
        v_max: float,
        v_i: float | None = None,
        v_f: float | None = None,
        s_i: float = 0.0,
        dt_tot: float | None = None
    ) -> None:
        """
        Creates a `MotionProfile` object.

        Parameters
        ----------
        ds_tot:
            Total travel distance.
        a_max:
            (Maximum) acceleration (of the motor). This value remains unaltered
            in the calculation of the motion profile (i.e. `self.a_top` is
            always equal to `a_m`).
        v_max:
            (Maximum) speed limit (of the motor). `v_m` needs to be
            distinguished from the top velocity `self.v_top` of the motion
            profile. `v_m` poses a maximum limit when calculating `self.v_top`.
        v_i: 
            Start velocity of the movement, if known.
        v_f: optional
            End velocity of the movement, if known.
        s_i:
            Initial position of the axis or motor shaft.
        dt_tot: optional
            Total travel time. When total travel time is specified, a motion
            profile is calculated so that total travel distance `ds_tot` is 
            covered in `dt_tot` (seconds).

        Units can be chosen freely, but must be consistent. Usually time is 
        chosen to be expressed in seconds. The units of position (displacement),
        velocity, and acceleration have units of length in common. For example,
        if time is in seconds and position is in units of mm, then velocity must
        be in mm/s, and acceleration in mm/s².

        A motion profile can be calculated for two use cases:
        (1) Travel distance `ds_tot` is known, but travel time `dt_tot` is not
            (`dt_tot` is `None`).
            The top velocity `self.v_top` is calculated so that the total travel
            distance `ds_tot` is accomplished in the shortest possible travel
            time `dt_tot`, taking into account the maximum velocity `v_m`, the
            (maximum) acceleration `a_m`, and the initial and final boundary
            velocities `v_i` and `v_f`.
        (2) Travel distance `ds_tot` and travel time `dt_tot` are both known.
            The travel time `dt_tot` adds an extra constraint to the calculation
            of `self.v_top`.

        The top acceleration `self.a_top` used in the calculations is always
        equal to the (maximum) acceleration `a_m`.

        If the initial boundary velocity `v_i` is undetermined (`None`), there
        is no initial acceleration phase (no initial velocity change) and the
        initial velocity will be equal to the calculated top velocity `v_top`.
        The same also applies to the final boundary velocity `v_f`.

        If neither the initial, nor the final boundary velocity is determined
        (`v_i` and `v_f` are both `None`), the motion profile only has a
        constant-velocity phase where the velocity is taken to be equal to the
        value of `v_m`.
        """
        # Given by the user:
        self.v_max = v_max
        self.a_max = self.a_top = a_max
        self.ds_tot = ds_tot
        self.s_i = s_i
        self.v_i = v_i
        self.v_f = v_f
        self.dt_tot = dt_tot

        # To be calculated:
        self.v_top: float = 0.0
        self.dt_i: float = 0.0
        self.dt_f: float = 0.0
        self.dv_i: float = 0.0
        self.dv_f: float = 0.0
        self.ds_i: float = 0.0
        self.ds_f: float = 0.0
        self.ds_cov: float = 0.0
        self.dt_cov: float = 0.0

        self._calculate_motion_profile()

    # noinspection PyUnreachableCode
    def _calculate_motion_profile(self) -> None:
        if self.ds_tot == 0.0 and self.dt_tot is not None:
            self.dt_cov = self.dt_tot
            return

        if self.ds_tot > 0.0:
            if self.dt_tot is None:
                # Case 1: without time constraint.
                self.v_top = self._calc_v_top_without_time_constraint()
            else:
                # Case 2: with time constraint.
                self.v_top = self._calc_v_top_with_time_constraint(self.dt_tot)

            self.dv_i = self._calc_dv_i(self.v_top)
            self.dt_i = self._calc_dt_acc(self.dv_i, self.a_top)
            v_i = self.v_top if self.v_i is None else self.v_i
            self.ds_i = self._calc_ds_i(v_i, self.a_top, self.dv_i, self.dt_i)
            self.dv_f = self._calc_dv_f(self.v_top)
            self.dt_f = self._calc_dt_acc(self.dv_f, self.a_top)
            self.ds_f = self._calc_ds_f(self.v_top, self.a_top, self.dv_f, self.dt_f)
            self.dt_cov = self._calc_dt_cov(self.v_top, self.a_top)
            self.ds_cov = self._calc_ds_cov(self.v_top, self.a_top)
            self.dt_tot = self._calc_dt_tot(self.v_top, self.a_top)

    def _calc_dv_i(self, v_top: float) -> float:
        """
        Velocity change in initial acceleration phase.
        """
        if self.ds_tot == 0.0: return 0.0
        return v_top - self.v_i if self.v_i is not None else 0.0

    def _calc_dv_f(self, v_top: float) -> float:
        """
        Velocity change in final acceleration phase.
        """
        if self.ds_tot == 0.0: return 0.0
        return self.v_f - v_top if self.v_f is not None else 0.0

    @staticmethod
    @abstractmethod
    def _calc_dt_acc(dv: float, a_top: float) -> float:
        """
        Acceleration time (time duration of the initial/final acceleration phase).
        """
        ...
    
    @abstractmethod
    def _calc_ds_i(
        self,
        v_i: float,
        a_top: float,
        dv_i: float,
        dt_i: float
    ) -> float:
        """
        Displacement during initial acceleration phase.
        """
        ...
    
    @abstractmethod
    def _calc_ds_f(
        self,
        v_top: float,
        a_top: float,
        dv_f: float,
        dt_f: float
    ) -> float:
        """
        Displacement during final acceleration phase.
        """
        ...

    def _calc_ds_cov(self, v_top: float, a_top: float) -> float:
        """
        Displacement during constant-velocity phase.
        """
        if self.ds_tot == 0.0: return 0.0
        dv_i = self._calc_dv_i(v_top)
        dt_i = self._calc_dt_acc(dv_i, a_top)

        if self.v_i is None:
            v_i = v_top
        else:
            v_i = self.v_i
        ds_i = self._calc_ds_i(v_i, a_top, dv_i, dt_i)

        dv_f = self._calc_dv_f(v_top)
        dt_f = self._calc_dt_acc(dv_f, a_top)
        ds_f = self._calc_ds_f(v_top, a_top, dv_f, dt_f)
        return self.ds_tot - (ds_i + ds_f)

    def _calc_dt_cov(self, v_top: float, a_top: float) -> float:
        """
        Time duration of constant-velocity phase.
        """
        if self.ds_tot == 0.0: return 0.0
        ds_cov = self._calc_ds_cov(v_top, a_top)
        return ds_cov / v_top

    def _calc_dt_tot(self, v_top: float, a_top: float) -> float:
        """
        Total time duration of the movement (travel time).
        """
        dv_i = self._calc_dv_i(v_top)
        dt_i = self._calc_dt_acc(dv_i, a_top)
        dv_f = self._calc_dv_f(v_top)
        dt_f = self._calc_dt_acc(dv_f, a_top)
        dt_cov = self._calc_dt_cov(v_top, a_top)
        return dt_i + dt_f + dt_cov

    def _calc_v_top_without_time_constraint(self) -> float:
        """
        Calculates `v_top` so that `ds_cov` = 0 (i.e. without constant-velocity 
        phase).
        """
        def _search_top_velocity() -> float:

            def fn(v_top: float) -> float:
                ds_cov = self._calc_ds_cov(v_top, self.a_top)
                return ds_cov

            try:
                sol = root_scalar(fn, bracket=(1.e-12, self.v_max))
                return sol.root
            except ValueError:
                raise ValueError("Could not find a suitable top velocity.")

        if self.ds_tot == 0.0:
            return 0.0
        v_top_i = self.v_max
        if self.v_f and self.v_i is not None:
            dv = self.v_f - self.v_i
            if dv > 0.0:
                v_top_i = self.v_f
            if dv < 0.0:
                v_top_i = self.v_i
        ds_cov = self._calc_ds_cov(v_top_i, self.a_top)
        if ds_cov < 0.0:
            v_top = _search_top_velocity()
            return v_top
        return v_top_i

    def limit_dt_cov(self, dt_tot: float) -> float:
        # Find `v_top` for which `dt_cov` = 0.
        def _f(v_top):
            dv_i = self._calc_dv_i(v_top)
            dv_f = self._calc_dv_f(v_top)
            dt_i = self._calc_dt_acc(dv_i, self.a_top)
            dt_f = self._calc_dt_acc(dv_f, self.a_top)
            dt_cov = dt_tot - (dt_i + dt_f)
            return dt_cov

        try:
            sol = root_scalar(_f, bracket=(1.0, self.v_max))
            return sol.root
        except ValueError:
            warnings.warn(
                "`v_top` for which `dt_cov` = 0 cannot be determined.",
                category=UserWarning
            )
            return self.v_max

    def limit_ds_cov(self) -> float:
        # Find `v_top` for which `ds_cov` = 0.
        def _f(v_top):
            dv_i = self._calc_dv_i(v_top)
            dv_f = self._calc_dv_f(v_top)
            dt_i = self._calc_dt_acc(dv_i, self.a_max)
            dt_f = self._calc_dt_acc(dv_f, self.a_max)
            v_i = v_top if self.v_i is None else self.v_i
            ds_i = self._calc_ds_i(v_i, self.a_max, dv_i, dt_i)
            ds_f = self._calc_ds_f(v_top, self.a_max, dv_f, dt_f)
            ds_cov = self.ds_tot - (ds_i + ds_f)
            return ds_cov

        try:
            sol = root_scalar(_f, bracket=(1.e-12, self.v_max))
            return sol.root
        except ValueError:
            warnings.warn(
                "`v_top` for which `ds_cov` = 0 cannot be determined.",
                category=UserWarning
            )
            return self.v_max

    def _calc_v_top_with_time_constraint(self, dt_tot: float) -> float:
        """
        Calculates `v_top` so that `ds_tot` is finished after `dt_tot` s when 
        acceleration is `self.a_top`.
        """
        def objective(v_top: float) -> float:
            try:
                dv_i = self._calc_dv_i(v_top)
                dv_f = self._calc_dv_f(v_top)

                dt_i = self._calc_dt_acc(dv_i, self.a_top)
                dt_f = self._calc_dt_acc(dv_f, self.a_top)
                dt_cov = dt_tot - (dt_i + dt_f)
                if dt_cov < 0.0:
                    raise ValueError
                v_i = v_top if self.v_i is None else self.v_i
                ds_i = self._calc_ds_i(v_i, self.a_top, dv_i, dt_i)
                ds_f = self._calc_ds_f(v_top, self.a_top, dv_f, dt_f)
                ds_cov1 = self.ds_tot - (ds_i + ds_f)
                if ds_cov1 < 0.0:
                    raise ValueError
                # constant-velocity travel distance must also satisfy:
                ds_cov2 = v_top * dt_cov
                dev = ds_cov2 - ds_cov1
                return dev ** 2
            except ValueError:
                return 1e10

        # noinspection PyTypeChecker
        res = minimize_scalar(
            objective, 
            bounds=(1.e-12, self.v_max),
            method='bounded'
        )
        if res.success and abs(res.fun) <= 1e-6:
            return res.x
        raise ValueError("Could not find a suitable top velocity.")
        
    @abstractmethod
    def position(
        self, 
        t_arr: np.ndarray, 
        s0: float,
        v0: float,
        dv: float
    ) -> float | np.ndarray:
        """
        Returns position(s) at given time moment(s) during initial/final 
        acceleration phase.
        """
        ...

    @abstractmethod
    def velocity(
        self,
        t_arr: np.ndarray,
        v0: float,
        dv: float
    ) -> np.ndarray:
        """
        Returns velocities(s) at given time moment(s) during initial/final 
        acceleration phase.
        """
        ...

    @abstractmethod
    def acceleration(
        self, 
        t_arr: np.ndarray,
        dv: float
    ) -> np.ndarray:
        """
        Returns accelerations(s) at given time moment(s) during initial/final 
        acceleration phase.
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
        t0, v0 = 0.0, self.v_i if self.v_i is not None else self.v_top
        t1 = t0 + self.dt_i
        if self.dt_i > 0.0:
            t1_arr = np.linspace(t0, t1, 100)
            v1_arr = self.velocity(t1_arr, v0, self.dv_i)
        else:
            t1_arr, v1_arr = np.array([t0]), np.array([v0])

        # constant-velocity phase
        t1, v1 = float(t1_arr[-1]), float(v1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr = np.linspace(t1, t2, 100)
            v2_arr = np.full_like(t2_arr, self.v_top)
            t2, v2 = float(t2_arr[-1]), float(v2_arr[-1])
        else:
            t2_arr, v2_arr = None, None
            t2, v2 = float(t1_arr[-1]), float(v1_arr[-1])

        # final acceleration phase
        if self.dt_f > 0:
            t3 = t2 + self.dt_f
            t3_arr = np.linspace(t2, t3, 100)
            v3_arr = self.velocity(t3_arr, self.v_top, self.dv_f)
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
            t1_arr = np.linspace(t0, t1, 100)
            s1_arr = self.position(t1_arr, s0=self.s_i, v0=v0, dv=self.dv_i)
        else:
            t1_arr, s1_arr = np.array([t0]), np.array([s0])

        # constant-velocity phase
        t1, s1 = float(t1_arr[-1]), float(s1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr = np.linspace(t1, t2, 100)
            s2_arr = s1 + self.v_top * (t2_arr - t1)
            t2, s2 = float(t2_arr[-1]), float(s2_arr[-1])
        else:
            t2_arr, s2_arr, v2_arr = None, None, None
            t2, s2 = t1, s1

        # final acceleration phase
        if self.dt_f > 0.0:
            t3 = t2 + self.dt_f
            t3_arr = np.linspace(t2, t3, 100)
            s3_arr = self.position(t3_arr, s0=s2, v0=self.v_top, dv=self.dv_f)
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
        t1_arr = np.linspace(t0, t1, 100)
        a1_arr = self.acceleration(t1_arr, dv=self.dv_i)

        # constant-velocity phase
        t1, a1 = float(t1_arr[-1]), float(a1_arr[-1])
        t2 = t1 + self.dt_cov
        if t2 > t1:
            t2_arr = np.linspace(t1, t2, 100)
            a2_arr = np.full_like(t2_arr, 0.0)
            t2, a2 = float(t2_arr[-1]), float(a2_arr[-1])
        else:
            t2_arr = None
            a2_arr = None
            t2, a2 = float(t1_arr[-1]), float(a1_arr[-1])

        # final acceleration phase
        if self.dt_f > 0:
            t3 = t2 + self.dt_f
            t3_arr = np.linspace(t2, t3, 100)
            a3_arr = self.acceleration(t3_arr, dv=self.dv_f)
        else:
            t3_arr, a3_arr = None, None

        if t2_arr is None and t3_arr is not None:
            t_arr = np.concatenate((t1_arr, t3_arr))
            a_arr = np.concatenate((a1_arr, a3_arr))
        elif t2_arr is not None and t3_arr is None:
            t_arr = np.concatenate((t1_arr, t2_arr))
            a_arr = np.concatenate((a1_arr, a2_arr))
        elif t2_arr is None and t3_arr is None:
            t_arr = t1_arr
            a_arr = a1_arr
        else:
            t_arr = np.concatenate((t1_arr, t2_arr, t3_arr))
            a_arr = np.concatenate((a1_arr, a2_arr, a3_arr))

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

    def get_velocity_from_time_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a time moment `t` and returns the
        velocity `velocity` at that time moment (`0 <= t <= dt_tot`).
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

    def get_velocity_from_position_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a position `s` and returns the velocity
        `v` at that position (`0 <= s <= ds_tot`).
        """
        t_ax, s_ax = self.position_profile()
        v_ax = np.array(list(map(self.get_velocity_from_time_fn(), t_ax)))
        v_ax = np.clip(v_ax, 1e-12, None)
        interp = interp1d(s_ax, v_ax)

        def f(s: float) -> float:
            try:
                v = interp(s)
            except ValueError:
                v = v_ax[0]
                if s > s_ax[-1]:
                    v = v_ax[-1]
            return v

        return f

    def get_position_from_time_fn(self) -> Callable[[float], float]:
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

    def get_time_from_position_fn(self) -> Callable[[float], float]:
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

    def get_ini_velocity_from_time_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a time moment `t` in seconds and
        returns the velocity `v` at that moment during the initial 
        acceleration phase of the movement (`0 <= t <= dt_i`).

        If `t > dt_i`, the velocity at `dt_i` is returned (i.e. the top velocity
        during the constant-velocity phase).
        """
        t0, v0 = 0.0, 0.0
        t1 = t0 + self.dt_i
        t_arr = np.linspace(t0, t1, 100)
        v_arr = self.velocity(t_arr, v0=v0, dv=self.dv_i)
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

    def get_fin_velocity_from_time_fn(
        self,
        t0: float,
        v0: float
    ) -> Callable[[float], float]:
        """
        Returns a function that takes a time moment `t` in seconds and returns
        the velocity `velocity` at that moment during the final acceleration phase of
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
        t_arr = np.linspace(t0, t1, 100)
        v_arr = self.velocity(t_arr, v0=v0, dv=self.dv_f)
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

    def get_ini_time_from_position_fn(self) -> Callable[[float], float]:
        """
        Returns a function that takes a position `s` during the initial
        acceleration phase and returns the time moment `t` in seconds when this
        position is reached.

        If `s` is greater than the final acceleration position, the time moment
        is returned when the final acceleration position is reached.
        """
        t0, v0, s0 = 0.0, 0.0, 0.0
        t1 = t0 + self.dt_i
        t_arr = np.linspace(t0, t1, 100)
        s_arr = self.position(t_arr, s0=s0, v0=v0, dv=self.dv_i)
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

    def get_fin_time_from_position_fn(
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
        t_arr = np.linspace(t0, t1, 100)
        v_arr = self.velocity(t_arr, v0=v0, dv=self.dv_f)

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions.
        if v_arr[-1] < 0.0:
            # Find time `t1` where `velocity = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = interp1d(t_arr, v_arr)
            sol = root_scalar(interp_t, bracket=(t_pos, t_neg))
            t1 = sol.root

        t_arr = np.linspace(t0, t1, 100)
        s_arr = self.position(t_arr, s0=s0, v0=v0, dv=self.dv_f)
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
    def _calc_dt_acc(dv: float, a_top: float) -> float:
        """
        Required acceleration time for required velocity change `dv` when 
        acceleration is `a_top`.
        """
        return abs(dv / a_top)
    
    def _calc_ds_i(
        self,
        v_i: float,
        a_top: float,
        dv_i: float,
        dt_i: float
    ) -> float:
        """
        Displacement during initial acceleration phase.
        """
        if self.ds_tot == 0.0: return 0.0
        if dt_i >= 0.0:
            if dt_i == 0.0: return 0.0
            if dv_i < 0.0: a_top *= -1
            ds_i = ca.position(dt_i, t0=0.0, s0=0.0, v0=v_i, a0=a_top)
            return abs(ds_i)
        raise ValueError("ds_i cannot be determined.")
    
    def _calc_ds_f(
        self,
        v_top: float,
        a_top: float,
        dv_f: float,
        dt_f: float
    ) -> float:
        """
        Displacement during final acceleration phase.
        """
        if self.ds_tot == 0.0: return 0.0
        if dt_f >= 0.0:
            if dt_f == 0.0: return 0.0
            if dv_f < 0.0: a_top *= -1
            ds_f = ca.position(dt_f, t0=0.0, s0=0.0, v0=v_top, a0=a_top)
            return abs(ds_f)
        raise ValueError("ds_f cannot be determined.")
    
    def position(
        self, 
        t_arr: np.ndarray, 
        s0: float,
        v0: float,
        dv: float
    ) -> np.ndarray:
        if dv < 0.0:
            a_top = -self.a_top
        else:
            a_top = self.a_top
        s_arr = ca.position(t_arr, t0=t_arr[0], s0=s0, v0=v0, a0=a_top)
        return s_arr
    
    def velocity(
        self, 
        t_arr: np.ndarray, 
        v0: float,
        dv: float
    ) -> np.ndarray:
        if dv < 0.0:
            a_top = -self.a_top
        else:
            a_top = self.a_top
        v_arr = ca.velocity(t_arr, t0=t_arr[0], v0=v0, a0=a_top)
        return v_arr
    
    def acceleration(
        self, 
        t_arr: np.ndarray,
        dv: float
    ) -> np.ndarray:
        if dv == 0.0:
            a_arr = np.full_like(t_arr, 0.0)
            return a_arr
        else:
            if dv < 0.0:
                a_top = -self.a_top
            else:
                a_top = self.a_top
            a_arr = ca.acceleration(t_arr, a0=a_top)
            return a_arr


class SCurvedProfile(MotionProfile):

    @staticmethod
    def _calc_dt_acc(dv: float, a_top: float) -> float:
        """
        Required acceleration time for required velocity change `dv` when 
        acceleration is `a_top`.
        """
        return abs(2 * dv / a_top)
    
    def _calc_ds_i(
        self,
        v_i: float,
        a_top: float,
        dv_i: float,
        dt_i: float
    ) -> float:
        """
        Displacement during initial acceleration phase.
        """
        if self.ds_tot == 0.0: return 0.0
        if dt_i >= 0.0:
            if dt_i == 0.0: return 0.0
            j1 = cj.get_constant_jerk(a_top, dv_i / 2)
            j2 = -j1
            s1 = cj.position(dt_i / 2, j0=j1, v0=v_i)
            v1 = cj.velocity(dt_i / 2, j0=j1, v0=v_i)
            a1 = cj.acceleration(dt_i / 2, j0=j1)
            s2 = cj.position(dt_i, t0=dt_i / 2, s0=s1, v0=v1, a0=a1, j0=j2)
            ds_i = s2 - 0.0
            return abs(ds_i)
        raise ValueError("ds_i cannot be determined.")
    
    def _calc_ds_f(
        self,
        v_top: float,
        a_top: float,
        dv_f: float,
        dt_f: float
    ) -> float:
        """
        Displacement during final acceleration phase.
        """
        if self.ds_tot == 0.0: return 0.0
        if dt_f >= 0.0:
            if dt_f == 0.0: return 0.0
            j1 = cj.get_constant_jerk(a_top, dv_f / 2)
            j2 = -j1
            s1 = cj.position(dt_f / 2, j0=j1, v0=v_top)
            v1 = cj.velocity(dt_f / 2, j0=j1, v0=v_top)
            a1 = cj.acceleration(dt_f / 2, j0=j1)
            s2 = cj.position(dt_f, t0=dt_f / 2, s0=s1, v0=v1, a0=a1, j0=j2)
            ds_f = s2 - 0.0
            return abs(ds_f)
        raise ValueError("ds_f cannot be determined.")

    def position(
        self,
        t_arr: np.ndarray,
        s0: float,
        v0: float,
        dv: float
    ) -> np.ndarray:
        if dv < 0.0:
            a_top = -self.a_top
        else:
            a_top = self.a_top
        dt = t_arr[-1] - t_arr[0]
        j1 = cj.get_constant_jerk(a_top, dv / 2)
        j2 = -j1
        t1 = t_arr[0] + dt / 2  # flex point
        t1_arr = t_arr[t_arr < t1]
        t2_arr = t_arr[t_arr >= t1]
        s1 = cj.position(t1, t0=t_arr[0], s0=s0, v0=v0, a0=0.0, j0=j1)
        v1 = cj.velocity(t1, t0=t_arr[0], v0=v0, a0=0.0, j0=j1)
        s1_arr = cj.position(t1_arr, t0=t_arr[0], s0=s0, v0=v0, a0=0.0, j0=j1)
        s2_arr = cj.position(t2_arr, t0=t1, s0=s1, v0=v1, a0=a_top, j0=j2)
        s_arr = np.concatenate((s1_arr, s2_arr))
        return s_arr
        
    def velocity(
        self,
        t_arr: np.ndarray,
        v0: float,
        dv: float
    ) -> np.ndarray:
        if dv < 0.0:
            a_top = -self.a_top
        else:
            a_top = self.a_top
        dt = t_arr[-1] - t_arr[0]
        j1 = cj.get_constant_jerk(a_top, dv / 2)
        j2 = -j1
        t1 = t_arr[0] + dt / 2  # flex point
        t1_arr = t_arr[t_arr < t1]
        t2_arr = t_arr[t_arr >= t1]
        v1 = cj.velocity(t1, t0=t_arr[0], v0=v0, a0=0.0, j0=j1)
        v1_arr = cj.velocity(t1_arr, t0=t_arr[0], v0=v0, a0=0.0, j0=j1)
        v2_arr = cj.velocity(t2_arr, t0=t1, v0=v1, a0=a_top, j0=j2)
        v_arr = np.concatenate((v1_arr, v2_arr))
        return v_arr

    def acceleration(
        self,
        t_arr: np.ndarray,
        dv: float
    ) -> np.ndarray:
        if dv == 0.0:
            a_arr = np.full_like(t_arr, 0.0)
            return a_arr
        else:
            if dv < 0.0:
                a_top = -self.a_top
            else:
                a_top = self.a_top
            dt = t_arr[-1] - t_arr[0]
            j1 = cj.get_constant_jerk(a_top, dv / 2)
            j2 = -j1
            t1 = t_arr[0] + dt / 2  # flex point
            t1_arr = t_arr[t_arr < t1]
            t2_arr = t_arr[t_arr >= t1]
            a1_arr = cj.acceleration(t1_arr, t0=t_arr[0], a0=0.0, j0=j1)
            a2_arr = cj.acceleration(t2_arr, t0=t1, a0=a_top, j0=j2)
            a_arr = np.concatenate((a1_arr, a2_arr))
            return a_arr


class RotationDirection(StrEnum):
    CW = "clockwise"
    CCW = "counterclockwise"

    def to_bool(self) -> bool:
        """Returns True for counterclockwise, False for clockwise."""
        return self == RotationDirection.CCW

    def to_int(self) -> int:
        """Returns 1 for counterclockwise, -1 for clockwise."""
        if self == RotationDirection.CW:
            return -1
        return 1

    def __int__(self) -> int:
        return self.to_int()

    def __bool__(self) -> bool:
        raise TypeError("Use .to_bool() for explicit conversion.")

    def __invert__(self) -> 'RotationDirection':
        if self == RotationDirection.CCW:
            return RotationDirection.CW
        return RotationDirection.CCW

    def toggle(self) -> 'RotationDirection':
        return ~self
