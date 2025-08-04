from typing import Callable
from abc import ABC, abstractmethod
from enum import StrEnum

import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import root_scalar
from scipy.interpolate import interp1d

from pyberryplc.charts import LineChart


class MotionProfile(ABC):
    dt_acc_min: float = 0.1

    def __init__(
        self,
        ds_tot: float,
        a_max: float,
        v_max: float,
        v_i: float | None,
        v_f: float | None,
        s_i: float = 0.0,
        dt_tot: float | None = None
    ) -> None:
        self.ds_tot = ds_tot
        self.a_max = a_max
        self.v_max = v_max
        self.v_i = v_i
        self.v_f = v_f
        self.s_i = s_i
        self.dt_tot = dt_tot

        self.v_top = 0.0
        self.dv_i = 0.0
        self.dv_f = 0.0
        self.a_top_i = 0.0
        self.a_top_f = 0.0
        self.dt_i = 0.0
        self.dt_f = 0.0
        self.ds_i = 0.0
        self.ds_f = 0.0
        self.ds_cov = 0.0
        self.dt_cov = 0.0

        self._calc_motion_profile()

    @abstractmethod
    def get_dv_min(self) -> float:
        """
        Returns the smallest achievable velocity change with acceleration
        `self.a_max` where the acceleration time equals the minimum allowable
        acceleration time `self.dt_acc_min`.
        """
        ...

    @ abstractmethod
    def _calc_a_top(self, dv: float) -> tuple[float, float]:
        """
        Calculates the top acceleration of the motion and the accleration time.

        Parameters
        ----------
        dv :
            Required velocity change.

        Returns
        -------
        a_top :
            In case the acceleration time that follows from `dv` and `self.a_max`
            is smaller than the minimum permitted acceleration time
            `self.dt_acc_min`, the top acceleration `a_top` of the motion is
            calculated so that acceleration time would equal the minimum
            permitted acceleration time (`a_top` < `a_max`). Otherwise, `a_top`
            is kept equal to `self.a_max`.
        dt_acc :
            Resulting acceleration time.
        """
        ...

    @abstractmethod
    def acceleration(
        self,
        dv: float,
        t0: float = 0.0
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the acceleration values at given time moments during
        acceleration of the motion.

        Parameters
        ----------
        dv :
            Required velocity change.
        t0:
            Initial time moment of the acceleration.

        Returns
        -------
        t_arr :
            Numpy array with the time values during acceleration of the motion.
        a_arr:
            Numpy array with the corresponding values of the acceleration.
        """
        ...

    def velocity_acc(
        self,
        dv: float,
        t0: float = 0.0,
        v0: float = 0.0
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the velocity values at given time moments during
        acceleration of the motion.

        Parameters
        ----------
        dv :
            Required velocity change.
        t0 :
            Initial time moment.
        v0 :
            Initial velocity.

        Returns
        -------
        t_arr :
            Numpy array with the time values during acceleration of the motion.
        v_arr:
            Numpy array with the corresponding values of the velocity.
        """
        t_arr, a_arr = self.acceleration(dv, t0)
        v_arr = cumulative_trapezoid(a_arr, t_arr, initial=0)
        v_arr += v0
        return t_arr, v_arr

    def position_acc(
        self,
        dv: float,
        t0: float = 0.0,
        s0: float = 0.0,
        v0: float = 0.0
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates the position values at given time moments during
        acceleration of the motion.

        Parameters
        ----------
        dv :
            Required velocity change.
        t0 :
            Initial time moment.
        s0 :
            Initial position.
        v0 :
            Initial velocity.

        Returns
        -------
        t_arr :
            Numpy array with the time values during acceleration of the motion.
        s_arr:
            Numpy array with the corresponding values of the position.
        """
        t_arr, v_arr = self.velocity_acc(dv, t0, v0)
        s_arr = cumulative_trapezoid(v_arr, t_arr, initial=0)
        s_arr += s0
        return t_arr, s_arr

    def _calc_ds(self, dv: float):
        """
        Calculates the distance traveled during acceleration of the motion.

        Parameters
        ----------
        dv :
            Required velocity change.

        Returns
        -------
        ds :
            Distance traveled during acceleration of the motion.
        """
        _, s_arr = self.position_acc(dv)
        ds = s_arr[-1] - s_arr[0]
        return ds

    def _calc_dv_i(self, v_top: float) -> float:
        dv = v_top - self.v_i if self.v_i is not None else 0.0  # assumes self.v_i = v_top
        return dv

    def _calc_dv_f(self, v_top: float) -> float:
        dv = self.v_f - v_top if self.v_f is not None else 0.0  # assumes self.v_f = v_top
        return dv

    def _determine_v_top_without_time_constraint(self) -> float:

        def fn(v_top: float) -> float:
            dv_i = self._calc_dv_i(v_top)
            dv_f = self._calc_dv_f(v_top)
            ds_i = self._calc_ds(abs(dv_i))
            ds_f = self._calc_ds(abs(dv_f))
            ds_cov = self.ds_tot - (ds_i + ds_f)
            return ds_cov

        ds_cov = fn(self.v_max)
        if ds_cov > 0.0:
            return self.v_max

        try:
            sol = root_scalar(fn, bracket=(0.0, self.v_max))
            return sol.root
        except ValueError:
            raise ValueError("Top velocity cannot be determined.")

    def _determine_v_top_with_time_constraint(self) -> float:

        def fn(v_top: float) -> float:
            dv_i = self._calc_dv_i(v_top)
            dv_f = self._calc_dv_f(v_top)
            _, dt_i = self._calc_a_top(abs(dv_i))
            _, dt_f = self._calc_a_top(abs(dv_f))
            ds_i = self._calc_ds(abs(dv_i))
            ds_f = self._calc_ds(abs(dv_f))
            ds_cov1 = self.ds_tot - (ds_i + ds_f)
            dt_cov = self.dt_tot - (dt_i + dt_f)
            ds_cov2 = v_top * dt_cov
            dev = ds_cov2 - ds_cov1
            return dev

        try:
            sol = root_scalar(fn, bracket=(0.0, self.v_max))
            return sol.root
        except ValueError:
            raise ValueError("Top velocity cannot be determined.")

    def _calc_motion_profile(self) -> None:
        if self.ds_tot == 0.0 and self.dt_tot is not None:
            self.dt_cov = self.dt_tot
            return

        if self.ds_tot > 0.0 and self.dt_tot is None:
            self.v_top = self._determine_v_top_without_time_constraint()
        elif self.ds_tot > 0.0 and self.dt_tot is not None and self.dt_tot > 0.0:
            self.v_top = self._determine_v_top_without_time_constraint()
        else:
            raise ValueError("Motion profile cannot be determined.")

        self.dv_i = self._calc_dv_i(self.v_top)
        self.dv_f = self._calc_dv_f(self.v_top)
        self.a_top_i, self.dt_i = self._calc_a_top(abs(self.dv_i))
        self.a_top_f, self.dt_f = self._calc_a_top(abs(self.dv_f))
        self.ds_i = self._calc_ds(abs(self.dv_i))
        self.ds_f = self._calc_ds(abs(self.dv_f))
        self.ds_cov = self.ds_tot - (self.ds_i + self.ds_f)
        if self.ds_cov > 0.0:
            self.dt_cov = self.ds_cov / self.v_top
        self.dt_tot = self.dt_i + self.dt_cov + self.dt_f

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
        t0 = 0.0
        v0 = self.v_i if self.v_i is not None else self.v_top
        if self.dt_i > 0.0:
            t1_arr, v1_arr = self.velocity_acc(self.dv_i, v0=v0)
        else:
            t1_arr, v1_arr = np.array([t0]), np.array([v0])

        # constant-velocity phase
        t1 = float(t1_arr[-1])
        v1 = float(v1_arr[-1])
        if self.dt_cov > 0.0:
            t2 = t1 + self.dt_cov
            t2_arr = np.linspace(t1, t2, 100)
            v2_arr = np.full_like(t2_arr, self.v_top)
        else:
            t2_arr, v2_arr = None, None
            t2, v2 = t1, v1

        # final acceleration phase
        if self.dt_f > 0:
            t3_arr, v3_arr = self.velocity_acc(self.dv_f, t0=t2, v0=self.v_top)
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
        s0 = self.s_i
        v0 = self.v_i if self.v_i is not None else self.v_top
        if self.dt_i > 0.0:
            t1_arr, s1_arr = self.position_acc(self.dv_i, s0=s0, v0=v0)
        else:
            t1_arr, s1_arr = np.array([0.0]), np.array([s0])

        # constant-velocity phase
        t1, s1 = float(t1_arr[-1]), float(s1_arr[-1])
        if self.dt_cov > 0.0:
            t2 = t1 + self.dt_cov
            t2_arr = np.linspace(t1, t2, 100)
            s2_arr = s1 + self.v_top * (t2_arr - t1)
            s2 = float(s2_arr[-1])
        else:
            t2_arr, s2_arr, v2_arr = None, None, None
            t2, s2 = t1, s1

        # final acceleration phase
        if self.dt_f > 0.0:
            t3_arr, s3_arr = self.position_acc(self.dv_f, t0=t2, s0=s2, v0=self.v_top)
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
        t1_arr, a1_arr = self.acceleration(self.dv_i)

        # constant-velocity phase
        if self.dt_cov > 0.0:
            t1 = float(t1_arr[-1])
            t2 = t1 + self.dt_cov
            t2_arr = np.linspace(t1, t2, 100)
            a2_arr = np.full_like(t2_arr, 0.0)
        else:
            t2_arr = None
            a2_arr = None
            t2, a2 = float(t1_arr[-1]), float(a1_arr[-1])

        # final acceleration phase
        if self.dt_f > 0:
            t3_arr, a3_arr = self.acceleration(self.dv_f, t0=t2)
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

    def get_velocity_time_fn(self) -> Callable[[float], float]:
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
        returns the velocity `v` at that moment during the initial
        acceleration phase of the movement (`0 <= t <= dt_i`).

        If `t > dt_i`, the velocity at `dt_i` is returned (i.e. the top velocity
        during the constant-velocity phase).
        """
        t_arr, v_arr = self.velocity_acc(self.dv_i)
        t1 = t_arr[-1]
        v_max = v_arr[-1]
        interp = interp1d(t_arr, v_arr)

        def f(t: float) -> float:
            if 0.0 <= t <= t1:
                v = interp(t)
            elif t < 0.0:
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
        t_arr, v_arr = self.velocity_acc(self.dv_f, t0, v0)
        t1 = t_arr[-1]
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
        t_arr, s_arr = self.position_acc(self.dv_i)
        t_min = t_arr[0]
        t_max = t_arr[-1]
        s_max = s_arr[-1]
        interp_s = interp1d(s_arr, t_arr)

        def f(s: float) -> float:
            if 0.0 <= s <= s_max:
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
        t_arr, v_arr = self.velocity_acc(self.dv_f, t0, v0)
        t1 = t_arr[-1]

        # Between t0 and t1 it is possible for the velocity to become negative
        # depending on the initial conditions.
        if v_arr[-1] < 0.0:
            # Find time `t1` where `velocity = 0`
            t_pos = t_arr[v_arr > 0][-1]
            t_neg = t_arr[v_arr < 0][0]
            interp_t = interp1d(t_arr, v_arr)
            sol = root_scalar(interp_t, bracket=(t_pos, t_neg))
            t1 = sol.root

        t_arr, s_arr = self.position_acc(self.dv_f, t1, s0, v0)
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

    def get_dv_min(self) -> float:
        dv_min = self.dt_acc_min * self.a_max
        return dv_min

    def _calc_a_top(self, dv: float) -> tuple[float, float]:
        if dv == 0.0:
            return 0.0, 0.0
        dv = abs(dv)
        dt_acc = dv / self.a_max
        if dt_acc < self.dt_acc_min:
            a_top = dv / self.dt_acc_min
            return a_top, self.dt_acc_min
        return self.a_max, dt_acc

    def acceleration(
        self,
        dv: float,
        t0: float = 0.0
    ) -> tuple[np.ndarray, np.ndarray]:
        a_top, dt_acc = self._calc_a_top(dv)
        if dt_acc > 0.0:
            t_arr = np.linspace(0.0, dt_acc, 100)
            t_arr = t0 + t_arr
            a_arr = np.full_like(t_arr, a_top)
            if dv < 0.0:
                a_arr = np.full_like(t_arr, -a_top)
        else:
            t_arr = np.array([t0])
            a_arr = np.array([0.0])
        return t_arr, a_arr


class SCurvedProfile(MotionProfile):

    def get_dv_min(self) -> float:
        dv_min = self.dt_acc_min * self.a_max / 2
        return dv_min

    def _calc_a_top(self, dv: float) -> tuple[float, float]:
        if dv == 0.0:
            return 0.0, 0.0
        dv = abs(dv)
        dt_acc = 2 * dv / self.a_max
        if dt_acc < self.dt_acc_min:
            theta = np.atan(2 * self.a_max / self.dt_acc_min)
            a = -np.sin(theta) * np.cos(theta)
            b = self.dt_acc_min * np.sin(theta)
            c = -dv
            a_top = min(
                (-b + np.sqrt(b ** 2 - 4 * a * c)) / (2 * a),
                (-b - np.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
            )
            return a_top, self.dt_acc_min
        return self.a_max, dt_acc

    def acceleration(
        self,
        dv: float,
        t0: float = 0.0
    ) -> tuple[np.ndarray, np.ndarray]:
        from .elementary import cubic as cj
        a_top, dt_acc = self._calc_a_top(dv)
        if dt_acc > 0.0:
            t_arr = np.linspace(t0, t0 + dt_acc, 100)
            dt = t_arr[-1] - t_arr[0]
            if a_top < self.a_max:
                j1 = 2 * self.a_max / self.dt_acc_min
                theta = np.atan(j1)
                dt_acc = a_top / np.tan(theta)
                dt_cst = dt - 2 * dt_acc
                t1 = t_arr[0] + dt_acc
                t2 = t_arr[0] + dt_acc + dt_cst

                # Add t1 and t2 to t_arr if not already present.
                for t in [t1, t2]:
                    if not np.any(np.isclose(t_arr, t)):
                        t_arr = np.insert(t_arr, np.searchsorted(t_arr, t), t)

                t1_arr = t_arr[t_arr <= t1]
                tc_arr = t_arr[(t_arr > t1) & (t_arr < t2)]
                t2_arr = t_arr[t_arr >= t2]
                a1_arr = cj.acceleration(t1_arr, t0=t_arr[0], a0=0.0, j0=j1)
                ac_arr = cj.acceleration(tc_arr, t0=t1, a0=a_top, j0=0.0)
                a2_arr = cj.acceleration(t2_arr, t0=t2, a0=a_top, j0=-j1)
                a_arr = np.concatenate((a1_arr, ac_arr, a2_arr))
                if dv < 0.0:
                    a_arr = -a_arr
            else:
                j1 = 2 * self.a_max / dt
                dt_acc = dt / 2
                t1 = t_arr[0] + dt_acc

                # Add t1 to t_arr if not already present.
                if not np.any(np.isclose(t_arr, t1)):
                    t_arr = np.insert(t_arr, np.searchsorted(t_arr, t1), t1)

                t1_arr = t_arr[t_arr <= t1]
                t2_arr = t_arr[t_arr > t1]
                a1_arr = cj.acceleration(t1_arr, t0=t_arr[0], a0=0.0, j0=j1)
                a2_arr = cj.acceleration(t2_arr, t0=t1, a0=a_top, j0=-j1)
                a_arr = np.concatenate((a1_arr, a2_arr))
                if dv < 0.0:
                    a_arr = -a_arr
        else:
            t_arr = np.array([t0])
            a_arr = np.array([0.0])
        return t_arr, a_arr


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
