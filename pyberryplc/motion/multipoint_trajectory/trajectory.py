from typing import Sequence, List, Tuple, Union, Any, Literal, Dict, Optional
import json

import numpy as np
from scipy.interpolate import interp1d

from pyberryplc.charts import LineChart
from pyberryplc.motion import RotationDirection
from .motion_planning import MultipointMotionProfile


def split_on_displacement_sign_change(
    t: Sequence[float],
    x: Sequence[float],
    y: Sequence[float],
    *,
    ignore_zero: bool = True,
    return_break_indices: bool = False,
) -> Union[List[Any], Tuple[List[Any], List[Any]]]:
    """
    Split (t, x, y) into sub-sequences at every sign change of consecutive
    displacements in either x or y. The split point is shared: the last sample
    of segment i equals the first sample of segment i+1.

    Parameters
    ----------
    t, x, y:
        Aligned sequences of equal length (n >= 2).
    ignore_zero:
        If True (default), treat zero displacements as neutral and do not
        create boundaries on 0 → ± or ± → 0 by themselves. Zeros are "bridged"
        so a sign flip across a run of zeros still causes a single boundary at
        the first index where the non-zero sign resumes.
    return_break_indices:
        If True, also return the list of break indices (0-based).

    Returns
    -------
    segments:
        A list of tuples (t_seg, x_seg, y_seg), where each *_seg is a Python
        list. Segments share their boundary sample (overlapping endpoints).
        If `return_break_indices=True`, returns (segments, breaks) where breaks
        is a list of boundary indices (each boundary sample belongs to both
        adjacent segments).

    Examples
    --------
        >>> t = [0, 1, 2, 3, 4, 5]
        >>> x = [0, 1, 2, 1, 0, -1]   # dx: +, +, -, -, -
        >>> y = [0, 0, 0, 0, 0, 0]    # dy: 0, 0, 0, 0, 0
        >>> segs, brks = split_on_displacement_sign_change(t, x, y, return_break_indices=True)
    """
    if not (len(t) == len(x) == len(y)):
        raise ValueError("t, x, y must have the same length")
    n = len(t)
    if n < 2:
        # Nothing to split; return as a single segment
        return (
            [ (list(t), list(x), list(y)) ]
            if not return_break_indices
            else ([(list(t), list(x), list(y))], [])
        )

    t_arr = np.asarray(t)
    x_arr = np.asarray(x, dtype=float)
    y_arr = np.asarray(y, dtype=float)

    dx = np.diff(x_arr)
    dy = np.diff(y_arr)

    def _effective_sign(d: np.ndarray) -> np.ndarray:
        """
        Return sign array in {-1, 0, +1}; if ignore_zero=True, bridge zeros
        to avoid spurious boundaries on 0 steps, while still detecting a flip
        across zero-runs.
        """
        s = np.sign(d)
        if not ignore_zero or len(s) == 0:
            return s

        # Forward-fill zeros with previous sign
        s_ff = s.copy()
        for i in range(1, len(s_ff)):
            if s_ff[i] == 0:
                s_ff[i] = s_ff[i - 1]

        # Backfill leading zeros with the first non-zero sign (if any)
        if s_ff[0] == 0:
            nz = np.flatnonzero(s_ff != 0)
            if nz.size:
                s_ff[:nz[0]] = s_ff[nz[0]]
        return s_ff

    sx = _effective_sign(dx)
    sy = _effective_sign(dy)

    # A boundary at index j (1..n-2 mapped to 1..len(diff)-1) means the shared
    # sample is j because the sign flips between displacement (j-1 -> j) and
    # (j -> j+1).
    change_x = np.where(sx[1:] * sx[:-1] == -1)[0] + 1
    change_y = np.where(sy[1:] * sy[:-1] == -1)[0] + 1

    breaks = np.unique(np.concatenate([change_x, change_y]))
    # Build boundaries as sample indices, including start (0) and end (n-1)
    boundaries = np.concatenate([[0], breaks, [n - 1]])

    segments = []
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        a = int(a)
        b = int(b)
        # Include both ends; next segment starts at b (shared boundary sample)
        segments.append((
            t_arr[a:b+1].tolist(),
            x_arr[a:b+1].tolist(),
            y_arr[a:b+1].tolist(),
        ))

    if return_break_indices:
        return segments, breaks.tolist()
    return segments


def accel_from_speed_heading(t, v, theta):
    """
    Compute ax, ay, |a|, angle(alpha), and (a_t, a_n) from speed and heading.

    Parameters
    ----------
    t : array_like
        Time stamps [s], can be non-uniform.
    v : array_like
        Speed samples [m/s].
    theta : array_like
        Heading angles [rad], measured from +x, increasing CCW.

    Returns
    -------
    ax, ay : np.ndarray
        Cartesian acceleration components [m/s^2].
    a_mag : np.ndarray
        Acceleration magnitude [m/s^2].
    alpha : np.ndarray
        Angle of acceleration w.r.t. +x, in radians.
    a_t, a_n : np.ndarray
        Tangential (dv/dt) and normal (v*theta_dot) components [m/s^2].
    """
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    theta = np.unwrap(np.asarray(theta, dtype=float))  # avoid 2π jumps

    v_dot = np.gradient(v, t)  # dv/dt (handles non-uniform t)
    theta_dot = np.gradient(theta, t)  # dtheta/dt

    cos_th, sin_th = np.cos(theta), np.sin(theta)

    # a = v_dot*T + v*theta_dot*N, with T=(cos, sin), N=(-sin, cos)
    ax = v_dot * cos_th - v * theta_dot * sin_th
    ay = v_dot * sin_th + v * theta_dot * cos_th

    a_mag = np.hypot(ax, ay)
    alpha = np.arctan2(ay, ax)
    a_t = v_dot
    a_n = v * theta_dot
    return ax, ay, a_mag, alpha, a_t, a_n


TAxis = Literal["x", "y", "z"]
TAngularDisplacement = Tuple[float, RotationDirection]
TStepPulseSignal = Tuple[List[float], RotationDirection]


class Axis:

    def __init__(
        self,
        t_vals: List[float],
        coords: List[float],
        pitch: float,
        rdir_ref: RotationDirection,
        full_steps_per_rev: int,
        microstep_factor: int
    ) -> None:
        self.t_vals = t_vals
        self.coords = coords
        self.pitch = pitch
        self.rdir_ref = rdir_ref
        self.full_steps_per_rev = full_steps_per_rev
        self.f_microstep = microstep_factor

        self.angles = self._convert_to_angular(self.coords)
        self.Dang_m, self.rdir = self._get_angular_displacement()

    def _convert_to_angular(
        self,
        coords: List[float]
    ) -> List[float]:
        return [co * self.pitch * 360.0 for co in coords]

    def _get_angular_displacement(self) -> TAngularDisplacement:
        delta = self.angles[-1] - self.angles[0]
        magn = abs(delta)
        rdir = self.rdir_ref if delta >= 0.0 else ~self.rdir_ref
        return magn, rdir

    def get_stepper_signal(self) -> TStepPulseSignal:
        step_width = 20e-6  # step pulse width in seconds
        steps_per_degree = self.full_steps_per_rev * self.f_microstep / 360.0
        step_angle = 1.0 / steps_per_degree
        ang0 = self.angles[0]
        num_steps = int(round(self.Dang_m / step_angle))
        angles = [ang0 + i * step_angle for i in range(num_steps)]
        f = interp1d(self.angles, self.t_vals)
        t_arr = list(map(f, angles))
        delays = [
            max(0.0, t2 - t1 - step_width)
            for t1, t2 in zip(t_arr[:-1], t_arr[1:])
        ]
        return delays, self.rdir


class Segment:

    def __init__(
        self,
        n_axes: int,
        seg_tuple: Tuple[List[float], ...],
        pitch: Union[float, Tuple[float, ...]],
        rdir_ref: RotationDirection | tuple[RotationDirection, ...],
        full_steps_per_rev: int | tuple[int, ...],
        microstep_factor: int | tuple[int, ...]
    ) -> None:
        self.n_axes = n_axes
        self._seg_tuple = seg_tuple
        self.pitch = pitch
        self.rdir_ref = rdir_ref
        self.full_steps_per_rev = full_steps_per_rev
        self.f_microstep = microstep_factor
        self.axes: Dict[TAxis, Axis] = {}
        self._axis_mapping: Dict[int, TAxis] = {0: "x", 1: "y", 2: "z"}

        self._config_axes()

    @property
    def t(self) -> List[float]:
        return self._seg_tuple[0]

    @property
    def x(self) -> List[float]:
        return self._seg_tuple[1]

    @property
    def y(self) -> List[float]:
        return self._seg_tuple[2]

    @property
    def z(self) -> Optional[List[float]]:
        try:
            return self._seg_tuple[3]
        except IndexError:
            raise NotImplementedError

    def _get_pitch(self, index: int) -> float:
        if isinstance(self.pitch, (float, int)):
            return self.pitch
        if len(self.pitch) != self.n_axes:
            raise ValueError(
                f"The number of pitch values ({len(self.pitch)}) "
                f"and the number of axes ({self.n_axes}) don't "
                f"match."
            )
        return self.pitch[index]

    def _get_rdir_ref(self, index: int) -> RotationDirection:
        if not isinstance(self.rdir_ref, tuple):
            return self.rdir_ref
        if len(self.rdir_ref) != self.n_axes:
            raise ValueError(
                f"The number of positive reference rotation directions "
                f"({len(self.rdir_ref)}) and the number of axes "
                f"({self.n_axes}) don't match."
            )
        return self.rdir_ref[index]

    def _get_full_steps_per_rev(self, index: int) -> int:
        if isinstance(self.full_steps_per_rev, (float, int)):
            return self.full_steps_per_rev
        if len(self.full_steps_per_rev) != self.n_axes:
            raise ValueError(
                f"The number of values for full steps per rev "
                f"({len(self.full_steps_per_rev)}) and the number "
                f"of axes ({self.n_axes}) don't match."
            )
        return self.full_steps_per_rev[index]

    def _get_microstep_factor(self, index: int) -> int:
        if isinstance(self.f_microstep, (float, int)):
            return self.f_microstep
        if len(self.f_microstep) != self.n_axes:
            raise ValueError(
                f"The number of values for microstep factor "
                f"({len(self.f_microstep)}) and the number "
                f"of axes ({self.n_axes}) don't match."
            )
        return self.f_microstep[index]

    def _config_axes(self) -> None:
        for i in range(self.n_axes):
            axis_id = self._axis_mapping[i]
            axis = Axis(
                t_vals=self._seg_tuple[0],
                coords=self._seg_tuple[i + 1],
                pitch=self._get_pitch(i),
                rdir_ref=self._get_rdir_ref(i),
                full_steps_per_rev=self._get_full_steps_per_rev(i),
                microstep_factor=self._get_microstep_factor(i)
            )
            self.axes[axis_id] = axis

    def get_stepper_signals(self) -> Dict[TAxis, TStepPulseSignal]:
        d = {k: ax.get_stepper_signal() for k, ax in self.axes.items()}
        return d


class MultipointTrajectory:

    def __init__(
        self,
        motion_profile: MultipointMotionProfile,
        pitch: Union[float, Tuple[float, ...]],
        rdir_ref: RotationDirection | tuple[RotationDirection, ...],
        full_steps_per_rev: int | tuple[int, ...],
        microstep_factor: int | tuple[int, ...],
        n_axes: int = 2
    ) -> None:
        self.n_axes = n_axes
        self.motion_profile = motion_profile
        self.pitch = pitch
        self.rdir_ref = rdir_ref
        self.full_steps_per_rev = full_steps_per_rev
        self.f_microstep = microstep_factor
        self.segments: List[Segment] = []
        self._axis_mapping: Dict[int, TAxis] = {0: "x", 1: "y", 2: "z"}

        self._config_segments()

    @property
    def t(self) -> List[float]:
        return self.motion_profile.t

    @property
    def x(self) -> List[float]:
        return self.motion_profile.x

    @property
    def y(self) -> List[float]:
        return self.motion_profile.y

    def _config_segments(self) -> None:
        mp = self.motion_profile
        seg_tuples = split_on_displacement_sign_change(t=mp.t, x=mp.x, y=mp.y)
        for seg_tuple in seg_tuples:
            segment = Segment(
                n_axes=self.n_axes,
                seg_tuple=seg_tuple,
                pitch=self.pitch,
                rdir_ref=self.rdir_ref,
                full_steps_per_rev=self.full_steps_per_rev,
                microstep_factor=self.f_microstep
            )
            self.segments.append(segment)

    def get_stepper_signals(self) -> List[Dict[TAxis, TStepPulseSignal]]:
        return [seg.get_stepper_signals() for seg in self.segments]

    def save_stepper_signals(self, file_path: str) -> None:
        with open(file_path, "w") as f:
            json.dump(self.get_stepper_signals(), f)

    @classmethod
    def load_stepper_signals(cls, filepath: str) -> List[Dict[TAxis, TStepPulseSignal]]:
        with open(filepath, "r") as f:
            stepper_signals = json.load(f)
            return stepper_signals

    @property
    def position_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        t_arr = np.asarray(self.t)
        pos_profiles = {
            "x": (t_arr, np.asarray(self.x)),
            "y": (t_arr, np.asarray(self.y))
        }
        return pos_profiles

    @property
    def velocity_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        t = np.asarray(self.t)
        v = np.asarray(self.motion_profile.v)
        theta = np.asarray([pose.theta for pose in self.motion_profile.poses])
        vx = v * np.cos(theta)
        vy = v * np.sin(theta)
        vel_profiles = {"x": (t, vx), "y": (t, vy)}
        return vel_profiles

    @property
    def acceleration_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        t = np.asarray(self.t)
        v = np.asarray(self.motion_profile.v)
        theta = np.asarray([pose.theta for pose in self.motion_profile.poses])
        ax, ay, *_ = accel_from_speed_heading(t, v, theta)
        acc_profiles = {"x": (t, ax), "y": (t, ay)}
        return acc_profiles

    @staticmethod
    def _plot_profiles(
        profiles: dict[str, tuple[np.ndarray, np.ndarray]],
        ytitle: str
    ) -> LineChart:
        chart = LineChart()
        for axis_id, profile in profiles.items():
            chart.add_xy_data(
                label=axis_id,
                x1_values=profile[0],
                y1_values=profile[1]
            )
        chart.x1.add_title("time")
        chart.y1.add_title(ytitle)
        chart.add_legend(columns=len(profiles))
        return chart

    @property
    def position_profiles_plot(self) -> LineChart:
        pchart = self._plot_profiles(
            self.position_profiles,
            ytitle="position"
        )
        return pchart

    @property
    def velocity_profiles_plot(self) -> LineChart:
        vchart = self._plot_profiles(
            self.velocity_profiles,
            ytitle="velocity"
        )
        return vchart

    @property
    def acceleration_profiles_plot(self) -> LineChart:
        achart = self._plot_profiles(
            self.acceleration_profiles,
            ytitle="acceleration"
        )
        return achart

    def get_coordinates(self, _: int = 100) -> tuple[np.ndarray, ...]:
        return np.asarray(self.x), np.asarray(self.y)
