from dataclasses import dataclass, fields
import json
import csv

import numpy as np

from pyberryplc.charts import LineChart
from .profile import MotionProfile, RotationDirection


@dataclass
class MotionProfileParams:
    """
    Dataclass holding the parameters of a motion profile.

    Attributes
    ----------
    ds_tot:
        Total travel distance.
    a_max:
        (Maximum) acceleration (of the motor). This value remains unaltered
        in the calculation of the motion profile (i.e. `self.a_top` is
        always equal to `a_max`).
    v_max:
        (Maximum) speed limit (of the motor). `v_max` needs to be
        distinguished from the top velocity `self.v_top` of the motion
        profile. `v_max` poses a maximum limit when calculating `self.v_top`.
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
    """
    ds_tot: float
    a_max: float
    v_max: float
    v_i: float | None = None
    v_f: float | None = None
    s_i: float = 0.0
    dt_tot: float | None = None

    def as_dict(self) -> dict[str, float | None]:
        d = {f.name: getattr(self, f.name) for f in fields(self)}
        return d


class StepperMotorMock:
    """
    This class is only intended to mock a `StepperMotor` object.
    """
    def __init__(
        self,
        full_steps_per_rev: int = 200,
        microstep_factor: int = 1
    ) -> None:
        """
        Creates a `StepperMotorMock` object which has only the attributes that
        are needed to generate the step pulse train (list of the time delays
        between successive step pulses) for driving a stepper motor.

        Parameters
        ----------
        full_steps_per_rev : int, optional
            Full steps per revolution of the motor. Defaults to 200.
        microstep_factor : int
            Inverse of the microstep resolution.
        """
        self.full_steps_per_rev = full_steps_per_rev
        self.microstep_factor = microstep_factor

    @property
    def steps_per_degree(self) -> float:
        steps_per_degree = self.full_steps_per_rev * self.microstep_factor / 360.0
        return round(steps_per_degree, 6)

    @property
    def step_angle(self) -> float:
        step_angle = 1.0 / self.steps_per_degree
        return round(step_angle, 6)


class Axis:

    def __init__(
        self,
        profile_type: type[MotionProfile],
        pitch: float,
        rdir_ref: RotationDirection,
        alpha_max: float,
        omega_max: float,
        full_steps_per_rev: int,
        microstep_factor: int
    ) -> None:
        self.profile_type = profile_type
        self.pitch = pitch
        self.rdir_ref = rdir_ref
        self.alpha_max = alpha_max
        self.omega_max = omega_max
        self.full_steps_per_rev = full_steps_per_rev
        self.microstep_factor = microstep_factor

        self.s1: float = 0.0
        self.s2: float = 0.0
        self.theta1: float = 0.0
        self.dtheta: float = 0.0
        self.stepper_motor = StepperMotorMock(self.full_steps_per_rev, self.microstep_factor)
        self.rdir: RotationDirection = RotationDirection.CCW
        self.mpparams: MotionProfileParams | None = None

    def __call__(
        self,
        s1: float,
        s2: float,
    ) -> None:
        self.s1, self.s2 = s1, s2
        ds = s2 - s1
        self.theta1 = s1 * self.pitch * 360.0
        self.dtheta = abs(ds * self.pitch * 360.0)
        self.rdir = self.rdir_ref if ds >= 0.0 else ~self.rdir_ref
        self.mpparams = MotionProfileParams(
            ds_tot=self.dtheta,
            a_max=self.alpha_max,
            v_max=self.omega_max,
            v_i=0.0,
            v_f=0.0,
            s_i=self.theta1
        )

    def __str__(self) -> str:
        return f"{self.s1} -> {self.s2}"

    def is_undefined(self) -> bool:
        if self.mpparams is None:
            return True
        return False

    def is_moving(self) -> bool:
        if not self.is_undefined() and self.dtheta > 0.0:
            return True
        return False

    @property
    def motion_profile(self) -> MotionProfile:
        if not self.is_undefined():
            mp = self.profile_type(**self.mpparams.as_dict())
            return mp
        raise ValueError("Axis motion is still undefined.")

    def get_travel_time(self) -> float:
        if not self.is_undefined():
            if self.mpparams.dt_tot is None:
                self.mpparams.dt_tot = self.motion_profile.dt_tot
            return self.mpparams.dt_tot
        raise ValueError("Axis motion is still undefined.")

    @property
    def position_profile(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.is_undefined():
            t_arr, theta_arr = self.motion_profile.position_profile()
            s_arr = theta_arr / (360.0 * self.pitch)
            if self.rdir == ~self.rdir_ref:
                s0 = s_arr[0]
                ds_arr = s_arr - s0
                s0_arr = np.full_like(s_arr, s0)
                s_arr = s0_arr - ds_arr
            return t_arr, s_arr
        raise ValueError("Axis motion is still undefined.")

    @property
    def velocity_profile(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.is_undefined():
            t_arr, omega_arr = self.motion_profile.velocity_profile()
            v_arr = omega_arr / (360.0 * self.pitch)
            if self.rdir == ~self.rdir_ref:
                v_arr *= -1
            return t_arr, v_arr
        raise ValueError("Axis motion is still undefined.")

    @property
    def acceleration_profile(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.is_undefined():
            t_arr, alpha_arr = self.motion_profile.acceleration_profile()
            a_arr = alpha_arr / (360.0 * self.pitch)
            return t_arr, a_arr
        raise ValueError("Axis motion is still undefined.")

    def get_positions(self, t_arr: np.ndarray) -> np.ndarray:
        if not self.is_undefined():
            vfn = np.vectorize(self.motion_profile.get_position_from_time_fn())
            theta_arr = vfn(t_arr)
            s_arr = theta_arr / (360.0 * self.pitch)
            if self.rdir == ~self.rdir_ref:
                s0 = s_arr[0]
                ds_arr = s_arr - s0
                s0_arr = np.full_like(s_arr, s0)
                s_arr = s0_arr - ds_arr
            return s_arr
        raise ValueError("Axis motion is still undefined.")

    def get_stepper_signal(self) -> tuple[list[float], RotationDirection]:
        from pyberryplc.stepper.driver.base import TwoStageMotionProfileRotator
        # noinspection PyTypeChecker
        rotator = TwoStageMotionProfileRotator(self.stepper_motor)
        rotator.preprocess(self.rdir, self.motion_profile)
        return rotator.delays, self.rdir


class XYZSegment:

    def __init__(
        self,
        n_axes: int,
        profile_type: type[MotionProfile],
        pitch: float | tuple[float, ...],
        rdir_ref: RotationDirection | tuple[RotationDirection, ...],
        alpha_max: float | tuple[float, ...],
        omega_max: float | tuple[float, ...],
        full_steps_per_rev: int | tuple[int, ...],
        microstep_factor: int | tuple[int, ...]
    ) -> None:
        self.n_axes = n_axes
        self.profile_type = profile_type
        self.pitch = pitch
        self.rdir_ref = rdir_ref
        self.alpha_max = alpha_max
        self.omega_max = omega_max
        self.full_steps_per_rev = full_steps_per_rev
        self.microstep_factor = microstep_factor
        self.axes: dict[str, Axis] = {}
        self._axis_mapping = {0: "x", 1: "y", 2: "z"}
        self.p1: tuple[float, ...] = tuple()
        self.p2: tuple[float, ...] = tuple()
        self._config_axes()

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

    def _get_alpha_max(self, index: int) -> float:
        if isinstance(self.alpha_max, (float, int)):
            return self.alpha_max
        if len(self.alpha_max) != self.n_axes:
            raise ValueError(
                f"The number of maximum angular acceleration values "
                f"({len(self.alpha_max)}) and the number of axes "
                f"({self.n_axes}) don't match."
            )
        return self.alpha_max[index]

    def _get_omega_max(self, index: int) -> float:
        if isinstance(self.omega_max, (float, int)):
            return self.omega_max
        if len(self.omega_max) != self.n_axes:
            raise ValueError(
                f"The number of maximum angular velocity values "
                f"({len(self.omega_max)}) and the number of axes "
                f"({self.n_axes}) don't match."
            )
        return self.omega_max[index]

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

    def _get_microstep_factor(self, index: int):
        if isinstance(self.microstep_factor, (float, int)):
            return self.microstep_factor
        if len(self.microstep_factor) != self.n_axes:
            raise ValueError(
                f"The number of values for microstep factor "
                f"({len(self.microstep_factor)}) and the number "
                f"of axes ({self.n_axes}) don't match."
            )
        return self.microstep_factor[index]

    def _config_axes(self) -> None:
        for i in range(self.n_axes):
            axis_id = self._axis_mapping[i]
            axis = Axis(
                profile_type=self.profile_type,
                pitch=self._get_pitch(i),
                rdir_ref=self._get_rdir_ref(i),
                alpha_max=self._get_alpha_max(i),
                omega_max=self._get_omega_max(i),
                full_steps_per_rev=self._get_full_steps_per_rev(i),
                microstep_factor=self._get_microstep_factor(i)
            )
            self.axes[axis_id] = axis

    def __call__(
        self,
        p1: tuple[float, ...],
        p2: tuple[float, ...]
    ) -> None:
        if len(self.axes) != len(p1) or len(self.axes) != len(p2):
            raise ValueError(
                "The number of configured axes and the number of"
                "coordinates of the given points don't match."
            )
        self.p1, self.p2 = p1, p2
        axes_positions = [(s1, s2) for s1, s2 in zip(p1, p2)]
        for i, axis in enumerate(self.axes.values()):
            axis(*axes_positions[i])

    def __str__(self) -> str:
        return f"{self.p1} -> {self.p2}"

    @property
    def moving_axes(self) -> dict[str, Axis]:
        return {k: ax for k, ax in self.axes.items() if ax.is_moving()}

    def synchronize_axes(self) -> None:
        if len(self.moving_axes) > 1:
            slowest_axis = max(
                self.moving_axes.values(),
                key=lambda ax_: ax_.get_travel_time()
            )
        elif len(self.moving_axes) == 1:
            slowest_axis = list(self.moving_axes.values())[0]
        else:
            slowest_axis = None
        if len(self.axes) > 1 and slowest_axis is not None:
            for ax in self.axes.values():
                if ax is not slowest_axis:
                    ax.mpparams.dt_tot = slowest_axis.get_travel_time()

    def is_synchronized(self) -> bool:
        dt_tot_lst = [ax.get_travel_time() for ax in self.axes.values()]
        i_max = len(dt_tot_lst) - 1
        results: list[bool] = []
        for i in range(i_max):
            dt_tot1 = dt_tot_lst[i]
            dt_tot2 = dt_tot_lst[i + 1]
            if dt_tot1 == dt_tot2:
                results.append(True)
            else:
                results.append(False)
        if all(results):
            return True
        return False

    def get_path_deviation(self, n_points: int = 100) -> float:
        if not self.is_synchronized():
            raise ValueError("Axes have not been synchronized yet.")
        if len(self.moving_axes) > 1:
            dt_tot = list(self.axes.values())[0].mpparams.dt_tot
            t_arr = np.linspace(0.0, dt_tot, n_points)
            axes_positions = [
                ax.get_positions(t_arr)
                for ax in self.moving_axes.values()
            ]
            p0 = np.array([arr[0] for arr in axes_positions])
            p1 = np.array([arr[-1] for arr in axes_positions])
            dp = p1 - p0
            dp_norm_sq = np.dot(dp, dp)
            point_coords = np.vstack(axes_positions)
            _, n_cols = point_coords.shape
            max_dev = 0.0
            for i in range(n_cols):
                p = point_coords[:, i]
                alpha = np.dot(p - p0, dp) / dp_norm_sq
                proj = p0 + alpha * dp
                dev = np.linalg.norm(p - proj)
                max_dev = max(max_dev, dev)
            return max_dev
        return 0.0

    def get_coordinates(self, n_points: int = 100) -> tuple[np.ndarray, ...]:
        if not self.is_synchronized():
            raise ValueError("Axes have not been synchronized yet.")
        dt_tot = list(self.axes.values())[0].mpparams.dt_tot
        t_arr = np.linspace(0.0, dt_tot, n_points)
        coords = tuple(
            ax.get_positions(t_arr)
            for ax in self.axes.values()
        )
        return coords

    @property
    def position_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        d = {k: ax.position_profile for k, ax in self.axes.items()}
        return d

    @property
    def velocity_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        d = {k: ax.velocity_profile for k, ax in self.axes.items()}
        return d

    @property
    def acceleration_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        d = {k: ax.acceleration_profile for k, ax in self.axes.items()}
        return d

    def get_stepper_signals(self) -> dict[str, tuple[list[float], RotationDirection]]:
        d = {k: ax.get_stepper_signal() for k, ax in self.axes.items()}
        return d


class PointToPointTrajectory:

    def __init__(
        self,
        n_axes: int,
        profile_type: type[MotionProfile],
        pitch: float | tuple[float, ...],
        rdir_ref: RotationDirection | tuple[RotationDirection, ...],
        alpha_max: float | tuple[float, ...],
        omega_max: float | tuple[float, ...],
        full_steps_per_rev: int | tuple[int, ...],
        microstep_factor: int | tuple[int, ...]
    ) -> None:
        self.n_axes = n_axes
        self.profile_type = profile_type
        self.pitch = pitch
        self.rdir_ref = rdir_ref
        self.alpha_max = alpha_max
        self.omega_max = omega_max
        self.full_steps_per_rev = full_steps_per_rev
        self.microstep_factor = microstep_factor
        self._axis_mapping = {0: "x", 1: "y", 2: "z"}

        self.segments: list[XYZSegment] = []

    def _set_intermediate_velocities(self) -> None:
        size = len(self.segments)
        i_max = size - 1
        for i in range(size):
            if i < i_max:
                seg1 = self.segments[i]
                seg2 = self.segments[i + 1]
                for j in range(self.n_axes):
                    axis1 = list(seg1.axes.values())[j]
                    axis2 = list(seg2.axes.values())[j]
                    axis1.mpparams.dt_tot = None
                    axis2.mpparams.dt_tot = None
                    dtheta1 = axis1.mpparams.ds_tot
                    dtheta2 = axis2.mpparams.ds_tot
                    rdir1 = int(axis1.rdir)
                    rdir2 = int(axis2.rdir)
                    if dtheta1 == 0.0:
                        axis1.mpparams.v_i = 0.0
                        axis1.mpparams.v_f = 0.0
                        axis2.mpparams.v_i = 0.0
                    if dtheta2 == 0.0 or rdir1 * rdir2 < 0:
                        axis1.mpparams.v_f = 0.0
                        axis2.mpparams.v_i = 0.0
                    if dtheta1 > 0.0 and dtheta2 > 0.0 and rdir1 * rdir2 == 1:
                        v_top1 = axis1.motion_profile.v_top
                        v_top2 = axis2.motion_profile.v_top
                        v_top = min(v_top1, v_top2)
                        axis1.mpparams.v_f = v_top
                        axis2.mpparams.v_i = v_top
        for seg in self.segments:
            seg.synchronize_axes()

    def __call__(self, *points: tuple[float, ...], continuous: bool = False) -> None:
        self.segments = []
        point_pairs = list(zip(points[:-1], points[1:]))
        for point_pair in point_pairs:
            segment = XYZSegment(
                self.n_axes,
                self.profile_type,
                self.pitch,
                self.rdir_ref,
                self.alpha_max,
                self.omega_max,
                self.full_steps_per_rev,
                self.microstep_factor
            )
            segment(p1=point_pair[0], p2=point_pair[1])
            segment.synchronize_axes()
            self.segments.append(segment)
        if continuous:
            self._set_intermediate_velocities()

    @staticmethod
    def _connect_axis_profiles(
        *profiles: tuple[np.ndarray, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray]:
        t_arr_lst, arr_lst = [], []
        for i, profile in enumerate(profiles):
            t_arr, arr = profile
            if i > 0:
                dt_shift = t_arr_lst[-1][-1]
                t_arr += dt_shift
            t_arr_lst.append(t_arr)
            arr_lst.append(arr)
        t_arr_concat = np.concatenate(tuple(t_arr_lst))
        arr_concat = np.concatenate(tuple(arr_lst))
        return t_arr_concat, arr_concat

    @property
    def position_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        pos_profiles = [
            self._connect_axis_profiles(*profiles)
            for profiles in zip(*[
                seg.position_profiles.values()
                for seg in self.segments
            ])
        ]
        pos_profiles = {
            self._axis_mapping[i]: pos_profiles[i]
            for i in range(len(pos_profiles))
        }
        return pos_profiles

    @property
    def velocity_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        vel_profiles = [
            self._connect_axis_profiles(*profiles)
            for profiles in zip(*[
                seg.velocity_profiles.values()
                for seg in self.segments
            ])
        ]
        vel_profiles = {
            self._axis_mapping[i]: vel_profiles[i]
            for i in range(len(vel_profiles))
        }
        return vel_profiles

    @property
    def acceleration_profiles(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        acc_profiles = [
            self._connect_axis_profiles(*profiles)
            for profiles in zip(*[
                seg.acceleration_profiles.values()
                for seg in self.segments
            ])
        ]
        acc_profiles = {
            self._axis_mapping[i]: acc_profiles[i]
            for i in range(len(acc_profiles))
        }
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

    def get_coordinates(self, n_points: int = 100) -> tuple[np.ndarray, ...]:
        traj_coords = [[] for _ in range(self.n_axes)]
        for segment in self.segments:
            seg_coords = segment.get_coordinates(n_points)
            for i in range(self.n_axes):
                traj_coords[i].append(seg_coords[i])
        traj_coords = tuple(np.concatenate(lst) for lst in traj_coords)
        return traj_coords

    def get_stepper_signals(self) -> list[dict[str, tuple[list[float], RotationDirection]]]:
        l = [seg.get_stepper_signals() for seg in self.segments]
        return l

    def save_stepper_signals(self, file_path: str) -> None:
        with open(file_path, "w") as f:
            # noinspection PyTypeChecker
            json.dump(self.get_stepper_signals(), f)

    @classmethod
    def load_stepper_signals(cls, filepath: str) -> list[dict[str, tuple[list[float], RotationDirection]]]:
        with open(filepath, "r") as f:
            stepper_signals = json.load(f)
            return stepper_signals

    def load_points_from_csv(self, filepath: str, continuous: bool = False) -> None:
        """
        Load point coordinates from csv file. Units must be mm.

        Parameters
        ----------
        filepath:
            Path to csv file.
        continuous:
            Indicates whether the segments are run one by one (False) or in a single run (True).
        """
        with open(filepath) as fh:
            reader = csv.reader(fh)
            points = []
            for row in reader:
                if len(row) >= 2:
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        points.append((x, y))
                    except ValueError:
                        continue
        points = [(x / 1000, y / 1000) for x, y in points]  # mm -> m
        self(*points, continuous=continuous)
