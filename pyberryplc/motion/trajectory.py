"""
Implements the `TrajectoryPlanner` class (and its helper classes). 

The `TrajectoryPlanner` class is used to create 2D- or 3D-trajectories composed
of rectilinear segments.
"""
import warnings
from typing import Type, Any
from dataclasses import dataclass, field, fields
import pickle
import json

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import minimize_scalar

from pyberryplc.motion.multi_axis import MotionProfile, RotationDirection

TPoint = tuple[float, float] | tuple[float, float, float]
TPointPair = tuple[TPoint, TPoint]
TProfile = tuple[NDArray[np.float64], NDArray[np.float64]]
TSegmentPath = tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]
TMotionProfileDict = dict[str, dict[str, NDArray[np.float64]]]


@dataclass
class AxisData:
    """
    Dataclass used by `SegementData` to return the data of a single axis
    (either X-, Y-, or Z-axis).
    """
    profile_type: Type[MotionProfile]
    dtheta: float
    theta_i: float
    omega_m: float
    alpha_m: float
    omega_i: float
    omega_f: float
    rdir: RotationDirection
    pitch: float
    dt_tot: float
    mp: MotionProfile | None


@dataclass
class SegmentData:
    """
    Intermediate dataclass that holds data about the rectilinear segments in a 
    trajectory needed for generating the motion profiles and the stepper motor 
    driver signals for the X-, Y-, and/or Z-axis movements. 
    """
    point_pair: TPointPair
    dtheta_x: float
    dtheta_y: float
    dtheta_z: float
    rdir_x: RotationDirection | None
    rdir_y: RotationDirection | None
    rdir_z: RotationDirection | None
    theta_xi: float
    theta_yi: float
    theta_zi: float
    profile_type: type[MotionProfile]
    omega_xm: float | None = None
    omega_ym: float | None = None
    omega_zm: float | None = None
    alpha_xm: float | None = None
    alpha_ym: float | None = None
    alpha_zm: float | None = None
    omega_xi: float | None = None
    omega_xf: float | None = None
    omega_yi: float | None = None
    omega_yf: float | None = None
    omega_zi: float | None = None
    omega_zf: float | None = None
    xpitch: float = 0.0
    ypitch: float = 0.0
    zpitch: float = 0.0
    dt_tot_x: float = 0.0
    dt_tot_y: float = 0.0
    dt_tot_z: float = 0.0
    mp_x: MotionProfile | None = None
    mp_y: MotionProfile | None = None
    mp_z: MotionProfile | None = None
    
    def get_xaxis_data(self) -> AxisData:
        return AxisData(
            dtheta=self.dtheta_x,
            theta_i=self.theta_xi,
            omega_m=self.omega_xm,
            alpha_m=self.alpha_xm,
            omega_i=self.omega_xi,
            omega_f=self.omega_xf,
            profile_type=self.profile_type,
            rdir=self.rdir_x,
            pitch=self.xpitch,
            dt_tot=self.dt_tot_x,
            mp=self.mp_x
        )

    def get_yaxis_data(self) -> AxisData:
        return AxisData(
            dtheta=self.dtheta_y,
            theta_i=self.theta_yi,
            omega_m=self.omega_ym,
            alpha_m=self.alpha_ym,
            omega_i=self.omega_yi,
            omega_f=self.omega_yf,
            profile_type=self.profile_type,
            rdir=self.rdir_y,
            pitch=self.ypitch,
            dt_tot=self.dt_tot_y,
            mp=self.mp_y
        )

    def get_zaxis_data(self) -> AxisData:
        return AxisData(
            dtheta=self.dtheta_z,
            theta_i=self.theta_zi,
            omega_m=self.omega_zm,
            alpha_m=self.alpha_zm,
            omega_i=self.omega_zi,
            omega_f=self.omega_zf,
            profile_type=self.profile_type,
            rdir=self.rdir_z,
            pitch=self.zpitch,
            dt_tot=self.dt_tot_z,
            mp=self.mp_z
        )
    
    def get_axis_data(self, axis: str) -> AxisData:
        if axis == "x":
            return self.get_xaxis_data()
        elif axis == "y":
            return self.get_yaxis_data()
        elif axis == "z":
            return self.get_zaxis_data()
        raise ValueError(f"Parameter `axis` must be 'x', 'y', or 'z'.")
    
    def set_xaxis_data(self, axisdata: AxisData) -> None:
        self.dtheta_x = axisdata.dtheta
        self.theta_xi = axisdata.theta_i
        self.omega_xm = axisdata.omega_m
        self.alpha_xm = axisdata.alpha_m
        self.omega_xi = axisdata.omega_i
        self.omega_xf = axisdata.omega_f
        self.profile_type = axisdata.profile_type
        self.rdir_x = axisdata.rdir
        self.xpitch = axisdata.pitch
        self.dt_tot_x = axisdata.dt_tot
        self.mp_x = axisdata.mp
    
    def set_yaxis_data(self, axisdata: AxisData) -> None:
        self.dtheta_y = axisdata.dtheta
        self.theta_yi = axisdata.theta_i
        self.omega_ym = axisdata.omega_m
        self.alpha_ym = axisdata.alpha_m
        self.omega_yi = axisdata.omega_i
        self.omega_yf = axisdata.omega_f
        self.profile_type = axisdata.profile_type
        self.rdir_y = axisdata.rdir
        self.ypitch = axisdata.pitch
        self.dt_tot_y = axisdata.dt_tot
        self.mp_y = axisdata.mp
        
    def set_zaxis_data(self, axisdata: AxisData) -> None:
        self.dtheta_z = axisdata.dtheta
        self.theta_zi = axisdata.theta_i
        self.omega_zm = axisdata.omega_m
        self.alpha_zm = axisdata.alpha_m
        self.omega_zi = axisdata.omega_i
        self.omega_zf = axisdata.omega_f
        self.profile_type = axisdata.profile_type
        self.rdir_z = axisdata.rdir
        self.zpitch = axisdata.pitch
        self.dt_tot_z = axisdata.dt_tot
        self.mp_z = axisdata.mp
        
    def set_axis_data(self, axis: str, axisdata: AxisData) -> None:
        if axis == "x":
            self.set_xaxis_data(axisdata)
            return
        elif axis == "y":
            self.set_yaxis_data(axisdata)
            return
        elif axis == "z":
            self.set_zaxis_data(axisdata)
            return
        raise ValueError(f"Parameter `axis` must be 'x', 'y', or 'z'.")
    
    def __str__(self):
        lines = []
        for field in fields(self):
            value = getattr(self, field.name)
            lines.append(f"{field.name}: {value}")
        return "\n".join(lines)


@dataclass
class Segment:
    """
    Dataclass with attributes for drawing and analyzing the mult-axis motion
    profiles of a line segment in the trajectory. It can also contain the step
    pulse signals for driving the stepper motors of the X-, Y-, and/or Z-axis.
    
    Attributes
    ----------
    point_pair:
        The start and end point of the segment.
    segmentdata:
        `SegmentData` object created by the `TrajectoryPlanner` holding the data
        used to create the `Segment` object.
    mp_x:
        Motion profile of the X-axis motor rotation.
    mp_y:
        Motion profile of the Y-axis motor rotation.
    mp_z:
        Motion profile of the Z-axis motor rotation.
    position_profiles:
        Position profiles of the axes.
    velocity_profiles:
        Velocity profiles of the axes.
    acceleration_profiles:
        Acceleration profiles of the axes.
    delays_x:
        Step signal to drive the X-axis stepper motor (list of time delays 
        between successive step pulses).
    delays_y:
        Step signal to drive the Y-axis stepper motor.
    delays_z:
        Step signal to drive the Z-axis stepper motor.
    path_deviation:
        Maximum deviation between the planned rectilinear path connecting start 
        and end point of the line segment and the actual motion path. 
    """
    point_pair: TPointPair = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    segmentdata: SegmentData | None = None
    mp_x: MotionProfile | None = None
    mp_y: MotionProfile | None = None
    mp_z: MotionProfile | None = None
    position_profiles: tuple[TProfile, ...] | None = None
    velocity_profiles: tuple[TProfile, ...] | None = None
    acceleration_profiles: tuple[TProfile, ...] | None = None
    delays_x: list[float] = field(default_factory=list)
    delays_y: list[float] = field(default_factory=list)
    delays_z: list[float] = field(default_factory=list)
    rdir_x: RotationDirection = RotationDirection.COUNTERCLOCKWISE
    rdir_y: RotationDirection = RotationDirection.COUNTERCLOCKWISE
    rdir_z: RotationDirection = RotationDirection.COUNTERCLOCKWISE
    path_deviation: float = 0.0
    
    def __str__(self):
        lines = []
        for field in fields(self):
            value = getattr(self, field.name)
            lines.append(f"{field.name}: {value}")
        return "\n".join(lines)

    @property
    def stepper_driver_signals(self) -> dict[str, Any]:
        """
        Returns the step pulse signals and rotation directions for driving the
        stepper motors of the X-, Y-, and/or Z-axis. If not present, a
        `ValueError` exception is raised.
        """
        if any((self.delays_x, self.delays_y, self.delays_z)):
            return {
                "delays_x": self.delays_x,
                "delays_y": self.delays_y,
                "delays_z": self.delays_z,
                "rdir_x": self.rdir_x,
                "rdir_y": self.rdir_y,
                "rdir_z": self.rdir_z
            }
        raise ValueError("The segment does not contain stepper driver signals.")


class StepperMotorMock:
    """
    This class is only intended to mock a `StepperMotor` object. It uses the
    function `preprocess()` of class `TwoStageMotionProfileRotator(ProfileRotator)` 
    for calculating the time delays between successive step pulses to drive a
    stepper motor.
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
        return self.full_steps_per_rev * self.microstep_factor / 360.0

    @property
    def step_angle(self) -> float:
        return 1.0 / self.steps_per_degree


class MotionProcessor:
    """
    Created and used internally in class `TrajectoryPlanner`. Cannot be used
    without a `TrajectoryPlanner`.
    
    It generates the final synchronized motion profiles for the X-, Y-, and
    Z-axis to move from the start to the end point of a single line segment in a
    trajectory.
    
    The step pulses for driving the X-, Y- and Z-axis stepper motors can also be
    generated with this class, if `StepperMotorMock` objects are passed to the 
    constructor of `TrajectoryPlanner` object.
    """
    def __init__(self, planner: 'TrajectoryPlanner') -> None:
        """
        Creates a `MotionProcessor` object.

        Parameters
        ----------
        planner:
            `TrajectoryPlanner` object to which the `MotionProcessor` object
            belongs.
        """
        from pyberryplc.stepper.driver.base import TwoStageMotionProfileRotator
        
        self.planner = planner
        self._segmentdata: SegmentData | None = None
        self._mp_x: MotionProfile | None = None
        self._mp_y: MotionProfile | None = None
        self._mp_z: MotionProfile | None = None
        self._x_rotator: TwoStageMotionProfileRotator | None = None
        self._y_rotator: TwoStageMotionProfileRotator | None = None
        self._z_rotator: TwoStageMotionProfileRotator | None = None
        self._x_delays: list[float] | None = None
        self._y_delays: list[float] | None = None
        self._z_delays: list[float] | None = None
        if self.planner.x_motor is not None:
            # noinspection PyTypeChecker
            self._x_rotator = TwoStageMotionProfileRotator(self.planner.x_motor)
        if self.planner.y_motor is not None:
            # noinspection PyTypeChecker
            self._y_rotator = TwoStageMotionProfileRotator(self.planner.y_motor)
        if self.planner.z_motor is not None:
            # noinspection PyTypeChecker
            self._z_rotator = TwoStageMotionProfileRotator(self.planner.z_motor)
    
    def _create_motion_profile(
        self,
        axis: str,
        dt_tot: float | None = None
    ) -> MotionProfile:
        """
        Takes the data of the indicated axis and creates a motion profile for
        that axis.
        """
        # noinspection PyUnreachableCode
        match axis:
            case "x":
                axisdata = self._segmentdata.get_xaxis_data()
            case "y":
                axisdata = self._segmentdata.get_yaxis_data()
            case "z":
                axisdata = self._segmentdata.get_zaxis_data()
            case _:
                raise ValueError(f"Invalid axis: {axis}")
        mp = axisdata.profile_type(
            ds_tot=axisdata.dtheta,
            a_m=axisdata.alpha_m,
            v_m=axisdata.omega_m,
            v_i=axisdata.omega_i,
            s_i=axisdata.theta_i,
            v_f=axisdata.omega_f,
            dt_tot=dt_tot
        )
        return mp
   
    def _synchronize_motion_profiles(
        self, 
        mp_x: MotionProfile, 
        mp_y: MotionProfile,
        mp_z: MotionProfile
    ) -> tuple[MotionProfile, MotionProfile, MotionProfile]:
        """
        Synchronizes the motion profiles of the X-, Y-, and Z-axis movements in
        the segment.

        Checks which of the motion profiles needs the most time and modifies
        the top velocity of the other motion profiles so that all axis movements 
        would finish at the same time.
        """
        # Only consider axes that need to move.
        moving = {
            axis: mp
            for axis, mp in (("x", mp_x), ("y", mp_y), ("z", mp_z))
            if mp.ds_tot > 0.0
        }
        non_moving = [mp for mp in (mp_x, mp_y, mp_z) if mp.ds_tot == 0.0]

        if len(moving) == 1:
            # Only one axis needs to move -> synchronisation is not needed.
            moving = [mp for mp in moving.values()]
            for mp in non_moving: mp.dt_tot = mp.dt_cov = moving[0].dt_tot
            return mp_x, mp_y, mp_z

        # Find the moving axis that has the longest travel time; this travel 
        # time must also become the travel time of the other moving axes.
        dt_tot_dict = {axis: mp.dt_tot for axis, mp in moving.items()}
        axis_with_dt_max = max(dt_tot_dict, key=dt_tot_dict.get)
        dt_max = dt_tot_dict[axis_with_dt_max]

        # Also set the travel time of non-moving axes to `dt_max` (needed for
        # plotting the motion profile of non-moving axes)
        for mp in non_moving: mp.dt_tot = mp.dt_cov = dt_max

        # The axis with the longest travel time remains unchanged and can be
        # omitted.
        moving.pop(axis_with_dt_max)

        # Recalculate the motion profile of the other moving axes so that their
        # travel time would equal the longest travel time.
        mp_x = self._create_motion_profile("x", dt_max) if "x" in moving.keys() else mp_x
        mp_y = self._create_motion_profile("y", dt_max) if "y" in moving.keys() else mp_y
        mp_z = self._create_motion_profile("z", dt_max) if "z" in moving.keys() else mp_z
        return mp_x, mp_y, mp_z
    
    def _create_motion_profiles(
        self
    ) -> tuple[MotionProfile, MotionProfile, MotionProfile]:
        """
        Creates and returns the final synchronized motion profiles for the X-,
        Y-, and Z-axis in the segment.
        """
        mp_x = self._create_motion_profile("x")
        mp_y = self._create_motion_profile("y")
        mp_z = self._create_motion_profile("z")
        mp_x, mp_y, mp_z = self._synchronize_motion_profiles(mp_x, mp_y, mp_z)
        return mp_x, mp_y, mp_z
    
    def _get_position_profiles(self) -> tuple[TProfile, TProfile, TProfile]:
        """
        Returns the position profiles of the X-, Y-, and Z-axis movements in the
        current segment.
        """
        def create_position_profile(mp, pitch, rdir):
            t_arr, theta_arr = mp.position_profile()
            s_arr = theta_arr / (360.0 * pitch)  # convert axis motor angle back to axis position  
            if rdir == RotationDirection.CLOCKWISE:
                s0 = s_arr[0]
                ds_arr = s_arr - s0
                s0_arr = np.full_like(s_arr, s0)
                s_arr = s0_arr - ds_arr
            return t_arr, s_arr
        
        pos_x = create_position_profile(
            self._mp_x, 
            self._segmentdata.xpitch,
            self._segmentdata.rdir_x
        )
        pos_y = create_position_profile(
            self._mp_y,
            self._segmentdata.ypitch,
            self._segmentdata.rdir_y
        )
        pos_z = create_position_profile(
            self._mp_z,
            self._segmentdata.zpitch,
            self._segmentdata.rdir_z
        )
        return pos_x, pos_y, pos_z
    
    def _get_velocity_profiles(self) -> tuple[TProfile, TProfile, TProfile]:
        """
        Returns the velocity profiles of the X-, Y-, and Z-axis movements in the
        current segment.
        """
        def create_velocity_profile(mp, pitch, rdir):
            t_arr, omega_arr = mp.velocity_profile()
            v_arr = omega_arr / (360.0 * pitch)
            if rdir == RotationDirection.CLOCKWISE:
                v_arr *= -1.0
            return t_arr, v_arr
        
        vel_x = create_velocity_profile(
            self._mp_x,
            self._segmentdata.xpitch,
            self._segmentdata.rdir_x
        )
        vel_y = create_velocity_profile(
            self._mp_y,
            self._segmentdata.ypitch,
            self._segmentdata.rdir_y
        )
        vel_z = create_velocity_profile(
            self._mp_z,
            self._segmentdata.zpitch,
            self._segmentdata.rdir_z
        )
        return vel_x, vel_y, vel_z
    
    def _get_acceleration_profiles(self) -> tuple[TProfile, TProfile, TProfile]:
        """
        Returns the acceleration profiles of the X-, Y-, and Z-axis movements in
        the current segment.
        """
        def create_acceleration_profile(mp, pitch):
            t_arr, alpha_arr = mp.acceleration_profile()
            a_arr = alpha_arr / (360.0 * pitch)
            return t_arr, a_arr
        
        acc_x = create_acceleration_profile(
            self._mp_x,
            self._segmentdata.xpitch
        )
        acc_y = create_acceleration_profile(
            self._mp_y,
            self._segmentdata.ypitch
        )
        acc_z = create_acceleration_profile(
            self._mp_z,
            self._segmentdata.zpitch
        )
        return acc_x, acc_y, acc_z
    
    def _generate_step_pulse_delays(self):
        """
        Generates the step pulse signals to drive the axis stepper motors
        based on the calculated motion profiles of the axes.
        """
        if not all((self._mp_x, self._mp_y, self._mp_z)):
            self._create_motion_profiles()
        
        if self._x_rotator:
            self._x_rotator.preprocess(self._segmentdata.rdir_x, self._mp_x)
            self._x_delays = self._x_rotator.delays
        
        if self._y_rotator:
            self._y_rotator.preprocess(self._segmentdata.rdir_y, self._mp_y)
            self._y_delays = self._y_rotator.delays
        
        if self._z_rotator:
            self._z_rotator.preprocess(self._segmentdata.rdir_z, self._mp_y)
            self._z_delays = self._z_rotator.delays

    def _get_path_deviation(
        self,
        segment: Segment,
        num_points: int = 100
    ) -> float:
        """
        Calculates and returns the maximum deviation between the actual segment
        path (which depends on the motion profiles of the individual axis
        movements) and the straight line between the start and end point of the
        segment.

        Parameters
        ----------
        segment:
            The segment for which the maximum path deviation is to be
            determined.
        num_points:
            To determine the maximum path deviation, a number of equally spaced
            points on the position profiles of the axis movements in the segment
            need to be taken. `num_points` indicates how many points are taken.
            The default number is 100 points.
        """
        moving_axes: dict[str, AxisData] = {}
        segmentdata = segment.segmentdata
        if segmentdata.dtheta_x > 0.0:
            moving_axes["x"] = segmentdata.get_xaxis_data()
            moving_axes["x"].mp = segment.mp_x
        if segmentdata.dtheta_y > 0.0:
            moving_axes["y"] = segmentdata.get_yaxis_data()
            moving_axes["y"].mp = segment.mp_y
        if segmentdata.dtheta_z > 0.0:
            moving_axes["z"] = segmentdata.get_zaxis_data()
            moving_axes["z"].mp = segment.mp_z
        path_deviation = self.planner._get_path_deviation(moving_axes, num_points)
        return path_deviation

    def create_segment(
        self, 
        segmentdata: SegmentData, 
        num_points: int = 100
    ) -> Segment:
        """
        Creates a `Segment` object from the data contained in the `SegmentData`
        object.

        Calculates the final synchronized motion profiles of the axes, and the
        step pulse signals to drive the axis stepper motors.
        """
        self._segmentdata = segmentdata
        self._mp_x, self._mp_y, self._mp_z = self._create_motion_profiles()
        self._generate_step_pulse_delays()

        segment = Segment()
        segment.segmentdata = self._segmentdata
        segment.point_pair = self._segmentdata.point_pair
        if self._mp_x: segment.mp_x = self._mp_x
        if self._mp_y: segment.mp_y = self._mp_y
        if self._mp_z: segment.mp_z = self._mp_z
        segment.position_profiles = self._get_position_profiles()
        segment.velocity_profiles = self._get_velocity_profiles()
        segment.acceleration_profiles = self._get_acceleration_profiles()
        if self._x_delays: segment.delays_x = self._x_delays
        if self._y_delays: segment.delays_y = self._y_delays
        if self._z_delays: segment.delays_z = self._z_delays
        segment.rdir_x = self._segmentdata.rdir_x
        segment.rdir_y = self._segmentdata.rdir_y
        segment.rdir_z = self._segmentdata.rdir_z
        segment.path_deviation = self._get_path_deviation(segment, num_points)
        return segment


class Trajectory(list[Segment]):
    """
    Represents a 3D trajectory in XYZ-space.

    A `Trajectory` object is a list of `Segment` objects.
    """
    def _connect_segments(self) -> tuple[dict[str, dict[str, NDArray[np.float64]]], ...]:
        """
        Connects the motion profiles of the segments in the trajectory.
        """
        def _connect_vel(*segments: Segment):
            """Connects the velocity profiles."""
            vel_x_lst, vel_y_lst, vel_z_lst = zip(*[
                segment.velocity_profiles
                for segment in segments
            ])
            vel_x_connected = _connect(*vel_x_lst)
            vel_y_connected = _connect(*vel_y_lst)
            vel_z_connected = _connect(*vel_z_lst)
            return vel_x_connected, vel_y_connected, vel_z_connected

        def _connect_pos(*segments: Segment):
            """Connects the position profiles."""
            pos_x_lst, pos_y_lst, pos_z_lst = zip(*[
                segment.position_profiles
                for segment in segments
            ])
            pos_x_connected = _connect(*pos_x_lst)
            pos_y_connected = _connect(*pos_y_lst)
            pos_z_connected = _connect(*pos_z_lst)
            return pos_x_connected, pos_y_connected, pos_z_connected

        def _connect_acc(*segments: Segment):
            """Connects the acceleration profiles."""
            acc_x_lst, acc_y_lst, acc_z_lst = zip(*[
                segment.acceleration_profiles
                for segment in segments
            ])
            acc_x_connected = _connect(*acc_x_lst)
            acc_y_connected = _connect(*acc_y_lst)
            acc_z_connected = _connect(*acc_z_lst)
            return acc_x_connected, acc_y_connected, acc_z_connected

        def _connect(*profiles):
            """Connects the profiles (either position, velocity, or 
            acceleration) of all segments in the trajectory.
            """
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

        pos_profile = _connect_pos(*self)
        pos_profile = {
            "x": {
                "time": pos_profile[0][0],
                "values": pos_profile[0][1]
            },
            "y": {
                "time": pos_profile[1][0],
                "values": pos_profile[1][1]
            },
            "z": {
                "time": pos_profile[2][0],
                "values": pos_profile[2][1]
            }
        }

        vel_profile = _connect_vel(*self)
        vel_profile = {
            "x": {
                "time": vel_profile[0][0],
                "values": vel_profile[0][1]
            },
            "y": {
                "time": vel_profile[1][0],
                "values": vel_profile[1][1]
            },
            "z": {
                "time": vel_profile[2][0],
                "values": vel_profile[2][1]
            },
        }

        acc_profile = _connect_acc(*self)
        acc_profile = {
            "x": {
                "time": acc_profile[0][0],
                "values": acc_profile[0][1]
            },
            "y": {
                "time": acc_profile[1][0],
                "values": acc_profile[1][1]
            },
            "z": {
                "time": acc_profile[2][0],
                "values": acc_profile[2][1]
            }
        }
        return pos_profile, vel_profile, acc_profile

    @property
    def motion_profiles(self) -> tuple[TMotionProfileDict, ...] | None:
        """
        Returns the connected position, velocity, and acceleration profiles of 
        all segments in the trajectory.

        Returns
        -------
        pos_profile : dict[str, float]
            Connected position profile of the trajectory.
        vel_profile : dict[str, float]
            Connected velocity profile of the trajectory.
        acc_profile : dict[str, float]
            Connected acceleration profile of the trajectory.

        The dictionaries all have the same structure:
        ```
        <profile> = {
            "x": {
                "time": <array of time values>,
                "values": <array of corresponding profile values>
            },
            "y": {
                "time": <array of time values>,
                "values": <array of corresponding profile values>
            },
            "z": {
                "time": <array of time values>,
                "values": <array of corresponding profile values>
            }
        }
        ```
        """
        if len(self) > 0:
            pos, vel, acc = self._connect_segments()
            return pos, vel, acc
        return None
    
    @property
    def duration(self) -> float:
        """
        Returns the total time to execute the trajectory.
        """
        dt_tot = sum(seg.mp_x.dt_tot for seg in self)
        return dt_tot

    def get_path_deviation(self) -> float:
        """
        Returns the average of the maximum path deviations in the segments of
        the trajectory.
        """
        devs = [seg.path_deviation for seg in self]
        dev_avg = sum(devs) / len(devs)
        return dev_avg

    def save(self, file_path: str) -> None:
        """
        Saves the `Trajectory` object `self` to the specified file using pickle.
        """
        with open(file_path, 'wb') as f:
            # noinspection PyTypeChecker
            pickle.dump(self, f)

    @classmethod
    def load(cls, file_path: str) -> 'Trajectory':
        """
        Loads (unpickles) a `Trajectory` object from the specified file.
        """
        with open(file_path, "rb") as f:
            trajectory = pickle.load(f)
            return trajectory

    def get_stepper_driver_signals(self) -> list[dict[str, Any]]:
        """
        Returns a list of dicts containing the stepper driver signals for each
        segment in the trajectory.
        """
        signals = [segment.stepper_driver_signals for segment in self]
        return signals

    def save_stepper_driver_signals(self, file_path) -> None:
        """
        Saves the list with stepper driver signals to a JSON file.
        """
        with open(file_path, "w") as f:
            json.dump(self.get_stepper_driver_signals(), f)

    @classmethod
    def load_stepper_driver_signals(cls, file_path) -> list[dict[str, Any]]:
        """
        Loads the list with stepper driver signals from the specified file
        path.
        """
        with open(file_path, "r") as f:
            signals = json.load(f)
            return signals


class TrajectoryPlanner:
    """
    Creates a `Trajectory` object. 

    A `Trajectory` object is a list of `Segment` objects. A `Segment`object 
    contains the motion profiles and rotation directions of the X-, Y-, and 
    Z-axis movements. It can also contain the stepper motor signals (actually 
    lists of time delays between successive step pulses) for driving the X-, Y-, 
    and Z-axis stepper motors.

    The `TrajectoryPlanner` generates the necessary data about the segments in
    a trajectory. It determines the boundary velocities of the segments (the X-,
    Y- and Z-components of the velocity at the start point and at the end point
    of each segment). Internally it uses a `MotionProcessor` object to create
    the final synchronized X-, Y-, and Z-axis motion profiles and to generate
    the step pulse signals for driving the X-, Y-, and Z-axis stepper motors.
    """
    def __init__(
        self, 
        pitch: float | tuple[float, float, float],
        profile_type: Type[MotionProfile],
        a_m: float | tuple[float, float, float],
        v_m: float | tuple[float, float, float],
        x_motor: StepperMotorMock | None = None,
        y_motor: StepperMotorMock | None = None,
        z_motor: StepperMotorMock | None = None
    ) -> None:
        """
        Creates a `TrajectoryPlanner` object.
        
        Parameters
        ----------
        pitch:
            Pitch of the lead screw (i.e. number of revolutions of the screw to
            move the nut one meter). Converts linear to rotational motion.
            If a single float is given, this value is used for all two or three 
            motion axes. If the axes have different pitches, a tuple (x, y, z) 
            can be passed.
        profile_type:
            Concrete subclass of `MotionProfile`. Either `TrapzoidalProfile`, or
            `SCurvedProfile` are supported.
        a_m:
            Maximum or available angular acceleration of the motor (deg/s²). 
            This value will depend on the torque capabilities of the motor and 
            on the total inertia moment of the motor and the connected load.
            If a single float is given, this value is used for all two or three 
            motors. If the axes have different motors, a tuple (x, y, z) can be 
            passed.
        v_m:
            Maximum or available angular speed of the motor (deg/s). If a single
            float is given, this value is used for all two or three motors. If 
            the axes have different motors, a tuple (x, y, z) can be passed.
        x_motor: optional
            A mock for the stepper motor of the X-axis. Used for generating the
            step signal of the X-axis stepper motor. It holds the number of 
            full steps to make one revolution of the motor shaft 
            (`full_steps_per_rev`) and, in case microstepping is used, the 
            microstep factor, which is the inverse of the microstep resolution. 
        y_motor: optional
            A mock for the stepper motor of the Y-axis.
        z_motor: optional
            A mock for the stepper motor of the Z-axis.
        """
        self.profile_type = profile_type
        if not isinstance(pitch, tuple):
            self.xpitch = self.ypitch = self.zpitch = pitch
        else:
            self.xpitch, self.ypitch, self.zpitch = pitch
        if not isinstance(a_m, tuple):
            self.ax_m = self.ay_m = self.az_m = a_m
        else:
            self.ax_m, self.ay_m, self.az_m = a_m
        if not isinstance(v_m, tuple):
            self.vx_m = self.vy_m = self.vz_m = v_m
        else:
            self.vx_m, self.vy_m, self.vz_m = v_m
        self.x_motor = x_motor
        self.y_motor = y_motor
        self.z_motor = z_motor
        self.motion_processor = MotionProcessor(self)
        self._segmentdata_lst: list[SegmentData] = []
    
    @staticmethod
    def _check_dimension(*points: TPoint) -> list[TPoint]:
        """
        Checks whether points have two or three coordinates. If points only
        have two coordinates (x and y), a z-coordinate with a value of 0.0 is
        added.
        """
        points = list(points)
        for i, point in enumerate(points):
            if len(point) == 2:
                point = list(point)
                point.append(0.0)
                point = tuple(point)
                # noinspection PyTypeChecker
                points[i] = point
        return points
    
    def _get_angular_displacements(
        self, 
        segment: TPointPair
    ) -> tuple[tuple[float, ...], tuple[RotationDirection | None, ...]]:
        """
        Calculates the angular displacements of the X-, Y- and Z-axis motor 
        that correspond with the linear displacements of a segment. Also
        determines the rotation direction of the motors.
        """
        xi = segment[0][0]
        yi = segment[0][1]
        zi = segment[0][2]
        xf = segment[1][0]
        yf = segment[1][1]
        zf = segment[1][2]
        dx = xf - xi
        dy = yf - yi
        dz = zf - zi
        dtheta_x = abs(round(360.0 * self.xpitch * dx, 6))
        dtheta_y = abs(round(360.0 * self.ypitch * dy, 6))
        dtheta_z = abs(round(360.0 * self.zpitch * dz, 6))
        if dx >= 0.0:
            rdir_x = RotationDirection.COUNTERCLOCKWISE
        else: 
            rdir_x = RotationDirection.CLOCKWISE
        if dy >= 0.0:
            rdir_y = RotationDirection.COUNTERCLOCKWISE
        else: 
            rdir_y = RotationDirection.CLOCKWISE
        if dz >= 0.0:
            rdir_z = RotationDirection.COUNTERCLOCKWISE
        else: 
            rdir_z = RotationDirection.CLOCKWISE
        return (dtheta_x, dtheta_y, dtheta_z), (rdir_x, rdir_y, rdir_z)
    
    def _get_initial_angles(self, segment: TPointPair) -> tuple[float, float, float]:
        """
        Calculates the angular positions of the X-, Y- and Z-axis motor shafts
        at the start point of a segment.
        """
        xi = segment[0][0]
        yi = segment[0][1]
        zi = segment[0][2]
        theta_xi = 360.0 * self.xpitch * xi
        theta_yi = 360.0 * self.ypitch * yi
        theta_zi = 360.0 * self.zpitch * zi
        return theta_xi, theta_yi, theta_zi
    
    def _create_segmentdata(self, pts: list[TPoint]) -> None:
        """
        Receiving the points that define the segments of the trajectory, it
        assembles the necessary data into `SegmentData` objects for planning the
        individual axis movements (angular displacement of the axes, rotation
        direction of the axes, initial angular position of the axes at the start
        point of each segment). Sets the velocity of the axes to zero at the
        start and at the end point of the trajectory.
        """
        segments = list(zip(pts[:-1], pts[1:]))
        i_max = len(segments) - 1
        self._segmentdata_lst = []
        for i, segment in enumerate(segments):
            dtheta, rdir = self._get_angular_displacements(segment)
            theta_i = self._get_initial_angles(segment)
            segmentdata = SegmentData(
                point_pair=segment,
                dtheta_x=dtheta[0], dtheta_y=dtheta[1], dtheta_z=dtheta[2],
                rdir_x=rdir[0], rdir_y=rdir[1], rdir_z=rdir[2],
                theta_xi=theta_i[0], theta_yi=theta_i[1], theta_zi=theta_i[2],
                profile_type=self.profile_type,
                xpitch = self.xpitch, ypitch = self.ypitch, zpitch = self.zpitch,
                omega_xm=self.vx_m, omega_ym=self.vy_m, omega_zm = self.vz_m,
                alpha_xm = self.ax_m, alpha_ym = self.ay_m, alpha_zm = self.az_m
            )
            # start and end point of the trajectory have zero velocities
            if i == 0:
                segmentdata.omega_xi = 0.0
                segmentdata.omega_yi = 0.0
                segmentdata.omega_zi = 0.0
            if i == i_max:
                segmentdata.omega_xf = 0.0
                segmentdata.omega_yf = 0.0
                segmentdata.omega_zf = 0.0
            self._segmentdata_lst.append(segmentdata)

    def _set_zero_velocities(self) -> None:
        """
        Sets the start and end velocity to zero for non-moving axes in segments
        of the trajectory. It also sets the end velocity of a moving axis to
        zero when the rotation direction reverses between two segments (the
        start velocity of the next segment is then also set to zero).
        """
        i_max = len(self._segmentdata_lst) - 1
        i = 0
        for segmentdata1 in self._segmentdata_lst:
            if i < i_max:
                segmentdata2 = self._segmentdata_lst[i + 1]
                for ax in ("x", "y", "z"):
                    axisdata1 = segmentdata1.get_axis_data(ax)
                    axisdata2 = segmentdata2.get_axis_data(ax)
                    dtheta_1 = axisdata1.dtheta
                    dtheta_2 = axisdata2.dtheta
                    rdir_1 = int(axisdata1.rdir)
                    rdir_2 = int(axisdata2.rdir)
                    if dtheta_1 == 0.0:
                        axisdata1.omega_i = 0.0
                        axisdata1.omega_f = 0.0
                        axisdata2.omega_i = 0.0
                    if dtheta_2 == 0.0 or rdir_1 * rdir_2 < 0.0:
                        axisdata1.omega_f = 0.0
                        axisdata2.omega_i = 0.0
                    segmentdata1.set_axis_data(ax, axisdata1)
                    segmentdata2.set_axis_data(ax, axisdata2)
            i += 1

    @staticmethod
    def _minimize_axis_profile_time(
        axisdata: AxisData,
        v_other: float,
        find: str
    ) -> MotionProfile:
        """
        Creates and returns a motion profile with either an optimized start or
        end velocity so that the required axis displacement would be executed in
        the shortest possible travel time.

        Parameters
        ----------
        axisdata:
            The data of the axis for which the motion profile needs to be
            created.
        v_other:
            If the optimal start velocity of the axis is to be determined,
            `v_other` is the value of the final velocity of this axis.
            Conversely, if the optimal end velocity is to be determined, it is
            the value of the start velocity.
        find: {"v_i", "v_f"}
            Indicates which velocity is to be determined: "v_i" refers to the
            start velocity, while "v_f" refers to the end velocity of the axis.
        """
        # If the initial or final velocity was already determined to be zero,
        # then we cannot change this requirement.
        c1 = find == "v_i" and axisdata.omega_i == 0.0
        c2 = find == "v_f" and axisdata.omega_f == 0.0
        if c1 or c2:
            mp = axisdata.profile_type(
                ds_tot=axisdata.dtheta,
                a_m=axisdata.alpha_m,
                v_m=axisdata.omega_m,
                v_i=axisdata.omega_i,
                v_f=axisdata.omega_f
            )
            return mp

        def fn(v: float) -> float:
            try:
                kwargs = (
                    {"v_i": v, "v_f": v_other} if find == "v_i"
                    else {"v_i": v_other, "v_f": v}
                )
                mp = axisdata.profile_type(
                    ds_tot=axisdata.dtheta,
                    a_m=axisdata.alpha_m,
                    v_m=axisdata.omega_m,
                    **kwargs
                )
                return mp.dt_tot
            except ValueError:
                return float("inf")

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            max_speed = min(axisdata.omega_m, np.sqrt(2 * axisdata.alpha_m * axisdata.dtheta))
            r = minimize_scalar(fn, bounds=(1.0, max_speed), method="bounded")
        if r.success:
            kwargs = (
                {"v_i": r.x, "v_f": v_other} if find == "v_i"
                else {"v_i": v_other, "v_f": r.x}
            )
            mp = axisdata.profile_type(
                ds_tot=axisdata.dtheta,
                a_m=axisdata.alpha_m,
                v_m=axisdata.omega_m,
                **kwargs
            )
            return mp
        raise ValueError(f"{find} could not be determined")
    
    @staticmethod    
    def _synchronize_motion_profiles(
        motion_profiles: dict[str, MotionProfile],
        find: str
    ) -> dict[str, MotionProfile]:
        """
        Determines the motion profile with the longest travel time and recreates
        the motion profiles of the other moving axes so that their travel time
        would equal the longest travel time (time synchronization)

        Parameters
        ----------
        motion_profiles:
            Dictionary with the motion profiles of the moving axes in a segment.
            The keys of the dictionary indicate to which axis each motion
            profile belongs.
        find: {"v_i", "v_f"}
            Indicates whether the start or end velocities of segments in the
            trajectory are being determined.

        Returns
        -------
        Dictionary with the synchronized motion profiles of the moving axes in
        the segment.
        """
        if len(motion_profiles) > 1:
            k_ref, mp_ref = max(motion_profiles.items(), key=lambda t: t[1].dt_tot)
            l = [(k, mp) for k, mp in motion_profiles.items() if mp is not mp_ref]
            for j, (k, mp) in enumerate(l):
                if find == "v_i":
                    mp = type(mp)(
                        ds_tot=mp.ds_tot,
                        a_m=mp.a_m,
                        v_m=mp.v_m,
                        v_i=None if mp.v_i != 0.0 else 0.0,
                        # don't use the initial velocity from the previous step, except if 
                        # it's zero -> allows the initial velocity still to change when
                        # the synchronized motion profile `mp` is (re)created here.
                        v_f=mp.v_f,
                        s_i=mp.s_i,
                        dt_tot=mp_ref.dt_tot
                    )
                if find == "v_f":
                    mp = type(mp)(
                        ds_tot=mp.ds_tot,
                        a_m=mp.a_m,
                        v_m=mp.v_m,
                        v_i=mp.v_i,
                        v_f=None if mp.v_f != 0.0 else 0.0,
                        s_i=mp.s_i,
                        dt_tot=mp_ref.dt_tot
                    )
                l[j] = (k, mp)
            l.append((k_ref, mp_ref))
            for k, mp in l: motion_profiles[k] = mp
        return motion_profiles
    
    def _minimize_segment_profile_time(
        self,
        moving_axes: dict[str, AxisData],
        find: str,
    ) -> dict[str, MotionProfile]:
        """
        Creates and returns the motion profiles for the moving axes in a segment
        of the trajectory.

        It follows a two-step procedure. First, the motion profiles of the
        moving axes are determined independently of each other so that their
        movement can be accomplished in the shortest possible travel time. Next,
        it is determined which axis has the longest travel time. The motion
        profiles of the other moving axes are then recreated so that their
        travel time would also equal the longest travel time in the segment.

        Parameters
        ----------
        moving_axes:
            Dictionary with the data of the moving axes in the segment. The keys
            of the dictionary indicate to which axis the data belongs.
        find: {"v_i", "v_f"}
            Indicates whether the start or end velocities of segments in the
            trajectory are being determined.

        Returns
        -------
        Dictionary with the motion profiles of the moving axes in a segment. The
        keys of the dictionary indicate to which axis each motion profile
        belongs.
        """
        # For each of the moving axes, create a motion profile that executes the
        # required displacement in the shortest possible travel time by altering
        # either the start or the end velocity of the axis movement in the 
        # segment.
        motion_profiles: dict[str, MotionProfile] = {}
        for k, axisdata in moving_axes.items():
            v_other = axisdata.omega_f if find == "v_i" else axisdata.omega_i
            mp = self._minimize_axis_profile_time(axisdata, v_other, find)
            motion_profiles[k] = mp
        
        # After the motion profile of each moving axis is created to execute its
        # own displacement in the shortest possible travel time, we need to 
        # synchronize these motion profiles so they all share the same total
        # travel time, equal to the maximum of the individual travel times.
        motion_profiles = self._synchronize_motion_profiles(motion_profiles, find)
        return motion_profiles        
    
    @staticmethod
    def _get_path_deviation(
        moving_axes: dict[str, AxisData],
        num_points: int = 100
    ) -> float:
        """
        Calculates and returns the maximum deviation between the actual segment
        path (which depends on the motion profiles of the individual axis
        movements) and the straight line between start and end point of the
        segment.

        Parameters
        ----------
        moving_axes:
            Dictionary with the data of the moving axes in the segment. The keys
            of the dictionary indicate to which axis the data belongs.
        num_points:
            To determine the maximum path deviation, a number of equally spaced
            points on the position profiles of the axis movements is taken.
            `num_points` indicates how many points are taken. The default is 100.
        """
        def get_axis_positions(
            moving_axes: dict[str, AxisData],
            num_points: int = 100
        ) -> list[NDArray[np.float64]]:
            """
            Returns the coordinates of points on the actual segment path. The
            returned coordinates (Numpy arrays) are separated per axis.
            """
            positions: list[NDArray[np.float64]] = []
            for axisdata in moving_axes.values():
                t_arr = np.linspace(0.0, axisdata.mp.dt_tot, num_points)
                theta_fn = np.vectorize(axisdata.mp.get_position_time_fn())
                s_arr = theta_fn(t_arr) / (360.0 * axisdata.pitch)
                s0_arr = np.full_like(s_arr, s_arr[0])
                ds_arr = (s_arr - s0_arr) * axisdata.rdir.to_int()
                s_arr = s0_arr + ds_arr
                positions.append(s_arr)
            return positions

        # Get the coordinates of points on the actual segment path.
        axis_positions = get_axis_positions(moving_axes, num_points)
        
        # Get the start- and end point of the segment.
        p0 = np.array([s_arr[0] for s_arr in axis_positions])
        p1 = np.array([s_arr[-1] for s_arr in axis_positions])
        dp = p1 - p0
        dp_norm_sq = np.dot(dp, dp)

        # Calculate the deviation of the points on the actual segment path to the
        # straight line segment and return the maximum of these deviations.
        point_coords = np.vstack(axis_positions)
        _, num_cols = point_coords.shape
        max_dev = 0.0
        for i in range(num_cols):
            p = point_coords[:, i]
            alpha = np.dot(p - p0, dp) / dp_norm_sq
            proj = p0 + alpha * dp
            deviation = np.linalg.norm(p - proj)
            max_dev = max(max_dev, deviation)
        return max_dev

    def _minimize_segment_path_deviation(
        self,
        moving_axes: dict[str, AxisData],
        find: str,
        allowed_path_deviation: float = 1.e-3,
        num_points: int = 100
    ) -> dict[str, MotionProfile]:
        """
        Calculates and returns the motion profiles of the moving axes in the
        segment.

        The routine uses Scipy `minimize_scalar(...)` to determine the maximum
        allowable velocity of the moving axes for which the segment path
        deviation is closest to the specified allowable value (this doesn't mean
        that the segment path deviation will be equal or smaller than the
        allowable value).
        The "cost function" internally uses `_minimize_segment_profile_time(...)`
        to determine the motion profiles of the moving axes at a certain maximum
        velocity. Then, the segment path deviation is calculated. The "cost
        function" is repeatedly executed by `minimize_scalar(...)` with
        different values for the maximum axis velocity. Finally, the value of
        the maximum velocity that results in the smallest difference with the
        allowed segment path deviation is returned by `minimize_scalar(...)`.

        Parameters
        ----------
        moving_axes:
            Dictionary with the data of the moving axes in the segment. The keys
            of the dictionary indicate to which axis the data belongs.
        find: {"v_i", "v_f"}
            Indicates whether the start or end velocities of the segments in the
            trajectory are being determined.
        allowed_path_deviation
            The allowable (maximum) segment path deviation in linear axis
            displacement units (this will depend on the units that were used to
            specify axis pitch, e.g. if pitch is in revs/meter, the deviation
            is expected to be in meters).
        num_points
            The number of points on the segment path that is used for
            calculating the segment path deviation. The default is 100.

        Returns
        -------
        Dictionary with the motion profiles of the moving axes in a segment. The
        keys of the dictionary indicate to which axis each motion profile
        belongs.
        """
        def fn(omega_m: float) -> float:
            try:
                for axisdata in moving_axes.values(): axisdata.omega_m = omega_m
                motion_profiles = self._minimize_segment_profile_time(moving_axes, find)
                for k, mp in motion_profiles.items(): moving_axes[k].mp = mp
                path_deviation = self._get_path_deviation(moving_axes, num_points)
                delta = allowed_path_deviation - path_deviation
                return delta ** 2
            except ValueError:
                return 1e10

        upper_bound = max(self.vx_m, self.vy_m, self.vz_m)
        result = minimize_scalar(fn, bounds=(1.0, upper_bound), method="bounded")
        if result.success:
            for k, axisdata in moving_axes.items():
                axisdata.omega_m = result.x
            motion_profiles = self._minimize_segment_profile_time(moving_axes, find)
            return motion_profiles
        raise ValueError(f"Optimization of path deviation failed: {result.message}")

    def _create_motion_profiles(
        self,
        segmentdata: SegmentData,
        find: str,
        optimize: bool = False,
        allowed_path_deviation: float = 1.e-3,
        num_points: int = 100
    ) -> dict[str, MotionProfile]:
        """
        Creates the motion profiles of the moving axes in a segment of the
        trajectory.

        Parameters
        ----------
        segmentdata:
            `SegmentData` object with the "motion data" of the axes in the
            current segment.
        find: {"v_i", "v_f"}
            Indicates whether the start or end velocities of the segments in the
            trajectory are being determined.
        optimize:
            Indicates if the calculation of the motion profiles needs to
            consider minimization of the segment path deviation.
        allowed_path_deviation:
            The allowable (maximum) segment path deviation in linear axis
            displacement units (this will depend on the units that were used to
            specify axis pitch, e.g. if pitch is in revs/meter, the deviation
            is expected to be in meters).
        num_points:
            The number of points on the segment path that is used for
            calculating the segment path deviation. The default is 100.

        Returns
        -------
        Dictionary with the motion profiles of the moving axes in a segment. The
        keys of the dictionary indicate to which axis each motion profile
        belongs.
        """
        # Seperate any non-moving axes from the moving axes in the segment.
        moving_axes: dict[str, AxisData] = {}
        xaxisdata = segmentdata.get_xaxis_data()
        yaxisdata = segmentdata.get_yaxis_data()
        zaxisdata = segmentdata.get_zaxis_data()
        
        for t in zip(("x", "y", "z"), (xaxisdata, yaxisdata, zaxisdata)):
            if t[1].dtheta > 0.0:
                moving_axes[t[0]] = t[1]
        
        if len(moving_axes) == 1:
            k, axisdata = moving_axes.popitem()
            v_other = axisdata.omega_f if find == "v_i" else axisdata.omega_i
            motion_profile = self._minimize_axis_profile_time(axisdata, v_other, find)
            return {k: motion_profile}
        
        if optimize:
            motion_profiles = self._minimize_segment_path_deviation(
                moving_axes, 
                find, 
                allowed_path_deviation, 
                num_points
            )
        else:
            motion_profiles = self._minimize_segment_profile_time(
                moving_axes, 
                find
            )
        return motion_profiles

    def _set_start_velocities(
        self,
        optimize: bool = False,
        allowed_path_deviation: float = 1.e-3,
        num_points: int = 100
    ) -> None:
        """
        Traverses the segments in the trajectory in reversed order (from end to
        start). Creates the motion profiles for the moving axes in each segment.
        The start velocity obtained for each of the axes is stored in the
        corresponding `SegmentData` object. This start velocity is also assigned
        as the final velocity of the same axis in the preceding segment of the
        trajectory.

        This function must be called before `_set_end_velocities()`.

        Parameters
        ----------
        optimize:
            Indicates if the calculation of the motion profiles needs to
            consider minimization of the segment path deviation.
        allowed_path_deviation:
            The allowable (maximum) segment path deviation in linear axis
            displacement units (this will depend on the units that were used to
            specify axis pitch, e.g. if pitch is in revs/meter, the deviation
            is expected to be in meters).
        num_points:
            The number of points on the segment path that is used for
            calculating the segment path deviation. The default is 100.
        """
        i_max = len(self._segmentdata_lst) - 1
        i = i_max
        for _ in range(len(self._segmentdata_lst)):
            if i > 0:
                segmentdata1 = self._segmentdata_lst[i]
                motion_profiles1 = self._create_motion_profiles(
                    segmentdata1,
                    find="v_i",
                    optimize=optimize,
                    allowed_path_deviation=allowed_path_deviation,
                    num_points=num_points
                )
                segmentdata2 = self._segmentdata_lst[i - 1]
                for k, mp in motion_profiles1.items():
                    v_i = mp.v_i if mp.v_i is not None else mp.v_top
                    # if `mp.v_i` is still None, it means `mp` was synchronized 
                    # in `_create_motion_profiles()`; in that case the start
                    # velocity `v_i` equals the top velocity `v_top`. 
                    if k == "x":
                        segmentdata1.omega_xi = v_i
                        segmentdata2.omega_xf = v_i
                    if k == "y":
                        segmentdata1.omega_yi = v_i
                        segmentdata2.omega_yf = v_i
                    if k == "z":
                        segmentdata1.omega_zi = v_i
                        segmentdata2.omega_zf = v_i
            i -= 1
    
    def _set_end_velocities(
        self,
        optimize: bool = False,
        allowed_path_deviation: float = 1.e-3,
        num_points: int = 100
    ) -> None:
        """
        Traverses the segments in the trajectory from start to end. Creates
        motion profiles for the moving axes in each segment. It then checks
        whether the obtained final velocity for each axis in a segment is
        smaller or not than the (with `_set_start_velocities(...)`) previously
        determined value held in the corresponding `SegmentData` object of the
        segment. Eventually, the smallest value is retained in the `SegmentData`
        object.

        This function must be called after `_set_start_velocities()`.

        Parameters
        ----------
        optimize:
            Indicates if the calculation of the motion profiles needs to
            consider minimization of the segment path deviation.
        allowed_path_deviation:
            The allowable (maximum) segment path deviation in linear axis
            displacement units (this will depend on the units that were used to
            specify axis pitch, e.g. if pitch is in revs/meter, the deviation
            is expected to be in meters).
        num_points:
            The number of points on the segment path that is used for
            calculating the segment path deviation. The default is 100.
        """
        i_max = len(self._segmentdata_lst) - 1
        for i, segmentdata1 in enumerate(self._segmentdata_lst):
            if i < i_max:
                motion_profiles1 = self._create_motion_profiles(
                    segmentdata1,
                    find="v_f",
                    optimize=optimize,
                    allowed_path_deviation=allowed_path_deviation,
                    num_points=num_points
                )
                segmentdata2 = self._segmentdata_lst[i + 1]
                for k, mp in motion_profiles1.items():
                    v_f = mp.v_f if mp.v_f is not None else mp.v_top
                    if k == "x":
                        v_f_min = min(v_f, segmentdata1.omega_xf)
                        segmentdata1.omega_xf = v_f_min
                        segmentdata2.omega_xi = v_f_min
                        segmentdata1.omega_xm = mp.v_m
                    if k == "y":
                        v_f_min = min(v_f, segmentdata1.omega_yf)
                        segmentdata1.omega_yf = v_f_min
                        segmentdata2.omega_yi = v_f_min
                        segmentdata1.omega_ym = mp.v_m
                    if k == "z":
                        v_f_min = min(v_f, segmentdata1.omega_zf)
                        segmentdata1.omega_zf = v_f_min
                        segmentdata2.omega_zi = v_f_min
                        segmentdata1.omega_zm = mp.v_m

    def _create_trajectory(
        self,
        optimize: bool = False,
        allowed_path_deviation: float = 1e-3,
        num_points: int = 100
    ) -> Trajectory:
        """
        Creates and returns a `Trajectory` object (list of `Segment` objects).

        Parameters
        ----------
        optimize:
            Indicates if the calculation of the motion profiles needs to
            consider minimization of the segment path deviation.
        allowed_path_deviation:
            The allowable (maximum) segment path deviation in linear axis
            displacement units (this will depend on the units that were used to
            specify axis pitch, e.g. if pitch is in revs/meter, the deviation
            is expected to be in meters).
        num_points:
            The number of points on the segment path that is used for
            calculating the segment path deviation. The default is 100.
        """
        # Calculate optimized start and end velocities for the moving axes in
        # each segment of the trajectory.
        self._set_start_velocities(optimize, allowed_path_deviation, num_points)
        self._set_end_velocities(optimize, allowed_path_deviation, num_points)
        
        # Create `Segment` objects from the `SegmentData` objects.
        trajectory = Trajectory()
        for segmentdata in self._segmentdata_lst:
            segment = self.motion_processor.create_segment(segmentdata)
            trajectory.append(segment)
        return trajectory
    
    def get_trajectory(
        self,
        *points: TPoint,
        **kwargs
    ) -> Trajectory:
        """
        Creates and returns a `Trajectory` object (list of `Segment` objects)
        from the given vertices of the trajectory.

        Parameters
        ----------
        *points:
            A sequence of points, either as (x, y) or (x, y, z) tuples.
        **kwargs:
            optimize:
                Indicates if the calculation of the motion profiles needs to
                consider minimization of the segment path deviation.
            allowed_path_deviation:
                The allowable (maximum) segment path deviation in linear axis
                displacement units (this will depend on the units that were used to
                specify axis pitch, e.g. if pitch is in revs/meter, the deviation
                is expected to be in meters).
            num_points:
                The number of points on the segment path that is used for
                calculating the segment path deviation. The default is 100.

        Returns
        -------
        Trajectory
        """
        optimize = kwargs.get("optimize", False)
        allowed_path_deviation = kwargs.get("allowed_path_deviation", 1e-3)
        num_points = kwargs.get("num_points", 100)

        points = self._check_dimension(*points)
        self._create_segmentdata(points)
        self._set_zero_velocities()

        trajectory = self._create_trajectory(optimize, allowed_path_deviation, num_points)
        return trajectory
    
    def get_segmentdata(self) -> list[SegmentData]:
        """
        Returns the `SegmentData` objects created by the `TrajectoryPlanner`.
        """
        return self._segmentdata_lst

    def _reset_trajectory_kinematics(
        self,
        a_m: float | tuple[float, float, float],
        v_m: float | tuple[float, float, float]
    ) -> None:
        """
        Sets the maximum acceleration and velocity of the axes to the specified
        new values to recalculate the trajectory.
        """
        # Maximum acceleration
        if not isinstance(a_m, tuple):
            self.ax_m = self.ay_m = self.az_m = a_m
        else:
            self.ax_m, self.ay_m, self.az_m = a_m
        # Maximum velocity
        if not isinstance(v_m, tuple):
            self.vx_m = self.vy_m = self.vz_m = v_m
        else:
            self.vx_m, self.vy_m, self.vz_m = v_m
        # Update segments
        for segmentdata in self._segmentdata_lst:
            segmentdata.alpha_xm = self.ax_m
            segmentdata.alpha_ym = self.ay_m
            segmentdata.alpha_zm = self.az_m
            segmentdata.omega_xm = self.vx_m
            segmentdata.omega_ym = self.vy_m
            segmentdata.omega_zm = self.vz_m

    def recreate_trajectory(
        self,
        a_m: float | tuple[float, float, float],
        v_m: float | tuple[float, float, float],
        **kwargs
    ) -> Trajectory:
        """
        Creates and returns a new trajectory with the new values for the maximum
        acceleration and velocity of the axes.

        Parameters
        ----------
        a_m:
            Maximum or available angular acceleration of the motor (deg/s²).
            This value will depend on the torque capabilities of the motor and
            on the total inertia moment of the motor and the connected load.
            If a single float is given, this value is used for all two or three
            motors. If the axes have different motors, a tuple (x, y, z) can be
            passed.
        v_m:
            Maximum or available angular speed of the motor (deg/s). If a single
            float is given, this value is used for all two or three motors. If
            the axes have different motors, a tuple (x, y, z) can be passed.
        **kwargs:
            optimize:
                Indicates if the calculation of the motion profiles needs to
                consider minimization of the segment path deviation.
            allowed_path_deviation:
                The allowable (maximum) segment path deviation in linear axis
                displacement units (this will depend on the units that were used to
                specify axis pitch, e.g. if pitch is in revs/meter, the deviation
                is expected to be in meters).
            num_points:
                The number of points on the segment path that is used for
                calculating the segment path deviation. The default is 100.

        Returns
        -------
        Trajectory
        """
        if not self._segmentdata_lst:
            raise ValueError("No trajectory was created before.")

        optimize = kwargs.get("optimize", False)
        allowed_path_deviation = kwargs.get("allowed_path_deviation", 1e-3)
        num_points = kwargs.get("num_points", 100)

        self._reset_trajectory_kinematics(a_m, v_m)
        trajectory = self._create_trajectory(optimize, allowed_path_deviation, num_points)
        return trajectory

    def optimize_trajectory(self, allow_dev: float) -> Trajectory:
        """
        Recreates the trajectory while reducing the global path deviation of the
        trajectory (i.e. the average of the maximum segment path deviations) as
        close as possible to `allow_dev`.
        """
        def fn(v_m: float) -> float:
            try:
                trajectory = self.recreate_trajectory(self.ax_m, v_m)
                dev = trajectory.get_path_deviation()
                delta = allow_dev - dev
                return delta ** 2
            except:
                return 1e10

        result = minimize_scalar(fn, bounds=(1.0, self.vx_m), method="bounded")
        if result.success:
            v_m = result.x
            trajectory = self.recreate_trajectory(self.ax_m, v_m)
            return trajectory
        if not result.success:
            raise ValueError(
                f"Could not find a suitable value for maximum velocity: "
                f"{result.message}."
            )
        raise ValueError("Could not find a suitable value for maximum velocity.")
