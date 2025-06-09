"""
Implements the `TrajectoryPlanner` class and its helper classes. Use the
`TrajectoryPlanner` class to create 2D- or 3D-trajectories with rectilinear
segements.
"""
import warnings
from typing import Type
from dataclasses import dataclass, field, fields

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import minimize_scalar

from pyberryplc.motion.multi_axis import MotionProfile, RotationDirection
from pyberryplc.stepper.driver.process import TwoStageProfileRotator


TPoint = tuple[float, float] | tuple[float, float, float]
TPointPair = tuple[TPoint, TPoint]
TProfile = tuple[NDArray[np.float64], NDArray[np.float64]]
TSegmentPath = tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]
TMotionProfileDict = dict[str, dict[str, NDArray[np.float64]]]


@dataclass
class AxisData:
    """
    Dataclass used by `SegementData` to return the data for a single axis
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
    Intermediate dataclass that holds data about the segments in a trajectory
    that is needed to generate the motion profiles and stepper motor signals 
    for the X-, Y-, and/or Z-axis movements. 
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
    Dataclass that has attributes to draw and analyze the generated motion 
    profiles of a segment in the trajectory, and holds the step pulse signals to 
    drive the stepper motors of the X-, Y-, and/or Z-axis.
    
    Attributes
    ----------
    point_pair:
        The start and endpoint of the segment.
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
    path_deviation: float = 0.0
    
    def __str__(self):
        lines = []
        for field in fields(self):
            value = getattr(self, field.name)
            lines.append(f"{field.name}: {value}")
        return "\n".join(lines)


class StepperMotorMock:
    """
    This class is only intended to mock a `StepperMotor` to be able to use the 
    function `preprocess()` of class `TwoStageProfileRotator(ProfileRotator)` 
    for calculating the time delays between successive step pulses that drive a 
    stepper motor.
    """
    def __init__(
        self,
        full_steps_per_rev: int = 200,
        microstep_factor: int = 1
    ) -> None:
        """
        Creates a `StepperMotorMock` object having just the attributes needed 
        to generate the step pulse train (list of the time delays between 
        successive step pulses).

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
    Used internally by `TrajectoryPlanner`.
    
    It generates the motion profiles along the X-, Y-, and Z-axis to move from 
    one point to the next along a straight segment (i.e. X-, Y-, and 
    Z-movements are synchronized).
    
    The step pulses to drive the X-, Y- and Z-axis stepper motors, can also be 
    generated, if `StepperMotorMock` objects were passed to the constructor of
    `TrajectoryPlanner` object.
    """
    def __init__(self, planner: 'TrajectoryPlanner' = None) -> None:
        """
        Creates a `MotionProcessor` object. This object is internally created
        in a `TrajectoryPlanner` object.
        """
        self.planner = planner
        self._segmentdata: SegmentData | None = None
        self._mp_x: MotionProfile | None = None
        self._mp_y: MotionProfile | None = None
        self._mp_z: MotionProfile | None = None
        self._x_rotator: TwoStageProfileRotator | None = None
        self._y_rotator: TwoStageProfileRotator | None = None
        self._z_rotator: TwoStageProfileRotator | None = None
        self._x_delays: list[float] | None = None
        self._y_delays: list[float] | None = None
        self._z_delays: list[float] | None = None
        if self.planner is not None:
            if self.planner.x_motor is not None:
                # noinspection PyTypeChecker
                self._x_rotator = TwoStageProfileRotator(self.planner.x_motor)
            if self.planner.y_motor is not None:
                # noinspection PyTypeChecker
                self._y_rotator = TwoStageProfileRotator(self.planner.y_motor)
            if self.planner.z_motor is not None:
                # noinspection PyTypeChecker
                self._z_rotator = TwoStageProfileRotator(self.planner.z_motor)
    
    def _create_motion_profile(
        self,
        axis: str,
        dt_tot: float | None = None
    ) -> MotionProfile:
        """
        Takes the segment data of the indicated axis and creates a motion 
        profile for that axis.
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
        Synchronizes the motion profiles of the X-, Y-, and Z-axis movements.
        Checks which of the motion profiles takes the most time, and changes the
        top velocity of the other motion profiles so all axis movements finish 
        at the same time moment.
        """
        # Only consider the axes that need to move.
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

        # Take the moving axis that needs the longest travel time; this must 
        # also become the travel time of the other moving axes:
        dt_tot_dict = {axis: mp.dt_tot for axis, mp in moving.items()}
        axis_with_dt_max = max(dt_tot_dict, key=dt_tot_dict.get)
        dt_max = dt_tot_dict[axis_with_dt_max]

        # Set the travel time of non-moving axes to `dt_max` (this is needed for
        # plotting the motion profile of non-moving axes)
        for mp in non_moving: mp.dt_tot = mp.dt_cov = dt_max

        # The axis with the longest travel time remains unchanged and can be
        # omitted for now.
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
        mp_x = self._create_motion_profile("x")
        mp_y = self._create_motion_profile("y")
        mp_z = self._create_motion_profile("z")
        mp_x, mp_y, mp_z = self._synchronize_motion_profiles(mp_x, mp_y, mp_z)
        return mp_x, mp_y, mp_z
    
    def _get_position_profiles(self) -> tuple[TProfile, TProfile, TProfile]:
        
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
    
    def _calc_delays(self):
        """
        Calculates the step pulse signals to drive the axis stepper motors
        based on the calculated motion profiles for the axes.
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
    
    def create_segment(
        self, 
        segmentdata: SegmentData, 
        num_points: int = 100
    ) -> Segment:
        """
        Creates a `Segment` object from the `SegmentData` object. Calculates
        the motion profiles of the axes and the step pulse signals to drive
        the stepper motors in the segment.
        """
        self._segmentdata = segmentdata
        self._mp_x, self._mp_y, self._mp_z = self._create_motion_profiles()
        self._calc_delays()

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
        segment.path_deviation = self.planner._get_path_deviation(segment, num_points)
        return segment


class Trajectory(list[Segment]):
    """
    Represents a 3D trajectory in XYZ-space. A `Trajectory` object is a list of 
    `Segment` objects.
    """
    def _connect_segments(self) -> tuple[dict[str, dict[str, NDArray[np.float64]]], ...]:
        """
        Connects the motion profiles of the successive segments in the trajectory.
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
        the segments in the trajectory.

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
        dt_tot = sum(seg.mp_x.dt_tot for seg in self)
        return dt_tot

    def get_path_deviation(self) -> float:
        """
        Calculates the maximum path deviation per segment of the trajectory and
        returns the average of these deviations.
        """
        devs = [seg.path_deviation for seg in self]
        dev_avg = sum(devs) / len(devs)
        return dev_avg


class TrajectoryPlanner:
    """
    Creates a `Trajectory` object. 

    A `Trajectory` object is a list of `Segment` objects. A `Segment`object 
    contains the motion profiles and rotation directions of the X-, Y-, and 
    Z-axis movements. It can also contain the step signals (lists of time delays
    between successive step pulses) for driving the X-, Y-, and Z-axis stepper
    motors.

    The `TrajectoryPlanner` generates all necessary data about the segments and 
    determines the boundary velocities of the segments (the X-, Y- and 
    Z-components of the velocity at the start point and at the end point of 
    each segment). Internally it uses a `MotionProcessor` object to generate the
    final X-, Y-, and Z-axis motion profiles and to generate the step signals to
    drive the X-, Y-, and Z-axis stepper motors.
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
        if not isinstance(pitch, tuple):
            self.xpitch = self.ypitch = self.zpitch = pitch
        else:
            self.xpitch, self.ypitch, self.zpitch = pitch

        self.profile_type = profile_type

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
        Takes the points that define the segments of the trajectory.
        Assembles and collects the necessary data into `SegmentData` objects
        (angular displacement of the axes, rotation direction of the axes,
        initial angular position of the axes at the start point of each segment).
        Sets the velocity of the axes to zero at the start and end point of the
        trajectory.
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
        Sets the start and end velocity to zero for non-moving axes in every 
        segment of the trajectory. Also sets the end velocity of a moving axis
        to zero when its rotation direction reverses between the current and the
        next segment. The start velocity of the next segment in the trajectory
        is then also set to zero.
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
        Creates a motion profile with either an optimized start or end velocity
        to accomplish the required axis displacement in the shortest possible
        travel time.
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
        if len(motion_profiles) > 1:
            # Get the motion profile with the longest travel time and recreate the
            # motion profile of the other moving axes so that their travel time
            # becomes equal to the longest travel time (time synchronization)
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
        # synchronize these motion profiles so that they all share the same
        # total travel time equal to the maximum of the individual travel times.
        motion_profiles = self._synchronize_motion_profiles(motion_profiles, find)
        return motion_profiles        
    
    @staticmethod
    def __get_path_deviation(
        moving_axes: dict[str, AxisData],
        num_points: int = 100
    ) -> float:

        def get_axis_positions(
            moving_axes: dict[str, AxisData],
            num_points: int = 100
        ) -> list[NDArray[np.float64]]:
            positions: list[NDArray[np.float64]] = []
            for axisdata in moving_axes.values():
                theta_fn = np.vectorize(axisdata.mp.get_position_time_fn())
                t_arr = np.linspace(0.0, axisdata.mp.dt_tot, num_points)
                s_arr = theta_fn(t_arr) / (360.0 * axisdata.pitch)
                s0_arr = np.full_like(s_arr, s_arr[0])
                ds_arr = (s_arr - s0_arr) * axisdata.rdir.to_int()
                s_arr = s0_arr + ds_arr
                positions.append(s_arr)
            return positions
        
        axis_positions = get_axis_positions(moving_axes, num_points)
        
        # Start- and end points of segment
        p0 = np.array([s_arr[0] for s_arr in axis_positions])
        p1 = np.array([s_arr[-1] for s_arr in axis_positions])
        dp = p1 - p0
        dp_norm_sq = np.dot(dp, dp)

        # Calculate distance to the straight segment line
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
    
    def _get_path_deviation(
        self, 
        segment: Segment, 
        num_points: int = 100
    ) -> float:    
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
        path_deviation = self.__get_path_deviation(moving_axes, num_points)
        return path_deviation
    
    def _minimize_segment_path_deviation(
        self,
        moving_axes: dict[str, AxisData],
        find: str,
        allowed_path_deviation: float = 1.e-3,
        num_points: int = 100
    ) -> dict[str, MotionProfile]:

        def fn(omega_m: float) -> float:
            try:
                for axisdata in moving_axes.values(): axisdata.omega_m = omega_m
                motion_profiles = self._minimize_segment_profile_time(moving_axes, find)
                for k, mp in motion_profiles.items(): moving_axes[k].mp = mp
                path_deviation = self.__get_path_deviation(moving_axes, num_points)
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
        trajectory. First, the motion profiles are created to perform their
        required displacement in the shortest possible travel time by changing
        either the start or end velocity of the movement (which one is
        designated by `find`). Next, the motion profiles are recreated, except
        the motion profile with the longest travel time, so that all have the
        same travel time equal to the longest travel time.

        Returns
        -------
        List of tuples. The tuples contain the axis designation and the
        associated motion profile.
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
        start). Creates time-synchronized motion profiles for the moving axes
        in each segment. The initial velocity obtained for each of the axes is
        held in the corresponding `SegmentData` object. The initial velocity of
        each axis is also assigned to the final velocity of the same axis in the
        preceding segment.
        This function must be called before `_set_end_velocities()`.
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
        time-synchronized motion profiles for the moving axes in each segment.
        Checks whether the obtained final velocity for each axis in the segment
        is smaller or not then the currently available value in the
        corresponding `SegmentData` object. The smallest value is kept in the
        `SegmentData` object.
        This function must be called after `_set_start_velocities()`.
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
        Creates and returns the `Trajectory` object, a list of `Segment` objects.
        """
        # Calculate optimal start and end velocities of the axes in each segment
        # of the trajectory.
        self._set_start_velocities(optimize, allowed_path_deviation, num_points)
        self._set_end_velocities(optimize, allowed_path_deviation, num_points)
        
        # Create the `Segment` objects with the final motion profiles and stepper
        # driver signals of each axis in the segments of the trajectory.
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
        Given a sequence of points, returns a `Trajectory` object, which is
        a list of `Segment` objects.
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
        Returns a list with the data about the segments that compose the
        trajectory (list of `SegmentData` objects).
        """
        return self._segmentdata_lst

    def _reset_trajectory_kinematics(
        self,
        a_m: float | tuple[float, float, float],
        v_m: float | tuple[float, float, float]
    ) -> None:
        """
        Resets the maximum acceleration and maximum velocity of the axes to the
        given new values for recalculation of the trajectory.
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
        Recreates the trajectory with the new values for the maximum
        acceleration and maximimum velocity, and returns a new `Trajectory`
        object.
        """
        if not self._segmentdata_lst:
            raise ValueError("No trajectory was created before.")

        optimize = kwargs.get("optimize", False)
        allowed_path_deviation = kwargs.get("allowed_path_deviation", 1e-3)
        num_points = kwargs.get("num_points", 100)

        self._reset_trajectory_kinematics(a_m, v_m)
        trajectory = self._create_trajectory(optimize, allowed_path_deviation, num_points)
        return trajectory

    def optimize_trajectory(
        self,
        allow_dev: float,
    ) -> Trajectory:

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
