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
from scipy.optimize import minimize_scalar, root_scalar

from pyberryplc.motion.multi_axis import MotionProfile, RotationDirection
from pyberryplc.stepper.driver.process import TwoStageProfileRotator


TPoint = tuple[float, float] | tuple[float, float, float]
TSegment = tuple[TPoint, TPoint]
TProfile = tuple[NDArray[np.float64], NDArray[np.float64]]


@dataclass
class AxisData:
    """
    Dataclass used by `SegementData` to return the data for a single axis
    (either X-, Y-, or Z-axis).
    """
    profile_type: Type[MotionProfile]
    dtheta: float
    theta_i: float
    v_m: float
    a_m: float
    v_i: float
    v_f: float
    rdir: RotationDirection


@dataclass
class SegmentData:
    """
    Intermediate dataclass that holds data about the segments in a trajectory
    that is needed to generate the motion profiles and step signals for the
    X-, Y-, and/or Z-axis movements. 
    """
    segment: TSegment
    dtheta_x: float
    dtheta_y: float
    dtheta_z: float
    rdir_x: RotationDirection | None
    rdir_y: RotationDirection | None
    rdir_z: RotationDirection | None
    theta_xi: float
    theta_yi: float
    theta_zi: float
    v_xm: float | None = None
    v_ym: float | None = None
    v_zm: float | None = None
    a_xm: float | None = None
    a_ym: float | None = None
    a_zm: float | None = None
    v_xi: float | None = None
    v_xf: float | None = None
    v_yi: float | None = None
    v_yf: float | None = None
    v_zi: float | None = None
    v_zf: float | None = None
    profile_type: Type[MotionProfile] | None = None

    def get_xaxis_data(self) -> AxisData:
        return AxisData(
            dtheta=self.dtheta_x,
            theta_i=self.theta_xi,
            v_m=self.v_xm,
            a_m=self.a_xm,
            v_i=self.v_xi,
            v_f=self.v_xf,
            profile_type=self.profile_type,
            rdir=self.rdir_x
        )

    def get_yaxis_data(self) -> AxisData:
        return AxisData(
            dtheta=self.dtheta_y,
            theta_i=self.theta_yi,
            v_m=self.v_ym,
            a_m=self.a_ym,
            v_i=self.v_yi,
            v_f=self.v_yf,
            profile_type=self.profile_type,
            rdir=self.rdir_y
        )

    def get_zaxis_data(self) -> AxisData:
        return AxisData(
            dtheta=self.dtheta_z,
            theta_i=self.theta_zi,
            v_m=self.v_zm,
            a_m=self.a_zm,
            v_i=self.v_zi,
            v_f=self.v_zf,
            profile_type=self.profile_type,
            rdir=self.rdir_z
        )
    
    def __str__(self):
        lines = []
        for field in fields(self):
            value = getattr(self, field.name)
            lines.append(f"{field.name}: {value}")
        return "\n".join(lines)


@dataclass
class Segment:
    """
    Dataclass that has the attributes to draw and investigate the generated 
    motion profiles of a segment in the trajectory, and to drive the stepper
    motors of the X-, Y-, and/or Z-axis.
    
    Attributes
    ----------
    mp_x:
        Motion profile of the X-axis motor rotation.
    mp_y:
        Motion profile of the Y-axis motor rotation.
    mp_z:
        Motion profile of the Z-axis motor rotation.
    delays_x:
        Step signal to drive the X-axis stepper motor (list of time delays 
        between successive step pulses).
    delays_y:
        Step signal to drive the Y-axis stepper motor.
    delays_z:
        Step signal to drive the Z-axis stepper motor.
    rdir_x:
        Rotation direction of the X-axis motor.
    rdir_y:
        Rotation direction of the Y-axis motor.
    rdir_z:
        Rotation direction of the Z-axis motor.
    """
    mp_x: MotionProfile | None = None
    mp_y: MotionProfile | None = None
    mp_z: MotionProfile | None = None
    delays_x: list[float] = field(default_factory=list)
    delays_y: list[float] = field(default_factory=list)
    delays_z: list[float] = field(default_factory=list)
    rdir_x: RotationDirection = RotationDirection.COUNTERCLOCKWISE
    rdir_y: RotationDirection = RotationDirection.COUNTERCLOCKWISE
    rdir_z: RotationDirection = RotationDirection.COUNTERCLOCKWISE
    position_profiles: tuple[TProfile, ...] | None = None
    velocity_profiles: tuple[TProfile, ...] | None = None
    acceleration_profiles: tuple[TProfile, ...] | None = None
    
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
    def __init__(self, parent: 'TrajectoryPlanner') -> None:
        """
        Creates a `MotionProcessor` object. This object is internally created
        in a `TrajectoryPlanner` object.
        """
        self.parent = parent
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
        if self.parent.x_motor is not None:
            # noinspection PyTypeChecker
            self._x_rotator = TwoStageProfileRotator(self.parent.x_motor)
        if self.parent.y_motor is not None:
            # noinspection PyTypeChecker
            self._y_rotator = TwoStageProfileRotator(self.parent.y_motor)
        if self.parent.z_motor is not None:
            # noinspection PyTypeChecker
            self._z_rotator = TwoStageProfileRotator(self.parent.z_motor)

    def _create_motor_motion_profile(self, axis: str) -> MotionProfile:
        """
        Takes the segment data belonging to the given axis and creates a
        motion profile for that axis.
        """
        # noinspection PyUnreachableCode
        match axis:
            case "x":
                data = self._segmentdata.get_xaxis_data()
            case "y":
                data = self._segmentdata.get_yaxis_data()
            case "z":
                data = self._segmentdata.get_zaxis_data()
            case _:
                raise ValueError(f"Invalid axis: {axis}")

        mp = data.profile_type(
            ds_tot=data.dtheta,
            a_m=data.a_m,
            v_m=data.v_m,
            v_i=data.v_i,
            s_i=data.theta_i,
            v_f=data.v_f
        )
        return mp

    @staticmethod
    def _synchronize_profiles(
        mp_x: MotionProfile,
        mp_y: MotionProfile,
        mp_z: MotionProfile
    ) -> tuple[MotionProfile, MotionProfile, MotionProfile]:
        """
        Synchronizes the motion profiles of the X-, Y-, and Z-axis movements.
        Checks which of the motion profiles takes the most time, and it 
        changes the top velocity of the other motion profiles such that all
        movements would finish at the same time.
        """
        # only consider the axes that need to move
        moving = [mp for mp in (mp_x, mp_y, mp_z) if mp.ds_tot > 0.0]
        non_moving = [mp for mp in (mp_x, mp_y, mp_z) if mp.ds_tot == 0.0]
        if len(moving) == 1:
            # only one axis needs to move: no synchronisation needed
            for mp in non_moving: mp.dt_tot = moving[0].dt_tot
            return mp_x, mp_y, mp_z

        # take axis with longest travel time; this must also become the travel
        # time of the other moving axes
        dt_tot_lst = [mp.dt_tot for mp in moving]
        dt_max = max(dt_tot_lst)
        for mp in non_moving: mp.dt_tot = dt_max
        idx = dt_tot_lst.index(dt_max)
        moving.pop(idx)  # axis with longest travel time can be omitted now. 
        for mp in moving:
            def fn(v: float) -> float:
                mp.v_top = v
                dev = dt_max - mp.dt_tot
                return dev

            sol = root_scalar(fn, bracket=[1.e-6, mp.v_top])
            mp.v_top = sol.root

        return mp_x, mp_y, mp_z

    def _calc_motor_motion_profiles(self) -> None:
        mp_x = self._create_motor_motion_profile("x")
        mp_y = self._create_motor_motion_profile("y")
        mp_z = self._create_motor_motion_profile("z")
        self._mp_x, self._mp_y, self._mp_z = self._synchronize_profiles(mp_x, mp_y, mp_z)
    
    def _get_position_profiles(self) -> tuple[TProfile, TProfile, TProfile]:
        
        def _create_position_profile(mp, pitch, rdir):
            t_arr, theta_arr = mp.position_profile()
            s_arr = theta_arr / (360.0 * pitch)
            if rdir == RotationDirection.CLOCKWISE:
                s0 = s_arr[0]
                ds_arr = s_arr - s0
                s0_arr = np.full_like(s_arr, s0)
                s_arr = s0_arr - ds_arr
            return t_arr, s_arr
        
        pos_x = _create_position_profile(
            self._mp_x, 
            self.parent.xpitch, 
            self._segmentdata.rdir_x
        )
        pos_y = _create_position_profile(
            self._mp_y,
            self.parent.ypitch,
            self._segmentdata.rdir_y
        )
        pos_z = _create_position_profile(
            self._mp_z,
            self.parent.zpitch,
            self._segmentdata.rdir_z
        )
        return pos_x, pos_y, pos_z
    
    def _get_velocity_profiles(self) -> tuple[TProfile, TProfile, TProfile]:
        
        def _create_velocity_profile(mp, pitch, rdir):
            t_arr, omega_arr = mp.velocity_profile()
            v_arr = omega_arr / (360.0 * pitch)
            if rdir == RotationDirection.CLOCKWISE:
                v_arr *= -1.0
            return t_arr, v_arr
        
        vel_x = _create_velocity_profile(
            self._mp_x,
            self.parent.xpitch,
            self._segmentdata.rdir_x
        )
        vel_y = _create_velocity_profile(
            self._mp_y,
            self.parent.ypitch,
            self._segmentdata.rdir_y
        )
        vel_z = _create_velocity_profile(
            self._mp_z,
            self.parent.zpitch,
            self._segmentdata.rdir_z
        )
        return vel_x, vel_y, vel_z
    
    def _get_acceleration_profiles(self) -> tuple[TProfile, TProfile, TProfile]:
        
        def _create_acceleration_profile(mp, pitch):
            t_arr, alpha_arr = mp.acceleration_profile()
            a_arr = alpha_arr / (360.0 * pitch)
            return t_arr, a_arr
        
        acc_x = _create_acceleration_profile(
            self._mp_x,
            self.parent.xpitch
        )
        acc_y = _create_acceleration_profile(
            self._mp_y,
            self.parent.ypitch
        )
        acc_z = _create_acceleration_profile(
            self._mp_z,
            self.parent.zpitch
        )
        return acc_x, acc_y, acc_z
    
    def _calc_delays(self):
        if not all((self._mp_x, self._mp_y, self._mp_z)):
            self._calc_motor_motion_profiles()
        
        if self._x_rotator:
            self._x_rotator.preprocess(self._segmentdata.rdir_x, self._mp_x)
            self._x_delays = self._x_rotator.delays
        
        if self._y_rotator:
            self._y_rotator.preprocess(self._segmentdata.rdir_y, self._mp_y)
            self._y_delays = self._y_rotator.delays
        
        if self._z_rotator:
            self._z_rotator.preprocess(self._segmentdata.rdir_z, self._mp_y)
            self._z_delays = self._z_rotator.delays
    
    def create_segment(self, segmentdata: SegmentData) -> Segment:
        self._segmentdata = segmentdata
        self._calc_motor_motion_profiles()
        self._calc_delays()
        segment = Segment()
        if self._mp_x: segment.mp_x = self._mp_x
        if self._mp_y: segment.mp_y = self._mp_y
        if self._mp_z: segment.mp_z = self._mp_z
        if self._x_delays: segment.delays_x = self._x_delays
        if self._y_delays: segment.delays_y = self._y_delays
        if self._z_delays: segment.delays_z = self._z_delays
        segment.rdir_x = segmentdata.rdir_x
        segment.rdir_y = segmentdata.rdir_y
        segment.rdir_z = segmentdata.rdir_z
        segment.position_profiles = self._get_position_profiles()
        segment.velocity_profiles = self._get_velocity_profiles()
        segment.acceleration_profiles = self._get_acceleration_profiles()
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
    def motion_profiles(self) -> tuple[dict[str, dict[str, NDArray[np.float64]]], ...] | None:
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


class TrajectoryPlanner:
    """
    Creates a `Trajectory` object. 

    A `Trajectory` object is a list of `Segment` objects. A `Segment`object 
    contains the motion profiles and rotation directions of the X-, Y-, and 
    Z-axis movements. 
    It can also contain the step signals (lists of time delays between 
    successive step pulses) for driving the X-, Y-, and Z-axis stepper motors. 

    The `TrajectoryPlanner` generates all necessary data about the segments and 
    determines the boundary velocities of the segments (the X-, Y- and 
    Z-components of the velocity at the start point and at the end point of 
    each segment). It internally uses a `MotionProcessor` object to generate the
    X-, Y-, and Z-axis motion profiles and to generate the step signals to drive
    the the X-, Y-, and Z-axis stepper motors.
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
    def _check_points(*points: TPoint) -> list[TPoint]:
        """
        Checks whether points have two or three coordinates. If points only
        have two coordinates (x and y), a z-coordinate with a value of 0.0 is
        added to the points.
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
        segment: TSegment
    ) -> tuple[tuple[float, ...], tuple[RotationDirection | None, ...]]:
        """
        Calculates the angular displacements of the the X-, Y- and Z-axis motor 
        that correspond with the linear displacements of a segment, and also
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
    
    def _get_initial_angles(self, segment: TSegment) -> tuple[float, float, float]:
        """
        Calculates the angular positions of the X-, Y- and Z-axis motor 
        shafts at the start point of a segment.
        """
        xi = segment[0][0]
        yi = segment[0][1]
        zi = segment[0][2]
        
        theta_xi = 360.0 * self.xpitch * xi
        theta_yi = 360.0 * self.ypitch * yi
        theta_zi = 360.0 * self.zpitch * zi
        return theta_xi, theta_yi, theta_zi
    
    def _create_segmentdata(self, pts: list[TPoint]) -> list[SegmentData]:
        """
        Takes the points that define the segments of the trajectory, and 
        collects the necessary data for creating `Segment` objects (angular
        displacements, rotation directions, initial angular positions at the
        start point of each segment, velocities at the start and end point of
        the trajectory).
        """
        segments = list(zip(pts[:-1], pts[1:]))
        i_max = len(segments) - 1
        segmentdata_lst = []
        for i, segment in enumerate(segments):
            dtheta, rdir = self._get_angular_displacements(segment)
            theta_i = self._get_initial_angles(segment)
            segmentdata = SegmentData(
                segment, 
                dtheta[0], dtheta[1], dtheta[2],
                rdir[0], rdir[1], rdir[2],
                theta_i[0], theta_i[1], theta_i[2]
            )
            segmentdata.v_xm = self.vx_m
            segmentdata.v_ym = self.vy_m
            segmentdata.v_zm = self.vz_m
            segmentdata.a_xm = self.ax_m
            segmentdata.a_ym = self.ay_m
            segmentdata.a_zm = self.az_m
            segmentdata.profile_type = self.profile_type
            if i == 0:
                segmentdata.v_xi = 0.0
                segmentdata.v_yi = 0.0
                segmentdata.v_zi = 0.0
            if i == i_max:
                segmentdata.v_xf = 0.0
                segmentdata.v_yf = 0.0
                segmentdata.v_zf = 0.0
            segmentdata_lst.append(segmentdata)
        return segmentdata_lst

    def _minimize_profile_time(
        self, 
        axis: str, 
        ds_tot: float, 
        v_other: float, 
        find: str
    ) -> float:
        """
        Method to find the optimal initial velocity (x, y, or z-component) to
        make the required displacement in the shortest possible time or to find
        the maximum final velocity (x, y, or z-component) that can be reached at
        the end point of a segment.
        """
        a_m, v_m = 0.0, 0.0
        match axis:
            case "x":
                a_m = self.ax_m
                v_m = self.vx_m
            case "y":
                a_m = self.ay_m
                v_m = self.vy_m
            case "z":
                a_m = self.az_m
                v_m = self.vz_m
        
        def fn(v: float) -> float:
            try:
                kwargs = {"v_i": v, "v_f": v_other} if find == "v_i" else {"v_i": v_other, "v_f": v}
                mp = self.profile_type(ds_tot=ds_tot, a_m=a_m, v_m=v_m, **kwargs)
                return mp.dt_tot
            except ValueError:
                return float("inf")
    
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            max_speed = min(v_m, np.sqrt(2 * a_m * ds_tot))
            r = minimize_scalar(fn, bounds=(1.e-6, max_speed), method="bounded")
        if r.success:
            return float(r.x)
        raise ValueError(f"{find} could not be determined")

    def _analyze_segmentdata(self, segmentdata_lst: list[SegmentData]) -> None:
        """After the initial segment data has been collected or determined, this
        method determines the achievable velocity components at the start and
        end point of each segments to ensure continuous movements along the
        trajectory. 
        """
        i_max = len(segmentdata_lst) - 1
        
        # Check for axes that don't move in a segment or where rotation 
        # direction reverses between two successive segments.
        i = 0
        for segmentdata1 in segmentdata_lst:
            if i < i_max:
                segmentdata2 = segmentdata_lst[i + 1]
                
                dtheta_x1 = segmentdata1.dtheta_x
                dtheta_x2 = segmentdata2.dtheta_x
                rdir_x1 = int(segmentdata1.rdir_x)
                rdir_x2 = int(segmentdata2.rdir_x)
                if dtheta_x1 == 0.0:
                    segmentdata1.v_xi = 0.0
                    segmentdata1.v_xf = 0.0
                    segmentdata2.v_xi = 0.0
                if dtheta_x2 == 0.0 or rdir_x1 * rdir_x2 < 0.0:
                    segmentdata1.v_xf = 0.0
                    segmentdata2.v_xi = 0.0
    
                dtheta_y1 = segmentdata1.dtheta_y
                dtheta_y2 = segmentdata2.dtheta_y
                rdir_y1 = int(segmentdata1.rdir_y)
                rdir_y2 = int(segmentdata2.rdir_y)
                if dtheta_y1 == 0.0:
                    segmentdata1.v_yi = 0.0
                    segmentdata1.v_yf = 0.0
                    segmentdata2.v_yi = 0.0
                if dtheta_y2 == 0.0 or rdir_y1 * rdir_y2 < 0.0:
                    segmentdata1.v_yf = 0.0
                    segmentdata2.v_yi = 0.0
    
                dtheta_z1 = segmentdata1.dtheta_z
                dtheta_z2 = segmentdata2.dtheta_z
                rdir_z1 = int(segmentdata1.rdir_z)
                rdir_z2 = int(segmentdata2.rdir_z)
                if dtheta_z1 == 0.0:
                    segmentdata1.v_zi = 0.0
                    segmentdata1.v_zf = 0.0
                    segmentdata2.v_zi = 0.0
                if dtheta_z2 == 0.0 or rdir_z1 * rdir_z2 < 0.0:
                    segmentdata1.v_zf = 0.0
                    segmentdata2.v_zi = 0.0
            i += 1
        
        # Find the optimal initial velocity (if not already determined before) 
        # at the start point of segments. For this, we start at the end point
        # of the trajectory and proceed to its start point.
        i = i_max
        for segmentdata2 in reversed(segmentdata_lst):
            if i > 0:
                segmentdata1 = segmentdata_lst[i - 1]
    
                if segmentdata2.v_xi is None:
                    segmentdata2.v_xi = self._minimize_profile_time(
                        axis="x",
                        ds_tot=segmentdata2.dtheta_x,
                        v_other=segmentdata2.v_xf,
                        find="v_i"
                    )
                    segmentdata1.v_xf = segmentdata2.v_xi
    
                if segmentdata2.v_yi is None:
                    segmentdata2.v_yi = self._minimize_profile_time(
                        axis="y",
                        ds_tot=segmentdata2.dtheta_y,
                        v_other=segmentdata2.v_yf,
                        find="v_i"
                    )
                    segmentdata1.v_yf = segmentdata2.v_yi
                
                if segmentdata2.v_zi is None:
                    segmentdata2.v_zi = self._minimize_profile_time(
                        axis="z",
                        ds_tot=segmentdata2.dtheta_y,
                        v_other=segmentdata2.v_zf,
                        find="v_i"
                    )
                    segmentdata1.v_zf = segmentdata2.v_zi
            i -= 1
        
        # In a final step, we also check the maximum velocity that is achievable
        # at the end points of the segments. As the end point of one segment 
        # coincides with the start point of the next segment, we can now have
        # two velocity values (the one from the previous step and the one from
        # this current step). We need to keep the smallest of these two values.
        i = 0
        for segmentdata1 in segmentdata_lst:
            if i < i_max:
                segmentdata2 = segmentdata_lst[i + 1]
    
                if segmentdata1.v_xf > 0.0:
                    v_xf = self._minimize_profile_time(
                        axis="x",
                        ds_tot=segmentdata1.dtheta_x,
                        v_other=segmentdata1.v_xi,
                        find="v_f"
                    )
                    v_xf = min(segmentdata1.v_xf, v_xf)
                    segmentdata1.v_xf = v_xf
                    segmentdata2.v_xi = v_xf
    
                if segmentdata1.v_yf > 0.0:
                    v_yf = self._minimize_profile_time(
                        axis="y",
                        ds_tot=segmentdata1.dtheta_y,
                        v_other=segmentdata1.v_yi,
                        find="v_f"
                    )
                    v_yf = min(segmentdata1.v_yf, v_yf)
                    segmentdata1.v_yf = v_yf
                    segmentdata2.v_yi = v_yf
                
                if segmentdata1.v_zf > 0.0:
                    v_zf = self._minimize_profile_time(
                        axis="z",
                        ds_tot=segmentdata1.dtheta_z,
                        v_other=segmentdata1.v_zi,
                        find="v_f"
                    )
                    v_zf = min(segmentdata1.v_zf, v_zf)
                    segmentdata1.v_zf = v_zf
                    segmentdata2.v_zi = v_zf
            i += 1
    
    def get_trajectory(self, *points: TPoint):
        """
        Given a sequence of points, returns a `Trajectory` object, which is
        a list of `Segment` objects.
        """
        points = self._check_points(*points)
        self._segmentdata_lst = self._create_segmentdata(points)
        self._analyze_segmentdata(self._segmentdata_lst)
        trajectory = Trajectory()
        for segmentdata in self._segmentdata_lst:
            segment = self.motion_processor.create_segment(segmentdata)
            trajectory.append(segment)
        return trajectory
    
    def get_segmentdata(self) -> list[SegmentData]:
        """
        Returns the data about the segments that compose the trajectory in a
        list.
        """
        return self._segmentdata_lst


def main():
    
    from pyberryplc.motion.multi_axis import SCurvedProfile
    from charts import LineChart
    
    p1 = (0.0, 0.0)
    p2 = (0.02, 0.02)
    
    planner = TrajectoryPlanner(
        pitch=100,
        profile_type=SCurvedProfile,
        a_m=360.0,
        v_m=180.0,
        x_motor=StepperMotorMock(200, 1),
        y_motor=StepperMotorMock(200, 1)
    )
    trajectory = planner.get_trajectory(p1, p2)
    for segment in trajectory:
        print(segment, end='\n\n')
    
    segmentdata_lst = planner.get_segmentdata()
    for segment_data in segmentdata_lst:
        print(segment_data, end='\n\n')
    
    pos, vel, acc = trajectory.motion_profiles
    pchart = LineChart()
    pchart.add_xy_data(
        label="x",
        x1_values=pos["x"]["time"],
        y1_values=pos["x"]["values"]
    )
    pchart.add_xy_data(
        label="y",
        x1_values=pos["y"]["time"],
        y1_values=pos["y"]["values"]
    )
    pchart.add_xy_data(
        label="z",
        x1_values=pos["z"]["time"],
        y1_values=pos["z"]["values"]
    )
    pchart.x1.add_title("time, s")
    pchart.y1.add_title("position, °")
    pchart.add_legend(columns=3)
    pchart.show()

    vchart = LineChart()
    vchart.add_xy_data(
        label="x",
        x1_values=vel["x"]["time"],
        y1_values=vel["x"]["values"]
    )
    vchart.add_xy_data(
        label="y",
        x1_values=vel["y"]["time"],
        y1_values=vel["y"]["values"]
    )
    vchart.add_xy_data(
        label="z",
        x1_values=vel["z"]["time"],
        y1_values=vel["z"]["values"]
    )
    vchart.x1.add_title("time, s")
    vchart.y1.add_title("velocity, °/s")
    vchart.add_legend(columns=3)
    vchart.show()

    achart = LineChart()
    achart.add_xy_data(
        label="x",
        x1_values=acc["x"]["time"],
        y1_values=acc["x"]["values"]
    )
    achart.add_xy_data(
        label="y",
        x1_values=acc["y"]["time"],
        y1_values=acc["y"]["values"]
    )
    achart.add_xy_data(
        label="z",
        x1_values=acc["z"]["time"],
        y1_values=acc["z"]["values"]
    )
    achart.x1.add_title("time, s")
    achart.y1.add_title("acceleration, °/s²")
    achart.add_legend(columns=3)
    achart.show()


if __name__ == '__main__':
    main()
    