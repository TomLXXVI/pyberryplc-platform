import warnings
from typing import Any
from dataclasses import dataclass

import numpy as np
from scipy.optimize import root_scalar, minimize_scalar

from pyberryplc.stepper.driver.process import TwoStageProfileRotator

from .multi_axis import (
    RotationDirection, 
    MotionProfile, 
    TrapezoidalProfile, SCurvedProfile, 
    MotionProfileType
)


TPoint = tuple[float, float]
TSegment = tuple[TPoint, TPoint]


class XYMotionProcessor:
    """
    Generates the motion profiles along the X- and Y-axis to move from one 
    point to the next along a rectilinear segment (i.e. X- and Y-movements
    are synchronized).
    """
    def __init__(
        self,
        pitch: float,
        motor_speed: float,
        motor_accel: float,
        profile_type: MotionProfileType = MotionProfileType.TRAPEZOIDAL
    ) -> None:
        """
        Initializes the `XYMotionControl` object.

        Parameters
        ----------
        pitch: float
            Pitch of the lead screw (i.e. number of revolutions of the screw to
            move the nut one meter). Converts linear to rotational motion.
        motor_speed: float
            Available angular speed of the motor (deg/s).
        motor_accel: float
            Available angular acceleration of the motor (deg/s²). This will
            depend on the torque capabilities of the motor and on the total 
            inertia moment of the motor and the connected load.
        profile_type : ProfileType
            The type of motion profile. Either a trapezoidal (default) or 
            S-curved motion profile.
        """
        if profile_type in MotionProfileType.get_types():
            self.profile_type = profile_type
        else:
            raise TypeError(f"Profile type '{profile_type}' does not exist.")
        
        self.pitch = pitch
        self.v_m = motor_speed
        self.a_m = motor_accel

        self._N = 360.0 * self.pitch  # convert revs/m -> deg/m
        self._pt_i: TPoint = (0.0, 0.0)
        self._pt_f: TPoint = (0.0, 0.0)
        self._theta_xi: float = 0.0
        self._theta_yi: float = 0.0
        self._omega_xi: float = 0.0
        self._omega_yi: float = 0.0
        self._omega_xf: float | None = None
        self._omega_yf: float | None = None
        self._dir_x: RotationDirection = RotationDirection.COUNTERCLOCKWISE
        self._dir_y: RotationDirection = RotationDirection.COUNTERCLOCKWISE
        self._dx: float = 0.0
        self._dy: float = 0.0
        self._dtheta_x: float = 0.0
        self._dtheta_y: float = 0.0
        self._mp_x: MotionProfile | None = None
        self._mp_y: MotionProfile | None = None
    
    def set_segmentdata(
        self,
        start: TPoint,
        end: TPoint,
        initial_speed: tuple[float, float] = (0.0, 0.0),
        final_speed: tuple[float | None, float | None] = (None, None)
    ) -> None:
        """Sets the start and end point of the segment, and also the velocity 
        components at the start and end point of the segment.
        """
        self._calc_displacements(start, end)
        self._omega_xi, self._omega_yi = initial_speed
        self._omega_xf, self._omega_yf = final_speed
        # Erase any previous motion profiles
        self._mp_x, self._mp_y = None, None
    
    def _calc_displacements(self, pt_i: TPoint, pt_f: TPoint) -> None:
        self._dir_x = RotationDirection.COUNTERCLOCKWISE
        self._dir_y = RotationDirection.COUNTERCLOCKWISE
        self._pt_i = pt_i
        self._pt_f = pt_f
        
        # Initial angles of the X- and Y-axis motor shaft
        self._theta_xi = self._N * pt_i[0]
        self._theta_yi = self._N * pt_i[1]
        
        # Linear displacements along the X- and Y-axis
        self._dx = self._pt_f[0] - self._pt_i[0]
        self._dy = self._pt_f[1] - self._pt_i[1]
        
        # Rotation direction of the X- and Y-axis motor
        if self._dx < 0.0:
            self._dir_x = RotationDirection.CLOCKWISE
            self._dx = abs(self._dx)
        if self._dy < 0.0:
            self._dir_y = RotationDirection.CLOCKWISE
            self._dy = abs(self._dy)
        
        # Angular displacements of the X- and Y-axis motor
        self._dtheta_x = self._N * self._dx
        self._dtheta_y = self._N * self._dy

    def _create_motion_profile(self, **kwargs) -> MotionProfile | None:
        # noinspection PyUnreachableCode
        match self.profile_type:
            case MotionProfileType.TRAPEZOIDAL:
                _MotionProfile = TrapezoidalProfile
            case MotionProfileType.S_CURVED:
                _MotionProfile = SCurvedProfile
            case _:
                _MotionProfile = TrapezoidalProfile

        mp = _MotionProfile(**kwargs)
        return mp
    
    @staticmethod
    def _synchronize_profiles(mp_x: MotionProfile, mp_y: MotionProfile):
        mp_lst = [mp_x, mp_y]
        dt_tot_lst = [mp.dt_tot for mp in mp_lst]
        dt_tot = max(dt_tot_lst)
        idx = dt_tot_lst.index(min(dt_tot_lst))
        mp = mp_lst[idx]  # motion profile with the shortest `dt_tot` (can be 0).
        if 0.0 not in dt_tot_lst:
            # Note: if one of the axes has `dt_tot` equal to zero, we have
            # single-axis motion -> no synchronisation needed.

            # Find top velocity of `mp` so that its `dt_tot` becomes equal to the
            # maximum `dt_tot`.
            def fn(v: float) -> float:
                mp.v_top = v
                dev = dt_tot - mp.dt_tot
                return dev

            sol = root_scalar(fn, bracket=[1.e-6, mp.v_top])
            mp.v_top = sol.root
        else:
            # Pass the travel time of the other axis also to the axis at rest so
            # that motion profiles can be drawn correctly.
            mp.dt_tot = dt_tot
        return mp_x, mp_y
    
    def _generate_profiles(self) -> None:
        """
        Generates the synchronised motion profiles for the X- and Y-axis 
        movements. The X- and Y-axis movements must start and end at the same 
        time, regardless of the size of their displacements.
        """
        self._mp_x = self._create_motion_profile(
            ds_tot=self._dtheta_x,
            a_m=self.a_m,
            v_m=self.v_m,
            v_i=self._omega_xi,
            s_i=self._theta_xi,
            v_f=self._omega_xf
        )
        self._mp_y = self._create_motion_profile(
            ds_tot=self._dtheta_y,
            a_m=self.a_m,
            v_m=self.v_m,
            v_i=self._omega_yi,
            s_i=self._theta_yi,
            v_f=self._omega_yf
        )
        self._mp_x, self._mp_y = self._synchronize_profiles(self._mp_x, self._mp_y)

    @property
    def x_motion(self) -> tuple[MotionProfile, RotationDirection]:
        """
        Returns the motion profile and rotation direction for the X-axis 
        movement.
        """
        if self._mp_x is None: self._generate_profiles()
        return self._mp_x, self._dir_x

    @property
    def y_motion(self) -> tuple[MotionProfile, RotationDirection]:
        """
        Returns the motion profile and rotation direction for the Y-axis 
        movement.
        """
        if self._mp_y is None: self._generate_profiles()
        return self._mp_y, self._dir_y


class FakeStepperMotor:
    """
    This class is only intended as a hack to be able to use the function 
    `preprocess()` of class `TwoStageProfileRotator(ProfileRotator)` for 
    calculating the time delays between successive step pulses that drive a 
    stepper motor.
    """
    def __init__(
        self,
        full_steps_per_rev: int = 200,
        microstep_factor: int = 1
    ) -> None:
        """
        Creates a `FakeStepperMotor` object having just the attributes needed 
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


class Segment2D:
    """
    Represents a rectilinear 2D segment.

    Given the start- and end point of the segment, the motion profile and the
    rotation direction of the X-axis and Y-axis movement are determined.
    For this, an `XYMotionProcessor` object must be connected to the `Segment2D` 
    class through class method `connect_xy_motion_processor()`.

    The step pulses to drive the X-axis and Y-axis stepper motors, can also be 
    generated, if `FakeStepperMotor` objects are connected to the `Segment2D` 
    class. For this, use class method `connect_stepper_motors()`.
    """
    _xy_motion_proc: XYMotionProcessor = None
    _x_motor: FakeStepperMotor = None
    _y_motor: FakeStepperMotor = None

    @classmethod
    def connect_xy_motion_processor(
        cls, 
        xy_motion_processor: XYMotionProcessor
    ) -> None:
        """
        Connects an `XYMotionProcessor` object to the `Segment2D` class.  
        """
        cls._xy_motion_proc = xy_motion_processor

    @classmethod
    def connect_stepper_motors(
        cls, 
        x_motor: FakeStepperMotor, 
        y_motor: FakeStepperMotor
    ) -> None:
        """
        Connects `FakeStepperMotor` objects to the `Segment2D` class. 
        """
        cls._x_motor = x_motor
        cls._y_motor = y_motor

    def __init__(
        self,
        start: TPoint,
        end: TPoint,
        ini_speed: tuple[float, float] = (0.0, 0.0),
        fin_speed: tuple[float | None, float | None] = (None, None)
    ) -> None:
        """
        Creates a `Segment2D` object.

        Parameters
        ----------
        start: tuple[float, float]
            Start point of the segment.
        end: tuple[float, float]
            End point of the segment.
        ini_speed: tuple[float, float]
            Initial velocity in the X-axis direction and in the Y-axis direction
            at the start point of the segment.
        fin_speed: tuple[float | None, float | None]
            Final velocity in the X-axis direction and in the Y-axis direction 
            at the end point of the segment.
        """
        self.start = start
        self.end = end
        self._mp_x, self._dir_x = None, None
        self._mp_y, self._dir_y = None, None

        if self._xy_motion_proc is not None:
            # Generate motion profiles for the X- and Y-axis movement.
            self._xy_motion_proc.set_segmentdata(start, end, ini_speed, fin_speed)
            self._mp_x, self._dir_x = self._xy_motion_proc.x_motion
            self._mp_y, self._dir_y = self._xy_motion_proc.y_motion
            
            # Generate step pulse trains to drive the X- and Y-axis motor.
            self._x_rotator, self._y_rotator = None, None
            if self._x_motor is not None:
                # noinspection PyTypeChecker
                self._x_rotator = TwoStageProfileRotator(self._x_motor)
                self._x_rotator.preprocess(self._dir_x, self._mp_x)
            if self._y_motor is not None:
                # noinspection PyTypeChecker
                self._y_rotator = TwoStageProfileRotator(self._y_motor)
                self._y_rotator.preprocess(self._dir_y, self._mp_y)

    @property
    def x_motion(self) -> tuple[MotionProfile, RotationDirection] | None:
        """
        Returns the motion profile and rotation direction of the X-axis 
        movement.
        """
        if self._xy_motion_proc is not None:
            return self._mp_x, self._dir_x
        return None

    @property
    def y_motion(self) -> tuple[MotionProfile, RotationDirection] | None:
        """
        Returns the motion profile and rotation direction of the Y-axis 
        movement.
        """
        if self._xy_motion_proc is not None:
            return self._mp_y, self._dir_y
        return None

    @property
    def x_delays(self) -> list[float] | None:
        """
        Returns the step pulse train (list of time delays between successive 
        pulses) to drive the X-axis stepper motor.
        """
        if self._x_rotator is not None:
            # noinspection PyProtectedMember
            return self._x_rotator._delays
        return None

    @property
    def y_delays(self) -> list[float] | None:
        """
        Returns the step pulse train (list of time delays between successive 
        pulses) to drive the Y-axis stepper motor.
        """
        if self._y_rotator is not None:
            # noinspection PyProtectedMember
            return self._y_rotator._delays
        return None
    
    @property
    def x_direction(self) -> RotationDirection:
        """
        Returns the rotation direction of the X-axis motor.
        """
        return self._dir_x

    @property
    def y_direction(self) -> RotationDirection:
        """
        Returns the rotation direction of the Y-axis motor.
        """
        return self._dir_y
       
    @property
    def position_profiles(self) -> tuple | None:
        """
        Returns the position profile of the X-axis and Y-axis movement, taking
        the rotation direction into account.
        
        Each position profile is a tuple of two arrays: the time values and the
        corresponding position values.
        """
        if self._xy_motion_proc is not None:
            tx_arr, thetax_arr = self._mp_x.position_profile()
            x_arr = thetax_arr / self._xy_motion_proc._N
            if self._dir_x == RotationDirection.CLOCKWISE:
                x0 = x_arr[0]
                dx_arr = x_arr - x0
                x0_arr = np.full_like(x_arr, x0)
                x_arr = x0_arr - dx_arr
            pos_x = (tx_arr, x_arr)
            
            ty_arr, thetay_arr = self._mp_y.position_profile()
            y_arr = thetay_arr / self._xy_motion_proc._N
            if self._dir_y == RotationDirection.CLOCKWISE:
                y0 = y_arr[0]
                dy_arr = y_arr - y0
                y0_arr = np.full_like(y_arr, y0)
                y_arr = y0_arr - dy_arr
            pos_y = (ty_arr, y_arr)
            
            return pos_x, pos_y
        return None

    @property
    def velocity_profiles(self) -> tuple | None:
        """
        Returns the velocity profile of the X-axis and Y-axis movement, taking
        the rotation direction into account. Counterclockwise velocities are 
        considered positive, while clockwise velocities are negative.
        
        Each velocity profile is a tuple of two arrays: the time values and the
        corresponding velocity values.
        """
        if self._xy_motion_proc is not None:
            tx_arr, omegax_arr = self._mp_x.velocity_profile()
            vx_arr = omegax_arr / self._xy_motion_proc._N
            if self._dir_x == RotationDirection.CLOCKWISE:
                vx_arr *= -1.0
            vel_x = (tx_arr, vx_arr)
            
            ty_arr, omegay_arr = self._mp_y.velocity_profile()
            vy_arr = omegay_arr / self._xy_motion_proc._N
            if self._dir_y == RotationDirection.CLOCKWISE:
                vy_arr *= -1.0
            vel_y = (ty_arr, vy_arr)
            
            return vel_x, vel_y
        return None
    
    @property
    def acceleration_profiles(self) -> tuple | None:
        """
        Returns the acceleration profile of the X-axis and Y-axis movement.
        
        Each acceleration profile is a tuple of two arrays: the time values and 
        the corresponding acceleration values.
        """
        if self._xy_motion_proc is not None:
            tx_arr, alphax_arr = self._mp_x.acceleration_profile()
            ax_arr = alphax_arr / self._xy_motion_proc._N
            acc_x = (tx_arr, ax_arr)
            
            ty_arr, alphay_arr = self._mp_y.acceleration_profile()
            ay_arr = alphay_arr / self._xy_motion_proc._N
            acc_y = (ty_arr, ay_arr)
            
            return acc_x, acc_y
        return None

    
class Trajectory2D(list[Segment2D]):
    """
    Represents a 2D trajectory in the XY-plane. A `Trajectory2D` object is a
    list of `Segment2D` objects.
    """
    def _connect_segments(self):
        """
        Connects the motion profiles of the successive segments in the 
        trajectory.
        """
        def _connect_vel(*segments: Segment2D):
            """Connects the velocity profiles."""
            vel_x_lst, vel_y_lst = zip(*[
                segment.velocity_profiles 
                for segment in segments
            ])
            vel_x_connected = _connect(*vel_x_lst)
            vel_y_connected = _connect(*vel_y_lst)
            return vel_x_connected, vel_y_connected

        def _connect_pos(*segments: Segment2D):
            """Connects the position profiles."""
            pos_x_lst, pos_y_lst = zip(*[
                segment.position_profiles 
                for segment in segments
            ])
            pos_x_connected = _connect(*pos_x_lst)
            pos_y_connected = _connect(*pos_y_lst)
            return pos_x_connected, pos_y_connected

        def _connect_acc(*segments: Segment2D):
            """Connects the acceleration profiles."""
            acc_x_lst, acc_y_lst = zip(*[
                segment.acceleration_profiles 
                for segment in segments
            ])
            acc_x_connected = _connect(*acc_x_lst)
            acc_y_connected = _connect(*acc_y_lst)
            return acc_x_connected, acc_y_connected

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
            }
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
            }
        }
        return pos_profile, vel_profile, acc_profile
    
    @property
    def motion_profiles(self) -> tuple[dict[str, Any], ...] | None:
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
            }
        }
        ```
        """
        if len(self) > 0:
            pos, vel, acc = self._connect_segments()
            return pos, vel, acc
        return None


@dataclass
class Segment2DData:
    """
    An intermediate data class used by class `Trajectory2DPlanner` for 
    temporarily holding data about the segements while planning the trajectory.
    """
    segment: TSegment
    dtheta_x: float
    dtheta_y: float
    v_xi: float | None = None
    v_xf: float | None = None 
    v_yi: float | None = None
    v_yf: float | None = None


class Trajectory2DPlanner:
    """
    Creates a `Trajectory2D` object. 

    A `Trajectory2D` object is a list of `Segment2D` objects. A `Segment2D` 
    object contains the motion profiles of the X-axis and Y-axis movement. It 
    can also contain the step pulse trains (lists of time delays between 
    successive step pulses) for driving the X-axis and Y-axis stepper motors. 
    
    The `Trajectory2DPlanner` determines the boundary velocities of the segments
    (the X- and Y-components of the velocity at the start point and at the end 
    point of each segment). Subsequently, all segments are transformed into 
    fully fledged `Segment2D` objects held inside a Trajectory2D` object.
    """
    def __init__(
        self,
        pitch: float,
        motor_speed: float,
        motor_accel: float,
        full_steps_per_rev: int = 200,
        microstep_factor: int = 1,
        profile_type: MotionProfileType = MotionProfileType.S_CURVED,
    ) -> None:
        """
        Initializes a `Trajectory2DPlanner` object.
        
        Parameters
        ----------
        pitch: float
            Pitch of the lead screw (i.e. number of revolutions of the screw to
            move the nut one meter). Converts linear to rotational motion.
        motor_speed: float
            Maximum or available angular speed of the motor (deg/s).
        motor_accel: float
            Maximum or available angular acceleration of the motor (deg/s²). 
            This will depend on the torque capabilities of the motor and on the 
            total inertia moment of the motor and the connected load.
        full_steps_per_rev : int, optional
            Full steps per revolution of the stepper motor. If provided, this 
            will override the current setting. Defaults to 200.
        microstep_factor : int, optional
            Inverse of the microstep resolution to be used by the stepper motor.
            Defaults to 1, which means full step mode (no microstepping). If, 
            for example, the desired microstep resolution should be 1/16, set
            `microstep_factor` to 16.
        profile_type : MotionProfileType
            The type of motion profile. Either a trapezoidal (default) or 
            S-curved motion profile.
        
        Notes
        -----
        Assumption is made that the lead screws of the X- and Y-axis and also 
        the X- and Y-axis stepper motor are both identical.
        """
        self.pitch = pitch
        self.v_m = motor_speed
        self.a_m = motor_accel
        self._profile_type = SCurvedProfile
        if profile_type == MotionProfileType.TRAPEZOIDAL:
            self._profile_type = TrapezoidalProfile
        
        # Connect `XYMotionProcessor` object to the `Segment2D` class.
        xy_motion_processor = XYMotionProcessor(pitch, motor_speed, motor_accel, profile_type)
        Segment2D.connect_xy_motion_processor(xy_motion_processor)
        
        # Connect `FakeStepperMotor` objects to the `Segment2D` class.
        x_motor = FakeStepperMotor(full_steps_per_rev, microstep_factor)
        y_motor = FakeStepperMotor(full_steps_per_rev, microstep_factor)
        Segment2D.connect_stepper_motors(x_motor, y_motor)

    def _minimize_profile_time(self, ds_tot: float, v_other: float, find: str) -> float:
        def fn(v: float) -> float:
            try:
                kwargs = {"v_i": v, "v_f": v_other} if find == "v_i" else {"v_i": v_other, "v_f": v}
                mp = self._profile_type(ds_tot=ds_tot, a_m=self.a_m, v_m=self.v_m, **kwargs)
                return mp.dt_tot
            except ValueError:
                return float("inf")

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            max_speed = min(self.v_m, np.sqrt(2 * self.a_m * ds_tot))
            r = minimize_scalar(fn, bounds=(1.e-6, max_speed), method="bounded")
        if r.success:
            return float(r.x)
        raise ValueError(f"{find} could not be determined")
    
    def _get_angular_displacements(self, segment: TSegment) -> tuple[float, float]:
        xi = segment[0][0]
        yi = segment[0][1]
        xf = segment[1][0]
        yf = segment[1][1]
        dx = xf - xi
        dy = yf - yi
        dtheta_x = round(360.0 * self.pitch * dx, 6)
        dtheta_y = round(360.0 * self.pitch * dy, 6)
        return dtheta_x, dtheta_y

    def _create_segmentdata(self, pts: list[TPoint]) -> list[Segment2DData]:
        segments = list(zip(pts[:-1], pts[1:]))
        i_max = len(segments) - 1
        segmentdata_lst = []
        for i, segment in enumerate(segments):
            dtheta_x, dtheta_y = self._get_angular_displacements(segment)
            segmentdata = Segment2DData(segment, dtheta_x, dtheta_y)
            if i == 0:
                segmentdata.v_xi = 0.0
                segmentdata.v_yi = 0.0
            if i == i_max:
                segmentdata.v_xf = 0.0
                segmentdata.v_yf = 0.0
            segmentdata_lst.append(segmentdata)
        return segmentdata_lst

    def _analyze_segmentdata(self, segmentdata_lst: list[Segment2DData]) -> None:
        i_max = len(segmentdata_lst) - 1
        
        i = 0
        for segmentdata1 in segmentdata_lst:
            if i < i_max:
                segmentdata2 = segmentdata_lst[i + 1]
                dtheta_x1 = segmentdata1.dtheta_x
                dtheta_x2 = segmentdata2.dtheta_x
                if dtheta_x1 == 0.0:
                    segmentdata1.v_xi = 0.0
                    segmentdata1.v_xf = 0.0
                    segmentdata2.v_xi = 0.0
                if dtheta_x2 == 0.0 or dtheta_x1 * dtheta_x2 < 0.0:
                    segmentdata1.v_xf = 0.0
                    segmentdata2.v_xi = 0.0

                dtheta_y1 = segmentdata1.dtheta_y
                dtheta_y2 = segmentdata2.dtheta_y
                if dtheta_y1 == 0.0:
                    segmentdata1.v_yi = 0.0
                    segmentdata1.v_yf = 0.0
                    segmentdata2.v_yi = 0.0
                if dtheta_y2 == 0.0 or dtheta_y1 * dtheta_y2 < 0.0:
                    segmentdata1.v_yf = 0.0
                    segmentdata2.v_yi = 0.0

            i += 1
        
        i = i_max
        for segmentdata2 in reversed(segmentdata_lst):
            if i > 0:
                segmentdata1 = segmentdata_lst[i - 1]

                if segmentdata2.v_xi is None:
                    segmentdata2.v_xi = self._minimize_profile_time(
                        ds_tot=segmentdata2.dtheta_x,
                        v_other=segmentdata2.v_xf,
                        find="v_i"
                    )
                    segmentdata1.v_xf = segmentdata2.v_xi

                if segmentdata2.v_yi is None:
                    segmentdata2.v_yi = self._minimize_profile_time(
                        ds_tot=segmentdata2.dtheta_y,
                        v_other=segmentdata2.v_yf,
                        find="v_i"
                    )
                    segmentdata1.v_yf = segmentdata2.v_yi
            i -= 1
        
        i = 0
        for segmentdata1 in segmentdata_lst:
            if i < i_max:
                segmentdata2 = segmentdata_lst[i + 1]

                if segmentdata1.v_xf > 0.0:
                    v_xf = self._minimize_profile_time(
                        ds_tot=segmentdata1.dtheta_x,
                        v_other=segmentdata1.v_xi,
                        find="v_f"
                    )
                    v_xf = min(segmentdata1.v_xf, v_xf)
                    segmentdata1.v_xf = v_xf
                    segmentdata2.v_xi = v_xf

                if segmentdata1.v_yf > 0.0:
                    v_yf = self._minimize_profile_time(
                        ds_tot=segmentdata1.dtheta_y,
                        v_other=segmentdata1.v_yi,
                        find="v_f"
                    )
                    v_yf = min(segmentdata1.v_yf, v_yf)
                    segmentdata1.v_yf = v_yf
                    segmentdata2.v_yi = v_yf
            i += 1
    
    def get_trajectory(self, *points: TPoint) -> Trajectory2D:
        """
        Creates a `Trajectory2D` object from a sequence of points. 
        
        A point is specified by a two-element tuple containing the X- and 
        Y-coordinate of that point.
        """
        trajectory = Trajectory2D()
        segmentdata_lst = self._create_segmentdata(list(points))
        self._analyze_segmentdata(segmentdata_lst)
        for segment_data in segmentdata_lst:
            segment_obj = Segment2D(
                start=segment_data.segment[0],
                end=segment_data.segment[1],
                ini_speed=(segment_data.v_xi, segment_data.v_yi),
                fin_speed=(segment_data.v_xf, segment_data.v_yf)
            )
            trajectory.append(segment_obj)
        return trajectory
