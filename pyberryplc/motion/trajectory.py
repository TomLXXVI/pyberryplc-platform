from dataclasses import dataclass

from .multi_axis import (
    Direction, 
    MotionProfile, 
    TrapezoidalProfile, SCurvedProfile, 
    ProfileType
)

from pyberryplc.stepper.driver.process import TwoStageProfileRotator


Point = tuple[float, float]


class XYMotionController:
    """
    Generates the motion profiles along the X- and Y-axis to move from one 
    point to another point along a straight line segment (synchronised X- and 
    Y-axis motion).
    """
    def __init__(
        self,
        pitch: float,
        motor_speed: float,
        motor_accel: float,
        profile_type: ProfileType = ProfileType.TRAPEZOIDAL
    ) -> None:
        """Initializes the `XYMotionControl` object.

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
        if profile_type in ProfileType.get_types():
            self.profile_type = profile_type
        else:
            raise TypeError(f"Profile type '{profile_type}' does not exist.")
        self.pitch = pitch
        self.motor_speed = motor_speed
        self.motor_accel = motor_accel

        self._N = 360.0 * self.pitch  # convert revs/m -> deg/m
        self._pt_start: Point = (0.0, 0.0)
        self._pt_end: Point = (0.0, 0.0)
        self._angle_ini_x: float = 0.0
        self._angle_ini_y: float = 0.0
        self._speed_ini_x: float = 0.0
        self._speed_ini_y: float = 0.0
        self._speed_fin_x: float | None = None
        self._speed_fin_y: float | None = None
        self._direction_x: Direction = Direction.COUNTERCLOCKWISE
        self._direction_y: Direction = Direction.COUNTERCLOCKWISE
        self._dx: float = 0.0
        self._dy: float = 0.0
        self._dtheta_x: float = 0.0
        self._dtheta_y: float = 0.0
        self._mp_x: MotionProfile | None = None
        self._mp_y: MotionProfile | None = None

    def set_points(self, pt_start: Point, pt_end: Point) -> None:
        """Sets the start and end point of the linear motion."""
        self._direction_x = Direction.COUNTERCLOCKWISE
        self._direction_y = Direction.COUNTERCLOCKWISE
        
        self._pt_start = pt_start
        self._pt_end = pt_end

        self._angle_ini_x = self._N * pt_start[0]
        self._angle_ini_y = self._N * pt_start[1]

        self._dx = self._pt_end[0] - self._pt_start[0]
        self._dy = self._pt_end[1] - self._pt_start[1]
        if self._dx < 0.0:
            self._direction_x = Direction.CLOCKWISE
            self._dx = abs(self._dx)
        if self._dy < 0.0:
            self._direction_y = Direction.CLOCKWISE
            self._dy = abs(self._dy)
        
        self._dtheta_x = self._N * self._dx
        self._dtheta_y = self._N * self._dy
        
        # Erase existing motion profiles
        self._mp_x, self._mp_y = None, None

    def set_boundary_velocities(
        self,
        speed_ini_x: float = 0.0,
        speed_ini_y: float = 0.0,
        speed_fin_x: float | None = None,
        speed_fin_y: float | None = None
    ) -> None:
        """Sets the initial and final velocities of the linear motion in x and y.

        The final velocity can be `None`, meaning that it is unknown. In that
        case the final velocity will be equal to the top velocity of the 
        motion.
        """
        self._speed_ini_x = speed_ini_x
        self._speed_ini_y = speed_ini_y
        self._speed_fin_x = speed_fin_x
        self._speed_fin_y = speed_fin_y

        # Erase existing motion profiles
        self._mp_x, self._mp_y = None, None

    def _create_motion_profile(self, **kwargs) -> MotionProfile | None:
        # noinspection PyUnreachableCode
        match self.profile_type:
            case ProfileType.TRAPEZOIDAL:
                _MotionProfile = TrapezoidalProfile
            case ProfileType.S_CURVED:
                _MotionProfile = SCurvedProfile
            case _:
                _MotionProfile = TrapezoidalProfile

        mp = _MotionProfile(**kwargs)
        return mp

    def _calc_motion_profiles(self) -> None:
        """Calculates the synchronised profiles of X- and Y-axis motion.

        The X- and Y-axis motion will have the same start and ending time,
        independent of their displacement.
        """
        # Calculate the time it takes for each movement (x and y) to do the
        # displacement at their initial velocity.
        if self._speed_ini_x > 0.0 and self._speed_ini_y > 0.0:
            dt_x = self._dtheta_x / self._speed_ini_x
            dt_y = self._dtheta_y / self._speed_ini_y
        else:
            dt_x = self._dtheta_x / self.motor_speed
            dt_y = self._dtheta_y / self.motor_speed
        
        # We take the movement that needs the longest travel time (smallest
        # initial velocity and/or largest displacement) as the reference.
        l_time = [dt_x, dt_y]
        dt_tot = max(l_time)
        index = l_time.index(dt_tot)
        if index == 0:
            ds_tot = self._dtheta_x
            v_ini = self._speed_ini_x
            v_fin = self._speed_fin_x
            s_ini = self._angle_ini_x
        else:
            ds_tot = self._dtheta_y
            v_ini = self._speed_ini_y
            v_fin = self._speed_fin_y
            s_ini = self._angle_ini_y

        # If the initial velocity of the reference movement is lower than the 
        # allowable motor speed, we initially try to accelerate this movement to
        # maximum motor speed. However, it might happen then that the other 
        # movement does not have enough time to finish. If this happens, we 
        # gradually increase the travel time of the first motion. 
        if v_ini <= self.motor_speed:
            i_max = 10
            i = 0
            while i < i_max:
                if i == 0: dt_tot = None  # Indicates to use maximum motor speed
                mp_tmp = self._create_motion_profile(
                    ds_tot=ds_tot,
                    v_m=self.motor_speed,
                    a_m=self.motor_accel,
                    dt_tot=dt_tot,
                    v_ini=v_ini,
                    v_fin=v_fin,
                    s_ini=s_ini
                )
                dt_tot = mp_tmp.dt_tot
                if index == 0:
                    self._mp_x = mp_tmp
                    try:
                        self._mp_y = self._create_motion_profile(
                            ds_tot=self._dtheta_y,
                            v_m=self.motor_speed,
                            a_m=self.motor_accel,
                            dt_tot=dt_tot,
                            v_ini=self._speed_ini_y,
                            v_fin=self._speed_fin_y,
                            s_ini=self._angle_ini_y
                        )
                    except ValueError:
                        dt_tot *= 1.1
                        i += 1
                        continue
                else:
                    self._mp_y = mp_tmp
                    try:
                        self._mp_x = self._create_motion_profile(
                            ds_tot=self._dtheta_x,
                            v_m=self.motor_speed,
                            a_m=self.motor_accel,
                            dt_tot=dt_tot,
                            v_ini=self._speed_ini_x,
                            v_fin=self._speed_fin_x,
                            s_ini=self._angle_ini_x
                        )
                    except ValueError:
                        dt_tot *= 1.1
                        i += 1
                        continue
                break

    @property
    def x_motion(self) -> tuple[MotionProfile, Direction]:
        """Returns the motion profile and rotation direction for the X-axis."""
        if self._mp_x is None: self._calc_motion_profiles()
        return self._mp_x, self._direction_x

    @property
    def y_motion(self) -> tuple[MotionProfile, Direction]:
        """Returns the motion profile and rotation direction for the Y-axis."""
        if self._mp_y is None: self._calc_motion_profiles()
        return self._mp_y, self._direction_y


class _DummyStepperMotor:
    """Represents a dummy `StepperMotor` object to calculate the step pulse 
    train (actually, the delays between successive step pulses) to drive a 
    stepper motor.
    
    This class is only intended as a "hack" to get access to the function 
    `preprocess()` of class `TwoStageProfileRotator(ProfileRotator)` that 
    calculates the delays between successive step pulses to drive a stepper 
    motor.
    """
    def __init__(
        self,
        full_steps_per_rev: int = 200,
        microstep_factor: int = 1
    ) -> None:
        """Creates a `DummyStepperMotor` object with just the attributes that 
        are  needed to generate the step pulse train (delays between step 
        pulses).
        
        Parameters
        ----------
        full_steps_per_rev : int, optional
            Full steps per revolution of the motor. If provided, this will 
            override the current setting. Defaults to 200.
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


class Segment:
    """
    Represents a single, linear segment in a trajectory in the (X,Y)-plane.

    Given the start- and end point of the segment, the motion profile and 
    rotation direction along the X-axis, and along the Y-axis are calculated.
    For this, an `XYMotionController` object must be linked to the `Segment` 
    class. See class method `set_xy_motion_control()`.

    If `DummyStepperMotor` objects for the X- and Y-axis are linked to the 
    `Segment` class, the delays between successive step pulses to drive the X- 
    and Y-axis stepper motors, can also be calculated. Use class method 
    `set_stepper_motors()` to link the `DummyStepperMotor` objects to the 
    `Segment` class.
    """
    _xy_motion_control: XYMotionController = None
    _x_motor: _DummyStepperMotor = None
    _y_motor: _DummyStepperMotor = None

    @classmethod
    def set_xy_motion_control(cls, xy_motion_control: XYMotionController) -> None:
        cls._xy_motion_control = xy_motion_control

    @classmethod
    def set_stepper_motors(
        cls, 
        x_motor: _DummyStepperMotor, 
        y_motor: _DummyStepperMotor
    ) -> None:
        cls._x_motor = x_motor
        cls._y_motor = y_motor

    def __init__(
        self,
        start_point: Point,
        end_point: Point,
        speed_ini_x: float = 0.0,
        speed_ini_y: float = 0.0,
        speed_fin_x: float | None = None,
        speed_fin_y: float | None = None
    ) -> None:
        """Creates a `Segment` object.

        Parameters
        ----------
        start_point: tuple[float, float]
            Start point of the segment.
        end_point: tuple[float, float]
            End point of the segment.
        speed_ini_x: float
            Initial speed in the X-axis direction at the start point of the 
            segment.
        speed_ini_y: float
            Initial speed in the Y-axis direction at the start point of the 
            segment.
        speed_fin_x: float
            Final speed in the X-axis direction at the end point of the segment.
        speed_fin_y: float
            Final speed in the Y-axis direction at the end point of the segment.
        """
        self.pt1 = start_point
        self.pt2 = end_point

        self._mp_x, self._dir_x = None, None
        self._mp_y, self._dir_y = None, None

        if self._xy_motion_control is not None:
            # Pass start and end point and boundary velocities to `XYMotionController`
            self._xy_motion_control.set_points(start_point, end_point)
            self._xy_motion_control.set_boundary_velocities(
                speed_ini_x, speed_ini_y,
                speed_fin_x, speed_fin_y
            )
            
            # Calculate motion profiles of X and Y
            self._mp_x, self._dir_x = self._xy_motion_control.x_motion
            self._mp_y, self._dir_y = self._xy_motion_control.y_motion
            
            # Calculate step pulse train to drive X and Y motor
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
    def x_motion(self) -> tuple[MotionProfile, Direction] | None:
        """Returns the segment's motion profile and rotation direction along 
        the X-axis.
        """
        if self._xy_motion_control is not None:
            return self._mp_x, self._dir_x
        return None

    @property
    def y_motion(self) -> tuple[MotionProfile, Direction] | None:
        """Returns the segment's motion profile and rotation direction along 
        the Y-axis.
        """
        if self._xy_motion_control is not None:
            return self._mp_y, self._dir_y
        return None

    @property
    def x_delays(self) -> list[float] | None:
        """Returns the list of delays between pulses to drive the X-axis motor."""
        if self._x_rotator is not None:
            # noinspection PyProtectedMember
            return self._x_rotator._delays
        return None

    @property
    def y_delays(self) -> list[float] | None:
        """Returns the list of delays between pulses to drive the Y-axis motor."""
        if self._y_rotator is not None:
            # noinspection PyProtectedMember
            return self._y_rotator._delays
        return None

    @property
    def final_speed_x(self) -> float | None:
        """Returns the x-component of the final speed at the end point of the
        segment.
        """
        if self._mp_x is not None:
            return self._mp_x.v_fin
        return None
    
    @property
    def final_speed_y(self) -> float | None:
        """Returns the y-component of the final speed at the end point of the
        segment.
        """
        if self._mp_y is not None:
            return self._mp_y.v_fin
        return None


class Trajectory(list[Segment]):
    """A `Trajectory` object is a list of `Segment` objects."""
    pass


class TrajectoryPlanner:
    """Creates a `Trajectory` object from a sequence of line segments. 
    
    A line segment is represented by a simple two-element tuple containing the 
    start and end point of the segment. Start and end point are also a 
    two-element tuples with the x- and y-coordinate of the point.
    
    A `Trajectory` object is a list of `Segment` objects. A `Segment` object
    contains the motion profiles of the X-axis and the Y-axis movement. It also 
    contains the lists of step pulse delays to drive the X-axis and Y-axis 
    stepper motors. 
    
    Based on the passed sequence of line segments, the `TrajectoryPlanner` 
    first determines the boundary velocities of the segments (the x- and 
    y-components of the initial velocity at the start point of each segment and 
    of the final velocity at the end point of each segment). Then each segment 
    in the passed sequence can be transformed into a fully fledged `Segment` 
    object (using the `XYMotionController` object inside the `Segment` object).
    """
    def __init__(
        self,
        pitch: float,
        motor_speed: float,
        motor_accel: float,
        full_steps_per_rev: int = 200,
        microstep_factor: int = 1,
        profile_type: ProfileType = ProfileType.TRAPEZOIDAL,
    ) -> None:
        """Initializes a `TrajectoryPlanner` object.
        
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
        profile_type : ProfileType
            The type of motion profile. Either a trapezoidal (default) or 
            S-curved motion profile.
        
        Notes
        -----
        It is assumed that the lead screws of X- and Y-axis and the X- and 
        Y-axis stepper motor are identical.
        """
        # Initialize class attributes of `Segment` class.
        xy_motion_control = XYMotionController(pitch, motor_speed, motor_accel, profile_type)
        x_motor = _DummyStepperMotor(full_steps_per_rev, microstep_factor)
        y_motor = _DummyStepperMotor(full_steps_per_rev, microstep_factor)
        Segment.set_xy_motion_control(xy_motion_control)
        Segment.set_stepper_motors(x_motor, y_motor)
    
    @staticmethod
    def create_trajectory(*segments: tuple[Point, Point]) -> Trajectory:
        """Creates a `Trajectory` object from a sequence of segments. A segment 
        is represented as a two-element tuple containing the start and end point
        of the line segment. A point is also a two-element tuple with the x- and
        y-coordinate of the point.
        """
        # Create a new `Trajectory` object.
        trajectory = Trajectory()
        
        # Create `Segment` objects and append them to the `Trajectory` object.
        speed_ini_x = 0.0
        speed_ini_y = 0.0
        i_max = len(segments) - 1
        for i, seg in enumerate(segments):
            # Determine final velocity in x and y at the end point of the 
            # segment to be created.
            if i < i_max:
                next_seg = segments[i + 1]
                next_dx = round(next_seg[1][0] - next_seg[0][0], 6)
                next_dy = round(next_seg[1][1] - next_seg[0][1], 6)
                if next_dx == 0.0:
                    speed_fin_x = 0.0
                else:
                    speed_fin_x = None
                if next_dy == 0.0:
                    speed_fin_y = 0.0
                else:
                    speed_fin_y = None
            else:  # i == i_max
                speed_fin_x = 0.0
                speed_fin_y = 0.0
            
            # Create `Segment` object and append it to the `trajectory` ->
            # this also calculates the motion profiles in x and y of the segment
            # and the step pulse train in x and y to drive the stepper motors.
            segment = Segment(
                start_point=seg[0],
                end_point=seg[1],
                speed_ini_x=speed_ini_x,
                speed_ini_y=speed_ini_y,
                speed_fin_x=speed_fin_x,
                speed_fin_y=speed_fin_y
            )
            trajectory.append(segment)
            
            # Get the calculated final velocity in x and y at the end point of
            # the current `Segment` object. This will be the initial velocity of
            # the next `Segment` object.
            mp_x, dir_x = segment.x_motion
            mp_y, dir_y = segment.y_motion
            speed_ini_x = mp_x.v_fin
            speed_ini_y = mp_y.v_fin
    
        return trajectory
