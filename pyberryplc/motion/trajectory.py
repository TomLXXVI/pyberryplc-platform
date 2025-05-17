from enum import StrEnum

from .multi_axis import Direction, MotionProfile, TrapezoidalProfile, SCurvedProfile

from pyberryplc.stepper.driver.base import StepperMotor
from pyberryplc.stepper.driver.process import ProfileRotatorProcess


class ProfileType(StrEnum):
    TRAPEZOIDAL = "trapezoidal"
    S_CURVED = "S-curved"

    @classmethod
    def get_types(cls) -> list[str]:
        return [cls.TRAPEZOIDAL, cls.S_CURVED]


Point = tuple[float, float]


class XYMotionControl:
    """
    Calculates the motion profiles along the X- and Y-axis to move from one 
    point to another point along a straight line (synchronised X- and Y-axis 
    motion).
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
        motor_speed:
            Available angular speed of the motor (deg/s).
        motor_accel:
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
        self._pt_start = pt_start
        self._pt_end = pt_end

        self._angle_ini_x = self._N * pt_start[0]
        self._angle_ini_y = self._N * pt_start[1]

        self._dx = self._pt_end[0] - self._pt_start[0]
        self._dy = self._pt_end[1] - self._pt_start[1]
        self._dtheta_x = self._N * self._dx
        self._dtheta_y = self._N * self._dy

        self._direction_x = Direction.COUNTERCLOCKWISE
        self._direction_y = Direction.COUNTERCLOCKWISE
        if self._dx < 0.0:
            self._direction_x = Direction.CLOCKWISE
        if self._dy < 0.0:
            self._direction_y = Direction.CLOCKWISE

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

    def _create_motion_profile(self, **kwargs) -> MotionProfile:
        # noinspection PyUnreachableCode
        match self.profile_type:
            case ProfileType.TRAPEZOIDAL:
                _MotionProfile = TrapezoidalProfile
            case ProfileType.S_CURVED:
                _MotionProfile = SCurvedProfile
            case _:
                _MotionProfile = TrapezoidalProfile
        return _MotionProfile(**kwargs)

    def _calc_motion_profiles(self) -> None:
        """Calculates the synchronised profiles of X- and Y-axis motion.

        The X- and Y-axis motion will have the same start and ending time,
        independent of their displacement.
        """
        # Calculate the time it takes for each movement (x and y) to make the
        # displacement at its constant initial velocity.
        if self._speed_ini_x > 0.0 and self._speed_ini_y > 0.0:
            dt_x = self._dtheta_x / self._speed_ini_x
            dt_y = self._dtheta_y / self._speed_ini_y
        else:
            dt_x = self._dtheta_x / self.motor_speed
            dt_y = self._dtheta_y / self.motor_speed
        l_time = [dt_x, dt_y]

        # We take the movement that needs the longest travel time (smallest
        # initial velocity and/or largest displacement).
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

        # If the initial velocity of this movement is lower than the maximum
        # motor speed, we accelerate this movement to the maximum motor speed.
        if v_ini < self.motor_speed:
            mp_tmp = self._create_motion_profile(
                ds_tot=ds_tot,
                v_m=self.motor_speed,
                a_m=self.motor_accel,
                v_ini=v_ini,
                v_fin=v_fin,
                s_ini=s_ini
            )
            if index == 0:
                self._mp_x = mp_tmp
                self._mp_y = self._create_motion_profile(
                    ds_tot=self._dtheta_y,
                    v_m=self.motor_speed,
                    a_m=self.motor_accel,
                    dt_tot=self._mp_x.dt_tot,
                    v_ini=self._speed_ini_y,
                    v_fin=self._speed_fin_y,
                    s_ini=self._angle_ini_y
                )
            else:
                self._mp_y = mp_tmp
                self._mp_x = self._create_motion_profile(
                    ds_tot=self._dtheta_x,
                    v_m=self.motor_speed,
                    a_m=self.motor_accel,
                    dt_tot=self._mp_y.dt_tot,
                    v_ini=self._speed_ini_x,
                    v_fin=self._speed_fin_x,
                    s_ini=self._angle_ini_x
                )

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


class Segment:
    """
    Represents a single, linear segment of a trajectory in the (X,Y)-plane.

    Given the start- and end point of the segment, the motion profile and 
    rotation direction along the X-axis, and along the Y-axis are calculated.
    For this, an `XYMotionControl` object must be linked to the `Segment` class.
    See class method `set_xy_motion_control()`.

    If the stepper motors of X- and Y-axis are linked to the `Segment` class,
    the delays between step pulses to drive the X- and the Y-axis stepper motor,
    can also be calculated. Use class method `set_stepper_motors()` to link the
    stepper motors.
    """
    _xy_motion_control: XYMotionControl = None
    _x_motor: StepperMotor = None
    _y_motor: StepperMotor = None

    @classmethod
    def set_xy_motion_control(cls, xy_motion_control: XYMotionControl) -> None:
        cls._xy_motion_control = xy_motion_control

    @classmethod
    def set_stepper_motors(cls, x_motor: StepperMotor, y_motor: StepperMotor) -> None:
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
            self._xy_motion_control.set_points(start_point, end_point)
            self._xy_motion_control.set_boundary_velocities(
                speed_ini_x, speed_ini_y,
                speed_fin_x, speed_fin_y
            )
            self._mp_x, self._dir_x = self._xy_motion_control.x_motion
            self._mp_y, self._dir_y = self._xy_motion_control.y_motion

            self._x_rotator, self._y_rotator = None, None
            if self._x_motor is not None:
                self._x_rotator = ProfileRotatorProcess(self._x_motor)
                self._x_rotator.preprocess(self._dir_x, self._mp_x)
            if self._y_motor is not None:
                self._y_rotator = ProfileRotatorProcess(self._y_motor)
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
