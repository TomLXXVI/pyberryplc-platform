from pyberryplc.motion.multi_axis import MotionProfile


class DynamicDelayGenerator:
    """
    Generates delays between step pulses based on the current phase of motion
    (acceleration, cruising, deceleration).
    """
    def __init__(
        self, 
        step_angle: float, 
        profile: MotionProfile
    ) -> None:
        """
        Creates a `DynamicDelayGenerator`.
        
        Parameters
        ----------
        step_angle:
            The step angle of the stepper motor.
        profile:
            Motion profile used for controlling the acceleration and 
            deceleration of the stepper motor.
        """
        self.profile = profile
        self.step_angle = step_angle

        self.s = 0.0  # current position pointer
        self.t = 0.0  # current time pointer
        self.step_index = 0
        
        self._accel_vel_fn = self.profile.get_ini_velocity_time_fn()
        self._accel_time_fn = self.profile.get_ini_time_position_fn()
        self._decel_vel_fn = None
        self._decel_time_fn = None

        self.phase = "accel"  # a new rotation always starts in the acceleration phase
        self.cruise_velocity = self._accel_vel_fn(self.profile.dt_i)  # velocity at the end of the initial acceleration phase
        self._stop_velocity_threshold = 1e-3

    def trigger_decel(self):
        """
        Triggers the deceleration phase of the motion. This function must be
        called externally to activate the deceleration phase of the motor and 
        stop the motor.
        """
        self.phase = "decel"
        t0 = self.t                   # start time of the deceleration phase
        s0 = self.s                   # start position of the deceleration phase
        v0 = self._accel_vel_fn(t0)   # initial velocity at the start of the deceleration phase (cruise velocity)
        
        self._decel_vel_fn = self.profile.get_fin_velocity_time_fn(t0, v0)
        self._decel_time_fn = self.profile.get_fin_time_position_fn(t0, s0, v0)

    def next_delay(self) -> float:
        """
        Returns the time delay between the current step pulse and the next
        step pulse.
        """
        if self.phase == "done": raise StopIteration("Motion complete.")
        
        target_s = self.s + self.step_angle  # next position = current position + step angle
        t_new = 0.0  # declare the next time moment a step pulse must be send to the driver
        
        if self.phase == "accel":
            t_new = self._accel_time_fn(target_s)
            # when the next time moment exceeds the acceleration time duration 
            # of the motion profile, continue to the cruising phase of the 
            # rotation. 
            if t_new >= self.profile.dt_i:
                self.phase = "cruise"
                t_new = self.t + self.step_angle / self.cruise_velocity
        
        elif self.phase == "cruise":
            t_new = self.t + self.step_angle / self.cruise_velocity

        # the deceleration phase must be triggered externally by calling method
        # `trigger_decel()`.
        elif self.phase == "decel":
            t_new = self._decel_time_fn(target_s)
            v_new = self._decel_vel_fn(t_new)
            if v_new < self._stop_velocity_threshold:
                # when the velocity is close to zero, the end of the rotation is 
                # signaled by raising `StopIteration`.
                self.phase = "done"
                raise StopIteration
        delay = t_new - self.t
        self.s += self.step_angle  # move the current position pointer to the next step pulse position  
        self.t = t_new  # move the current time pointer to the time moment for the next pulse
        return delay
