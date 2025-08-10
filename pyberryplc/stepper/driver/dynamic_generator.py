

class DynamicDelayGenerator:
    """
    Generates delays between step pulses based on the current phase of motion
    (acceleration, cruising, deceleration).
    """
    def __init__(
        self, 
        step_angle: float, 
        profile
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
        
        self._v_accel = self.profile.get_ini_velocity_from_time_fn()
        self._t_accel = self.profile.get_ini_time_from_position_fn()
        self._v_decel = None
        self._t_decel = None

        self.phase = "accel"  # a new rotation always starts in the acceleration phase
        self.v_cruise = self._v_accel(self.profile.dt_i)  # velocity at the end of the initial acceleration phase
        self.v_min = 1e-3

    def trigger_decel(self):
        """
        Triggers the deceleration phase of the motion. This function must be
        called externally to activate the deceleration phase of the motor and 
        stop the motor.
        """
        t0 = self.t                   # start time of the deceleration phase
        s0 = self.s                   # start position of the deceleration phase
        v0 = self.v_cruise            # initial velocity at the start of the deceleration phase (cruise velocity)
        self._v_decel = self.profile.get_fin_velocity_from_time_fn(t0, v0)
        self._t_decel = self.profile.get_fin_time_from_position_fn(t0, s0, v0)
        self.phase = "decel"
        
    def next_delay(self) -> float:
        """
        Returns the time delay between the current step pulse and the next
        step pulse.
        """
        if self.phase == "done": raise StopIteration("Motion complete.")
        
        target_s = self.s + self.step_angle  # next position = current position + step angle
        t_new = 0.0  # declare the next time moment a step pulse must be sent to the driver
        
        if self.phase == "accel":
            t_new = self._t_accel(target_s)
            # when the next time moment exceeds the acceleration time duration 
            # of the motion profile, continue to the cruising phase of the 
            # rotation. 
            if t_new >= self.profile.dt_i:
                self.phase = "cruise"
                t_new = self.t + self.step_angle / self.v_cruise
        
        elif self.phase == "cruise":
            t_new = self.t + self.step_angle / self.v_cruise

        # the deceleration phase must be triggered externally by calling method
        # `trigger_decel()`.
        elif self.phase == "decel":
            t_new = self._t_decel(target_s)
            v_new = self._v_decel(t_new)
            if v_new < self.v_min:
                # when the velocity is close to zero, the end of the rotation is 
                # signaled by raising `StopIteration`.
                self.phase = "done"
                raise StopIteration
        delay = t_new - self.t
        self.s += self.step_angle  # move the current position pointer to the next step pulse position  
        self.t = t_new  # move the current time pointer to the time moment for the next pulse
        return delay
