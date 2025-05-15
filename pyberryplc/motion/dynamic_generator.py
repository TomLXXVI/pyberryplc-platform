from .multi_axis import MotionProfile


class DynamicDelayGenerator:
    """
    Position-based dynamic delay generator for stepper motors.

    Generates delays between step pulses based on the current phase of motion:
    - Acceleration using profile.get_position_time_fn()
    - Cruise phase at constant top velocity
    - Deceleration triggered on demand, with interpolated decel position profile

    The generator tracks time and position internally. Once deceleration is 
    triggered, a local position interpolation is constructed from the current 
    (t, s, v) state.
    """
    def __init__(
        self, 
        step_angle: float, 
        profile: MotionProfile,
    ) -> None:
        self.profile = profile
        self.step_angle = step_angle

        self.s = 0.0  # current position (deg)
        self.t = 0.0  # current time (s)
        self.step_index = 0
        
        self._accel_fn_v_from_t = self.profile.get_ini_velocity_time_fn()
        self._accel_fn_t_from_s = self.profile.get_ini_time_position_fn()
        self._decel_fn_v_from_t = None
        self._decel_fn_t_from_s = None

        self.phase = "accel"
        self.cruise_velocity = self._accel_fn_v_from_t(self.profile.dt_ini)
        self._stop_velocity_threshold = 1e-3

    def trigger_decel(self):
        self.phase = "decel"
        t0 = self.t
        s0 = self.s
        v0 = self._accel_fn_v_from_t(t0)
        self._decel_fn_v_from_t = self.profile.get_fin_velocity_time_fn(t0, v0)
        self._decel_fn_t_from_s = self.profile.get_fin_time_position_fn(t0, s0, v0)

    def next_delay(self) -> float:
        if self.phase == "done":
            raise StopIteration("Motion complete.")

        target_s = self.s + self.step_angle
        t_new = 0.0

        if self.phase == "accel":
            t_new = self._accel_fn_t_from_s(target_s)
            if t_new >= self.profile.dt_ini:
                self.phase = "cruise"
                t_new = self.t + self.step_angle / self.cruise_velocity
                
        elif self.phase == "cruise":
            t_new = self.t + self.step_angle / self.cruise_velocity

        elif self.phase == "decel":
            t_new = self._decel_fn_t_from_s(target_s)
            v_new = self._decel_fn_v_from_t(t_new)
            if v_new < self._stop_velocity_threshold:
                self.phase = "done"
                raise StopIteration

        delay = t_new - self.t
        self.s += self.step_angle
        self.t = t_new
        return delay
