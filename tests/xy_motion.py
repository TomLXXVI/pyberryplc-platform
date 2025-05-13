from pyberryplc.motion import MotionProfile, xy_motion_control, get_pitch
from charts import LineChart

def get_step_angle(full_steps_per_rev: int = 200, microstep_factor: int = 1) -> float:
    steps_per_degree = full_steps_per_rev * microstep_factor / 360.0
    step_angle = 1 / steps_per_degree
    return round(step_angle, 6)


class DelayGenerator:
    
    def __init__(
        self,
        motion_profile: MotionProfile,
        step_angle: float,
        pulse_width: float = 10e-6
    ) -> None:
        self.motion_profile = motion_profile
        self.step_angle = step_angle
        self.pulse_width = pulse_width
        self.num_steps = None
        self.angles = None
        self.times = None
    
    def __call__(self) -> list[float]:
        final_angle = self.motion_profile.ds_tot
        self.num_steps = int(final_angle / self.step_angle)
        self.angles = [i * self.step_angle for i in range(self.num_steps + 1)]
        self.times = list(map(self.motion_profile.get_fn_time_from_position(), self.angles))
        # delays = [max(0.0, t2 - t1 - self.pulse_width) for t1, t2 in zip(self.times, self.times[1:])]
        delays = [t2 - t1 - self.pulse_width for t1, t2 in zip(self.times, self.times[1:])]
        return delays


def main():
    p1 = (0.0, 0.0)    
    p2 = (0.02, 0.02)  
    pitch = get_pitch(1, 0.01)
    print(pitch)
    omega = 45.0  # deg / s
    
    motion_x, motion_y = xy_motion_control(p1, p2, pitch, omega, 0.2, "trapezoidal")
    
    print("From motion profiles: ")
    print(f"X-axis angular speed: {motion_x.v_m:.3f} deg/s")
    print(f"Y-axis angular speed: {motion_y.v_m:.3f} deg/s")
    print(f"X-axis total travel time: {motion_x.dt_tot:.3f} s")
    print(f"Y-axis total travel time: {motion_y.dt_tot:.3f} s")
    print()
    
    pulse_width = 10e-6
    step_angle = get_step_angle(full_steps_per_rev=200, microstep_factor=2)
    
    print("Calculate delays in X and Y.")
    delay_generator_x = DelayGenerator(motion_x, step_angle, pulse_width)
    delay_generator_y = DelayGenerator(motion_y, step_angle, pulse_width)
    delays_in_x = delay_generator_x()
    delays_in_y = delay_generator_y()

    print(f"Motor step angle: {delay_generator_x.step_angle} deg")
    print(f"Number of steps: {delay_generator_x.num_steps}")
    print()
    
    total_delay_time_x = sum(delays_in_x)
    total_delay_time_y = sum(delays_in_y)
    
    total_pulse_time_x = delay_generator_x.num_steps * pulse_width
    total_pulse_time_y = delay_generator_y.num_steps * pulse_width
    
    total_travel_time_x = total_delay_time_x + total_pulse_time_x
    total_travel_time_y = total_delay_time_y + total_pulse_time_y
    
    print("Calculated from delays:")
    print(f"X-axis total travel time: {total_travel_time_x:.3f} s")
    print(f"Y-axis total travel time: {total_travel_time_y:.3f} s")
    print(f"X-axis final angle: {delay_generator_x.num_steps * step_angle} deg")
    print(f"Y-axis final angle: {delay_generator_y.num_steps * step_angle} deg")

    chart = LineChart()
    chart.add_xy_data(
        label="delays in x",
        x1_values=[i for i in range(delay_generator_x.num_steps)],
        y1_values=delays_in_x
    )
    chart.x1.add_title('step')
    chart.y1.add_title('delay')
    chart.show()
    
    print(delays_in_x[0], delays_in_x[delay_generator_x.num_steps - 1], min(delays_in_x))

if __name__ == '__main__':
    main()
    