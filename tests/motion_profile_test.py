from pyberryplc.motion_profiles import TrapezoidalProfile

from charts import LineChart

p = TrapezoidalProfile(
    ds_tot=720.0,
    dt_tot=2.0,
    dt_acc=0.5
)
t_ax, v_ax = p.velocity_profile()

chart = LineChart()
chart.add_xy_data(
    label='velocity profile',
    x1_values=t_ax,
    y1_values=v_ax
)
chart.x1.add_title('time, s')
chart.y1.add_title('velocity')
chart.show()

full_steps_per_rev = 200
microstep_factor = 2
step_width = 10e-6

steps_per_degree = full_steps_per_rev * microstep_factor / 360
step_angle = 1 / steps_per_degree

start_angle = 0.0
final_angle = p.ds_tot + step_angle
angles = [
    start_angle + i * step_angle 
    for i in range(int(final_angle / step_angle))
]
times = list(map(p.get_fn_time_from_position(), angles))
delays = [t2 - t1 for t1, t2 in zip(times, times[1:])]


print(angles[-1])
print(times[-1])
print(sum(delays), len(delays))
print(max(delays), min(delays))
