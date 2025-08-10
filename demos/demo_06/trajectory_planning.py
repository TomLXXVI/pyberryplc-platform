import numpy as np

from pyberryplc.motion.trajectory import PointToPointTrajectory
from pyberryplc.motion.profile_alt import SCurvedProfile, RotationDirection
from pyberryplc.charts import LineChart


trajectory = PointToPointTrajectory(
    n_axes=2,
    profile_type=SCurvedProfile,
    pitch=250.0,  # revs / m (lead = 4 mm / rev)
    rdir_ref=RotationDirection.CCW,
    alpha_max=1500.0,  # deg / s2
    omega_max=720.0,  # deg / s
    full_steps_per_rev=200,
    microstep_factor=2
)

trajectory(
    (0.0, 0.0),
    (0.02, 0.02),
    (0.04, 0.02),
    (0.06, 0.0),
    (0.0, 0.0)
)


# Plot of the 2D trajectory.
traj_coords = trajectory.get_coordinates()
traj_plot = LineChart()
traj_plot.add_xy_data(
    label="",
    x1_values=traj_coords[0],
    y1_values=traj_coords[1]
)
traj_plot.x1.add_title("x")
traj_plot.y1.add_title("y")
traj_plot.show()


# Position, velocity, and acceleration timing profiles.
trajectory.position_profiles_plot.show()
trajectory.velocity_profiles_plot.show()
trajectory.acceleration_profiles_plot.show()


# Step pulse signals
stepper_signals = trajectory.get_stepper_signals()
stepper_x_pulse_delays, stepper_y_pulse_delays = [], []
for seg_stepper_signals in stepper_signals:
    pulse_delays_x, _ = seg_stepper_signals["x"]
    pulse_delays_y, _ = seg_stepper_signals["y"]
    stepper_x_pulse_delays.append(np.array(pulse_delays_x))
    stepper_y_pulse_delays.append(np.array(pulse_delays_y))
stepper_x_pulse_delays = np.hstack(stepper_x_pulse_delays)
stepper_y_pulse_delays = np.hstack(stepper_y_pulse_delays)
stepper_signals_plot = LineChart()
stepper_signals_plot.add_xy_data(
    label="x",
    x1_values=[i for i in range(len(stepper_x_pulse_delays))],
    y1_values=stepper_x_pulse_delays
)
stepper_signals_plot.add_xy_data(
    label="y",
    x1_values=[i for i in range(len(stepper_y_pulse_delays))],
    y1_values=stepper_y_pulse_delays
)
stepper_signals_plot.y1.add_title("pulse delays, seconds")
stepper_signals_plot.add_legend()
stepper_signals_plot.show()


# Save trajectory step pulse signals
trajectory.save_stepper_signals("trajectory.json")

