"""
Motion Planning
---------------
Demo on how to use the `TrajectorPlanner` class to create the axis motion 
profiles of line segments that compose a trajectory, and to generate the step
pulse signals for driving the axis stepper motors.

What the demo script does:
-   A CSV file with the coordinates of the vertices of the trajectory is loaded 
    from file.
-   The `TrajectoryPlanner` creates and returns a `Trajectory` object. Motion 
    profiles of the axes are optimized to minimize path deviations.
-   The position, velocity, and acceleration profiles along the trajectory are
    plotted.
-   An XY plot of the segments paths is plotted, showing the path deviations 
    along the segments of the trajectory.
-   Data about the segments in the trajectory is printed.
-   The average path deviation along the trajectory is printed and the estimated 
    time duration to execute the trajectory.
-   Finally, the step pulse signals of each segment are saved as a list in a 
    JSON-file. This JSON-file can then be loaded into a `XYZMotionController`:
    see `motion_plc.py` in `demos/demo_stepper_02`.
"""
import warnings
import time
import pathlib

from pyberryplc.motion.trajectory import TrajectoryPlanner, StepperMotorMock
from pyberryplc.motion.multi_axis import SCurvedProfile, RotationDirection
from pyberryplc.charts import LineChart
from pyberryplc.motion.utils import (
    read_2Dtrajectory_csv,
    plot_position_profiles,
    plot_velocity_profiles,
    plot_acceleration_profiles,
    get_2Dtrajectory_path,
)


warnings.filterwarnings("ignore", category=UserWarning)


def main():
    
    input_path = "C:/Users/Tom/pycharm-projects/pyberryplc-platform/demos/demo_stepper_motion/demo_points.csv"
    points = read_2Dtrajectory_csv(input_path)
    points = [(x / 1000, y / 1000) for x, y in points]  # convert mm to m
    
    start = time.time()

    planner = TrajectoryPlanner(
        pitch=250,  # revs/m
        profile_type=SCurvedProfile,
        a_m=1000.0,  # deg/s2
        v_m=360.0,  # deg/s
        x_motor=StepperMotorMock(full_steps_per_rev=200, microstep_factor=1),
        y_motor=StepperMotorMock(full_steps_per_rev=200, microstep_factor=1),
        y_rdir_pos=RotationDirection.CLOCKWISE
    )
    trajectory = planner.get_trajectory(
        *points, 
        optimize=True, 
        allowed_path_deviation=1.e-4  # m --> 0.1 mm
    )
    
    end = time.time()
    print(f"Motion planning finished in {end - start:.0f} s.")
    
    ppf, vpf, apf = trajectory.motion_profiles
    pchart = plot_position_profiles(ppf); pchart.show()
    vchart = plot_velocity_profiles(vpf); vchart.show()
    achart = plot_acceleration_profiles(apf); achart.show()
    
    path, points = get_2Dtrajectory_path(trajectory)
    chart = LineChart()
    chart.add_xy_data(
        label="path",
        x1_values=path[0],
        y1_values=path[1]
    )
    chart.add_xy_data(
        label="points",
        x1_values=points[0],
        y1_values=points[1]
    )
    chart.show()

    for segment in trajectory:
        print(f"segment: {segment.point_pair}")
        print(f"p_dev = {segment.path_deviation:.6f}")
        print(
            f"x-axis: v_i = {segment.mp_x.v_i:.3f}, "
            f"v_f = {segment.mp_x.v_f:.3f}, "
            f"dt_tot = {segment.mp_x.dt_tot:.3f}, "
            f"v_m = {segment.mp_x.v_m:.3f}, "
            f"a_top = {segment.mp_x.a_top:.3f}, "
            f"v_top = {segment.mp_x.v_top:.3f}, "
        )
        print(
            f"y-axis: v_i = {segment.mp_y.v_i:.3f}, "
            f"v_f = {segment.mp_y.v_f:.3f}, "
            f"dt_tot = {segment.mp_y.dt_tot:.3f}, "
            f"v_m = {segment.mp_y.v_m:.3f}, "
            f"a_top = {segment.mp_y.a_top:.3f}, "
            f"v_top = {segment.mp_y.v_top:.3f}, "
        )
        print(
            f"z-axis: v_i = {segment.mp_z.v_i:.3f}, "
            f"v_f = {segment.mp_z.v_f:.3f}, "
            f"dt_tot = {segment.mp_z.dt_tot:.3f}, "
            f"v_m = {segment.mp_z.v_m:.3f}, "
            f"a_top = {segment.mp_z.a_top:.3f}, "
            f"v_top = {segment.mp_z.v_top:.3f}, "
        )
        print()
    
    print(f"Path deviation: {trajectory.get_path_deviation() * 1000:.3f} mm")
    print(f"Trajectory duration: {trajectory.duration:.3f} s")
    print()
    
    output_path = pathlib.Path("C:/Users/Tom/pycharm-projects/pyberryplc-platform/demos/demo_stepper_motion")
    file_name = "trajectory01.json"
    trajectory.save_stepper_driver_signals(str(output_path / file_name))


if __name__ == '__main__':
    main()
    