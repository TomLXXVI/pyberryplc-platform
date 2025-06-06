import csv

import numpy as np
from numpy.typing import NDArray

from pyberryplc.charts import LineChart
from pyberryplc.motion.trajectory import (
    Segment, 
    Trajectory, 
    TPointPair, 
    TMotionProfileDict,
    AxisData,
    TrajectoryPlanner
)
from pyberryplc.motion.multi_axis import MotionProfile, RotationDirection


def get_pitch(revs: int, distance: float) -> float:
    """Returns the pitch of a lead screw.

    Pitch is defined as the number of revolutions of the screw to travel the
    nut one meter.

    Parameters
    ----------
    revs : int
        Number of revolutions of the screw.
    distance : float
        Distance in meters travelled by the nut for the given number of 
        revolutions of the screw.
    """
    k = 1.0 / distance
    pitch = k * revs
    return pitch


def read_2Dtrajectory_csv(path: str) -> list[float]:
    """
    Reads a csv-file containing the (x,y) coordinates of points on a 
    2D-trajectory, and returns the points in a list.
    """
    points = []
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 2:
                try:
                    x = float(row[0])
                    y = float(row[1])
                    points.append((x, y))
                except ValueError:
                    continue
    return points


def plot_position_profiles(ppf: TMotionProfileDict) -> LineChart:
    """
    Plots the position profiles (x, y and z) in a `LineChart` object, and 
    returns this `LineChart` object. 
    """
    pchart = LineChart()
    pchart.add_xy_data(
        label="x",
        x1_values=ppf["x"]["time"],
        y1_values=ppf["x"]["values"]
    )
    pchart.add_xy_data(
        label="y",
        x1_values=ppf["y"]["time"],
        y1_values=ppf["y"]["values"]
    )
    pchart.add_xy_data(
        label="z",
        x1_values=ppf["z"]["time"],
        y1_values=ppf["z"]["values"]
    )
    pchart.x1.add_title("time, s")
    pchart.y1.add_title("ppfition, m")
    pchart.add_legend(columns=3)
    return pchart


def plot_velocity_profiles(vpf: TMotionProfileDict) -> LineChart:
    """
    Plots the velocity profiles (x, y and z) in a `LineChart` object, and 
    returns this `LineChart` object. 
    """
    vchart = LineChart()
    vchart.add_xy_data(
        label="x",
        x1_values=vpf["x"]["time"],
        y1_values=vpf["x"]["values"]
    )
    vchart.add_xy_data(
        label="y",
        x1_values=vpf["y"]["time"],
        y1_values=vpf["y"]["values"]
    )
    vchart.add_xy_data(
        label="z",
        x1_values=vpf["z"]["time"],
        y1_values=vpf["z"]["values"]
    )
    vchart.x1.add_title("time, s")
    vchart.y1.add_title("velocity, m/s")
    vchart.add_legend(columns=3)
    return vchart


def plot_acceleration_profiles(apf: TMotionProfileDict) -> LineChart:
    """
    Plots the acceleration profiles (x, y and z) in a `LineChart` object, and 
    returns this `LineChart` object. 
    """
    achart = LineChart()
    achart.add_xy_data(
        label="x",
        x1_values=apf["x"]["time"],
        y1_values=apf["x"]["values"]
    )
    achart.add_xy_data(
        label="y",
        x1_values=apf["y"]["time"],
        y1_values=apf["y"]["values"]
    )
    achart.add_xy_data(
        label="z",
        x1_values=apf["z"]["time"],
        y1_values=apf["z"]["values"]
    )
    achart.x1.add_title("time, s")
    achart.y1.add_title("acceleration, m/s²")
    achart.add_legend(columns=3)
    return achart


TSegment2DPath = TTrajectory2DPath = tuple[
    NDArray[np.float64], 
    NDArray[np.float64]
]

def get_2Dsegment_path(
    segment: Segment,
    num_points: int = 100
) -> tuple[TSegment2DPath, TPointPair]:
    """
    Returns the x- and y-coordinates of a number of linearly spaced points on
    the given 2D segment path.

    Returns
    -------
    path:
        Tuple with a Numpy array of the x-coordinates of the trajectory path and
        a Numpy array with the y-coordinates of the trajectory path.
    pair:
        Tuple with the start point and the end point of the segment.
    """
    thetax_fn = np.vectorize(segment.mp_x.get_position_time_fn())
    thetay_fn = np.vectorize(segment.mp_y.get_position_time_fn())
    t_arr = np.linspace(0.0, segment.mp_x.dt_tot, num_points)

    x_arr = thetax_fn(t_arr) / (360.0 * segment.xpitch)
    x0_arr = np.full_like(x_arr, x_arr[0])
    dx_arr = (x_arr - x0_arr) * segment.rdir_x.to_int()
    x_arr = x0_arr + dx_arr

    y_arr = thetay_fn(t_arr) / (360.0 * segment.ypitch)
    y0_arr = np.full_like(y_arr, y_arr[0])
    dy_arr = (y_arr - y0_arr) * segment.rdir_y.to_int()
    y_arr = y0_arr + dy_arr

    return (x_arr, y_arr), segment.point_pair


TSegment3DPath = TTrajectory3DPath = tuple[
    NDArray[np.float64], 
    NDArray[np.float64], 
    NDArray[np.float64]
]

def get_3Dsegment_path(
    segment: Segment,
    num_points: int = 100
) -> tuple[TSegment3DPath, TPointPair]:
    """
    Returns the x-, y-, and z-coordinates of a number of linearly spaced points 
    on the given 3D segment path.

    Returns
    -------
    path:
        Tuple with a Numpy array of the x-coordinates of the trajectory path,
        a Numpy array with the y-coordinates of the trajectory path, and a Numpy
        array with the z-coordinates of the trajectory path.
    pair:
        Tuple with the start point and the end point of the segment.
    """
    thetax_fn = np.vectorize(segment.mp_x.get_position_time_fn())
    thetay_fn = np.vectorize(segment.mp_y.get_position_time_fn())
    thetaz_fn = np.vectorize(segment.mp_z.get_position_time_fn())
    t_arr = np.linspace(0.0, segment.mp_x.dt_tot, num_points)

    x_arr = thetax_fn(t_arr) / (360.0 * segment.xpitch)
    x0_arr = np.full_like(x_arr, x_arr[0])
    dx_arr = (x_arr - x0_arr) * segment.rdir_x.to_int()
    x_arr = x0_arr + dx_arr

    y_arr = thetay_fn(t_arr) / (360.0 * segment.ypitch)
    y0_arr = np.full_like(y_arr, y_arr[0])
    dy_arr = (y_arr - y0_arr) * segment.rdir_y.to_int()
    y_arr = y0_arr + dy_arr

    z_arr = thetaz_fn(t_arr) / (360.0 * segment.zpitch)
    z0_arr = np.full_like(z_arr, z_arr[0])
    dz_arr = (z_arr - z0_arr) * segment.rdir_z.to_int()
    z_arr = z0_arr + dz_arr
    
    return (x_arr, y_arr, z_arr), segment.point_pair


def get_2Dtrajectory_path(
    trajectory2D: Trajectory,
    num_points: int = 100
) -> tuple[TTrajectory2DPath, TTrajectory2DPath]:
    """
    Returns x- and y-coordinates of the 2D trajectory path.

    Parameters
    ----------
    trajectory2D:
        The `Trajectory` object of which the path coordinates are wanted.
    num_points:
        The number of linearly spaced points per segment of the trajectory of
        which the x- and y-coordinates are determined.
    
    Returns
    -------
    path:
        Tuple of one Numpy array with the x-coordinates of linearly spaced
        points on the trajectory path and a second Numpy array with the
        y-coordinates of these points.
    points:
        Tuple of two Numpy arrays with the x- and the y-coordinates of the start
        points and end points of the segments in the trajectory.
    """
    x_arr_lst, y_arr_lst = [], []
    pt_lst = []
    for segment in trajectory2D:
        path, pair = get_2Dsegment_path(segment, num_points)
        x_arr_lst.append(path[0])
        y_arr_lst.append(path[1])
        pt_lst.append(pair)
    x_arr = np.concatenate(x_arr_lst)
    y_arr = np.concatenate(y_arr_lst)
    pts_i, pts_f = zip(*pt_lst)
    pts = list(pts_i)
    pts_f = list(pts_f)
    pts.append(pts_f[-1])
    pts_x, pts_y, _ = zip(*pts)
    pts_x, pts_y = np.array(pts_x), np.array(pts_y)
    return (x_arr, y_arr), (pts_x, pts_y)


def get_3Dtrajectory_path(
    trajectory3D: Trajectory,
    num_points: int = 100
) -> tuple[TTrajectory3DPath, TTrajectory3DPath]:
    """
    Returns x-, y-, and z-coordinates of the 3D trajectory path.

    Parameters
    ----------
    trajectory3D:
        The `Trajectory` object of which the path coordinates are wanted.
    num_points:
        The number of linearly spaced points per segment of the trajectory of
        which the x- and y-coordinates are determined.

    Returns
    -------
    path:
        Tuple of one Numpy array with the x-coordinates of linearly spaced
        points on the trajectory path, a second Numpy array with the
        y-coordinates of these points, and a third Numpy array with the
        z-coordinates of these points.
    points:
        Tuple of three Numpy arrays with the x-, y-, and the z-coordinates of 
        the start points and end points of the segments in the trajectory.
    """
    x_arr_lst, y_arr_lst, z_arr_lst = [], [], []
    pt_lst = []
    for segment in trajectory3D:
        path, pair = get_3Dsegment_path(segment, num_points)
        x_arr_lst.append(path[0])
        y_arr_lst.append(path[1])
        z_arr_lst.append(path[2])
        pt_lst.append(pair)
    x_arr = np.concatenate(x_arr_lst)
    y_arr = np.concatenate(y_arr_lst)
    z_arr = np.concatenate(z_arr_lst)
    pts_i, pts_f = zip(*pt_lst)
    pts = list(pts_i)
    pts_f = list(pts_f)
    pts.append(pts_f[-1])
    pts_x, pts_y, pts_z = zip(*pts)
    pts_x, pts_y, pts_z = np.array(pts_x), np.array(pts_y), np.array(pts_z)
    return (x_arr, y_arr, z_arr), (pts_x, pts_y, pts_z)


def compute_2Dsegment_path_deviation(segment, num_points: int = 100) -> float:
    """
    Returns the maximum deviation between the actual segment path and the
    ideal, rectilinear segment path.

    Parameters
    ----------
    segment:
        The `Segment` object under investigation.
    num_points:
        Number of points on the segment path where the deviation is calculated.
    """
    path, _ = get_2Dsegment_path(segment, num_points)
    x_arr, y_arr = path

    # Start- and end points of segment
    p0 = np.array([x_arr[0], y_arr[0]])
    p1 = np.array([x_arr[-1], y_arr[-1]])
    dp = p1 - p0
    dp_norm_sq = np.dot(dp, dp)

    # Calculate distance to the straight segment line
    max_dev = 0.0
    for x, y in zip(x_arr, y_arr):
        p = np.array([x, y])
        alpha = np.dot(p - p0, dp) / dp_norm_sq
        proj = p0 + alpha * dp
        deviation = np.linalg.norm(p - proj)
        max_dev = max(max_dev, deviation)

    return max_dev


def compute_3Dsegment_path_deviation(segment, num_points: int = 100) -> float:
    """
    Returns the maximum deviation between the actual segment path and the
    ideal, rectilinear segment path.

    Parameters
    ----------
    segment:
        The `Segment` object under investigation.
    num_points:
        Number of points on the segment path where the deviation is calculated.
    """
    path, _ = get_3Dsegment_path(segment, num_points)
    x_arr, y_arr, z_arr = path

    # Start- and end points of segment
    p0 = np.array([x_arr[0], y_arr[0], z_arr[0]])
    p1 = np.array([x_arr[-1], y_arr[-1], z_arr[-1]])
    dp = p1 - p0
    dp_norm_sq = np.dot(dp, dp)

    # Calculate distance to the straight segment line
    max_dev = 0.0
    for x, y, z in zip(x_arr, y_arr, z_arr):
        p = np.array([x, y, z])
        alpha = np.dot(p - p0, dp) / dp_norm_sq
        proj = p0 + alpha * dp
        deviation = np.linalg.norm(p - proj)
        max_dev = max(max_dev, deviation)
    return max_dev


def minimize_profile_time(
    profile_type: type[MotionProfile],
    ds_tot: float,
    a_m: float,
    v_m: float,
    v_i: float | None,
    v_f: float | None,
    s_i: float = 0.0
) -> MotionProfile:
    """
    Returns a motion profile with an optimal start/end velocity to make the
    required axis displacement in the shortest possible time.
    
    Parameters
    ----------
    profile_type: {TrapezoidalProfile, SCurvedProfile}
        Type of motion profile.
    ds_tot:
        Total travel distance of the movement (e.g. in degrees).
    a_m:
        Maximum acceleration of the motor (e.g. in degrees/s²).
    v_m:
        Maximum speed of the motor (e.g. in deg/s)
    v_i: 
        Start velocity of the movement (e.g. in deg/s) if known. 
    v_f: 
        End velocity of the movement (e.g. in deg/s) if known.
        Note that `v_i` and `v_f` cannot be both `None`. Either the start or
        end velocity must be specified.
    s_i: optional
        Initial position of the axis (e.g. in degrees).

    Returns
    -------
    MotionProfile
        The motion profile that fits between the boundary velocities `v_i` and
        `v_f` and travels the distance `ds_tot` in the shortest possible time
        without exceeding the maximum acceleration `a_m` or the maximum velocity
        `v_m`.
    """
    if v_i is None and v_f is None:
        raise ValueError("`v_i` and `v_f` cannot be both `None`.")
    axisdata = AxisData(
        profile_type=profile_type,
        dtheta=ds_tot,
        theta_i=s_i,
        omega_m=v_m,
        alpha_m=a_m,
        omega_i=v_i,
        omega_f=v_f,
        rdir=RotationDirection.COUNTERCLOCKWISE,
        pitch=0.0,
        d_tot=0.0
    )
    v_other = v_i if v_i is not None else v_f
    find = "v_i" if v_i is None else "v_f" 
    mp = TrajectoryPlanner._minimize_profile_time(axisdata, v_other, find)
    return mp
