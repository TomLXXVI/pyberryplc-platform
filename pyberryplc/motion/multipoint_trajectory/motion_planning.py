import warnings
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple, Literal, Optional, Dict
import math

from scipy.optimize import root_scalar

from pyberryplc.utils.math_utils import solve_quadratic_eq

from .fillet_path import FilletPath, Pose


EPS = 1e-12


@dataclass
class MotionConstraints:
    """
    Kinematic limits for time-parameterization along a geometric path.
    """
    v_max: float        # max. feed [units/s]
    a_t_max: float      # max. tangential acceleration [units/s^2]
    a_n_max: float      # max. lateral (centripetal) acceleration [units/s^2]
    j_max: float = 0.0  # max. jerk (rate of change of tangential acceleration) [units/s^3]


@dataclass
class MultipointMotionProfile:
    """
    Result of time-parameterization over an arc-length.
    """
    s: List[float]      # arc-length positions sampled from the path.
    t: List[float]      # time at each s[i] (0 at s[0])
    v: List[float]      # velocity at each s[i]
    a_t: List[float]    # tangential acceleration at each s[i]
    poses: List         # "Pose" object at each s[i]

    @property
    def x(self) -> List[float]:
        return [pose.x for pose in self.poses]

    @property
    def y(self) -> List[float]:
        return [pose.y for pose in self.poses]


class MotionPlanner(ABC):
    path: Optional[FilletPath] = None

    @classmethod
    @abstractmethod
    def parameterize_path(
        cls,
        path: FilletPath,
        cons: MotionConstraints,
        v_start: float = 0.0,
        v_end: float = 0.0,
    ) -> MultipointMotionProfile:
        """
        Computes the velocities and accelerations at given arc-length positions
        along a geometric path.

        Parameters
        ----------
        path:
            The geometric path.
        cons : MotionConstraints
            Kinematic limits imposed to the motion (v_max, a_t_max, a_n_max).
        v_start, v_end : float
            Boundary speeds at the start (s=0) and end (s=L) position of the
            path.

        Returns
        -------
        MultipointMotionProfile
        """
        ...

    @staticmethod
    def get_curvature_limited_speed_envelope(
        poses: List[Pose],
        v_max: float,
        a_n_max: float
    ) -> List[float]:
        """
        Returns the permissible speed at the given arc-length positions along
        the path, taking into account the path curvature in arc segments.

        Parameters
        ----------
        poses:
            List of "Pose" objects, containing the geometric data of the given
            positions on the path, including the curvature (kappa).
        v_max:
            The imposed maximum speed that may not be exceeded at any point
            along the path.
        a_n_max:
            The imposed maximum lateral acceleration that may not be exceeded
            at any bend in the path.
        """
        n = len(poses)
        v_lims = [v_max] * n
        if not math.isfinite(a_n_max) or a_n_max <= 0:
            return v_lims
        for i in range(n - 1):
            k_mid = 0.5 * (abs(poses[i].kappa) + abs(poses[i + 1].kappa))
            v_mid = v_max if k_mid <= 0.0 else min(v_max, math.sqrt(a_n_max / k_mid))
            v_lims[i] = min(v_lims[i], v_mid)
            v_lims[i + 1] = min(v_lims[i + 1], v_mid)
        return v_lims

    @staticmethod
    def get_speed_limit_envelope_fwd(
        s_vals: List[float],
        v_lims: List[float],
        v_start: float,
        a_t_max: float
    ) -> List[float]:
        """
        Returns the velocities at the given arc-length positions along the path,
        starting at the start point, where the velocity is known, and going
        forward towards the end point of the path, taking the maximum
        permissible tangential acceleration into account, but not exceeding
        the local velocity limits.
        """
        n = len(s_vals)
        v_vals = [0.0] * n
        v_vals[0] = min(v_lims[0], max(0.0, v_start))  # put known v_start first
        for i in range(1, n):
            ds = max(s_vals[i] - s_vals[i - 1], EPS)
            v_bound = math.sqrt(v_vals[i - 1] ** 2 + 2.0 * a_t_max * ds)
            v_vals[i] = min(v_lims[i], v_bound)
        return v_vals

    @staticmethod
    def get_speed_limit_envelope_bwd(
        s_vals: List[float],
        v_vals: List[float],
        v_end: float,
        a_t_max: float
    ) -> List[float]:
        """
        Returns the velocities at the given arc-length positions along the path,
        now starting at the end point, where the final velocity is also known,
        and going backward towards the start point of the path, taking the
        maximum permissible tangential acceleration into account.

        Note that parameter "v_vals" takes the v_vals which are returned from
        `_get_velocities_forward()`. Only if the backward calculated velocity
        at any position s_vals[i] is smaller than the forward calculated value,
        it will replace the forward value.
        """
        n = len(s_vals)
        v_vals[-1] = min(v_vals[-1], max(0.0, v_end))  # put known v_end last
        for i in range(n - 2, -1, -1):
            ds = max(s_vals[i + 1] - s_vals[i], EPS)
            v_bound = math.sqrt(v_vals[i + 1] ** 2 + 2.0 * a_t_max * ds)
            if v_vals[i] > v_bound:   # note: if v_start = v_vals[i=0] > v_bound, it will be replaced here by v_bound
                v_vals[i] = v_bound
        return v_vals

    @classmethod
    def get_speed_limit_envelope(
        cls,
        s_vals: List[float],
        poses: List[Pose],
        v_start: float,
        v_end: float,
        cons: MotionConstraints
    ) -> List[float]:
        """
        Returns the allowable velocities at the given arc-length positions along
        the path, taking into account the maximum allowable velocity v_max,
        the maximum allowable lateral acceleration a_n_max, and the maximum
        allowable tangential acceleration a_t_max.
        """
        # Calculate velocity limits at s_vals taking account of v_max and
        # a_n_max (in bends).
        v_lims = cls.get_curvature_limited_speed_envelope(poses, cons.v_max, cons.a_n_max)

        # Calculate intermediate velocities at s_vals, starting with v_start at
        # s_vals[0] and going forward, taking a_t_max into account, but never
        # exceeding the velocity constraints.
        v_vals = cls.get_speed_limit_envelope_fwd(s_vals, v_lims, v_start, cons.a_t_max)

        # Calculate intermediate velocities at s_vals, starting with v_end at
        # s_vals[-1] and going backward, taking a_t_max into account but never
        # exceeding the forward calculated velocity.
        v_vals = cls.get_speed_limit_envelope_bwd(s_vals, v_vals, v_end, cons.a_t_max)
        return v_vals


class TrapezoidalMotionPlanner(MotionPlanner):
    
    @classmethod
    def parameterize_path(
        cls,
        path: FilletPath,
        cons: MotionConstraints,
        v_start: float = 0.0,
        v_end: float = 0.0,
    ) -> MultipointMotionProfile:
        """
        Computes the velocities and accelerations at the given arc-length
        positions along a geometric path, assuming that acceleration can change
        instantaneously (jerk = 0).
        """
        cls.path = path
        s_vals = cls.path.sampler.points
        poses = cls.path.sampler.poses

        n = len(s_vals)
        if n == 0 or n != len(poses):
            raise ValueError(
                "s_vals and poses must be non-empty and of equal length."
            )
        v_vals = cls.get_speed_limit_envelope(
            s_vals, poses,
            v_start, v_end,
            cons
        )
        t_vals = cls.get_time_moments(
            s_vals, v_vals,
            cons.v_max
        )
        a_t_vals = cls.get_accelerations(v_vals, t_vals)
        return MultipointMotionProfile(s_vals, t_vals, v_vals, a_t_vals, poses)

    @classmethod
    def get_time_moments(
        cls,
        s_vals: List[float],
        v_vals: List[float],
        v_max: float
    ) -> List[float]:
        """
        Returns the times at which the given arc-length positions along
        the path are reached based on the allowable velocities at these
        positions, using the trapezoidal rule.
        """
        n = len(s_vals)
        t_vals = [0.0] * n
        v_floor = max(1e-3 * v_max, 1e-6)
        for i in range(1, n):
            ds = max(s_vals[i] - s_vals[i - 1], EPS)
            denom = max(v_vals[i - 1] + v_vals[i], v_floor)
            t_vals[i] = t_vals[i - 1] + 2.0 * ds / denom
        return t_vals

    @classmethod
    def get_accelerations(
        cls,
        v_vals: List[float],
        t_vals: List[float]
    ) -> List[float]:
        """
        Returns the tangential acceleration at the given arc-length positions
        along the path once the velocities and the times at which these
        positions are reached have been calculated.
        """
        n = len(v_vals)
        a_t_vals = [0.0] * n
        for i in range(1, n):
            dt = max(t_vals[i] - t_vals[i - 1], EPS)
            a_t_vals[i] = (v_vals[i] - v_vals[i - 1]) / dt
        a_t_vals[0] = a_t_vals[1] if n > 1 else 0.0
        return a_t_vals


SolveStatus = Literal["root", "hit_v_zero", "hit_a_limit"]


class CubicMotionSolver:
    """
    Contains functions for cubic motion (constant jerk) calculations over an
    elementary segment s0 -> s1 along a geometric path. Principal method of
    this class is `integrate()`.
    """
    a_max: float = 0.0
    j_max: float = 0.0
    _s_vals: List[float] = []
    _v_lims: List[float] = []
    _v_margin_rel: float = 0.01  # 1% hysteresis by default

    @classmethod
    def init(
        cls,
        a_max: float,
        j_max: float,
        s_vals: List[float],
        v_lims: List[float]
    ) -> None:
        """
        Initializes the `CubicMotionSolver`.

        Parameters
        ----------
        a_max:
            Maximum permissible acceleration (absolute value).
        j_max:
            Maximum jerk (absolute value).
        s_vals:
            List of arc-length positions sampled along the geometric path.
        v_lims:
            Speed limits at sampled positions along the geometric path.
        """
        cls.a_max = a_max
        cls.j_max = j_max
        cls._s_vals = s_vals
        cls._v_lims = v_lims

    @staticmethod
    def position(dt: float, s0: float, v0: float, a0: float, j0: float) -> float:
        """
        Solves for the position s1 at time dt from the initial position s0.

        Parameters
        ----------
        dt:
            Time passed since we were at position s0.
        s0:
            Initial position.
        v0:
            Velocity at position s0.
        a0:
            Acceleration at position s0.
        j0:
            Jerk, constant during dt.
        """
        s1 = s0 + v0 * dt + 1 / 2 * a0 * dt ** 2 + 1 / 6 * j0 * dt ** 3
        return s1

    @staticmethod
    def velocity(dt: float, v0: float, a0: float, j0: float) -> float:
        """
        Solves for the velocity at position s1 at a time dt from the initial
        position s0.

        Parameters
        ----------
        dt:
            Time passed since we were at position s0.
        v0:
            Velocity at position s0.
        a0:
            Acceleration at position s0.
        j0:
            Jerk, constant during dt.
        """
        v1 = v0 + a0 * dt + 1 / 2 * j0 * dt ** 2
        return v1

    @staticmethod
    def acceleration(dt: float, a0: float, j0: float) -> float:
        """
        Solves for the acceleration at position s1 at time dt from the initial
        position s0.

        Parameters
        ----------
        dt:
            Time passed since we were at position s0.
        a0:
            Acceleration at position s0.
        j0:
            Jerk, constant during dt.
        """
        a1 = a0 + j0 * dt
        return a1

    @staticmethod
    def A_eq(da: float, dt: float, j0: float) -> float:
        """
        Equation to solve for the change in acceleration over the time interval
        dt while jerk j0 remains constant. Returns the difference between the
        RHS and LHS of the equation. A solution is found when this difference
        is zero.

        Parameters
        ----------
        da:
            Targeted change in acceleration.
        dt:
            Time duration of this change.
        j0:
            Jerk, constant during dt.
        """
        A = j0 * dt - da
        return A

    @staticmethod
    def V_eq(dv: float, dt: float, a0: float, j0: float) -> float:
        """
        Equation to solve for the change in velocity over the time interval dt
        while jerk j0 remains constant. Returns the difference between the RHS
        and LHS of the equation. A solution is found when this difference is
        zero.

        Parameters
        ----------
        dv:
            Speed change.
        dt:
            Time duration of the speed change.
        a0:
            Acceleration at position s0.
        j0:
            Jerk, constant during dt.
        """
        V = 1 / 2 * j0 * dt ** 2 + a0 * dt - dv
        return V

    @staticmethod
    def S_eq(ds: float, dt: float, v0: float, a0: float, j0: float) -> float:
        """
        Equation to solve for the displacement over the time interval dt while
        jerk j0 remains constant. Returns the difference between the RHS
        and LHS of the equation. A solution is found when this difference is
        zero.

        Parameters
        ----------
        ds:
            Displacement.
        dt:
            Time duration of the displacement.
        v0:
            Velocity at initial position s0.
        a0:
            Acceleration at position s0.
        j0:
            Jerk, constant during dt.
        """
        S = 1 / 6 * j0 * dt ** 3 + 1 / 2 * a0 * dt ** 2 + v0 * dt - ds
        return S

    @classmethod
    def _solve_S_eq(
        cls,
        ds: float,
        v0: float,
        a0: float,
        j0: float, *,
        v_floor: float = 1e-9,
        dt_floor: float = 1e-9,
    ) -> Tuple[float, SolveStatus]:
        """
        Tries to solve the displacement equation
            1 / 6 * j0 * dt ** 3 + 1 / 2 * a0 * dt ** 2 + v0 * dt - ds = 0
        for dt, i.e. it returns the time that is needed for the displacement
        given the initial state of motion. Jerk j0 is considered constant during
        the displacement.
        """
        # Special case: jerk (almost) equal to zero -> S-equation either
        # quadratic or linear.
        if abs(j0) < 1e-12:
            dt, solve_status = cls._solve_zero_jerk_S_eq(
                ds, v0, a0, j0,
                v_floor, dt_floor
            )
            return dt, solve_status

        # General case: jerk different from zero.
        # Compute physical time caps: dt_v0 (time to v1 = 0) and dt_a (time to
        # go from a0 to either +a_max or -a_max)
        dt_v0 = cls._solve_dt_to_zero_v(v0, a0, j0)
        dt_a = cls._solve_dt_to_accel_limit(a0, j0)
        caps = [dt for dt in (dt_v0, dt_a) if dt is not None and dt > 0.0]
        dt_cap = min(caps) if caps else float("inf")

        # Initial guess for an upper bracket
        # Start from "no-jerk" estimate; ensure positive and not exceeding
        # dt_cap
        dt = min(
            max(ds / max(v0, v_floor), dt_floor),
            dt_cap if math.isfinite(dt_cap) else 1e6
        )

        # Try to find a sign change (zero crossing) before the time cap by
        # increasing time step dt
        # S0 = cls.S_eq(s1, dt=0.0)  # always equal to -ds (< 0)
        S1 = cls.S_eq(ds, dt, v0, a0, j0)
        expand = 0
        while S1 < 0.0 and expand < 32:
            # If we hit the time cap, quit expanding.
            if math.isfinite(dt_cap) and abs(dt - dt_cap) <= 1e-15:
                break
            # Increase dt, but never beyond the time cap.
            dt *= 1.5
            # Clamp to the time cap if dt exceeds it.
            if math.isfinite(dt_cap) and dt > dt_cap:
                dt = dt_cap
            S1 = cls.S_eq(ds, dt, v0, a0, j0)
            expand += 1

        # If S1 still negative at the time cap -> no root of S_fn before the
        # time cap.
        if S1 < 0.0:
            if math.isfinite(dt_cap):
                # Identify which cap we hit first
                if dt_v0 is not None and abs(dt_cap - dt_v0) <= 1e-12:
                    return dt_cap, "hit_v_zero"
                if dt_a is not None and abs(dt_cap - dt_a) <= 1e-12:
                    return dt_cap, "hit_a_limit"
            # No finite time cap -> treat as velocity cap
            return dt, "hit_v_zero"

        # If S1 not negative: we have a bracket
        sol = root_scalar(
            lambda dt_: cls.S_eq(ds, dt_, v0, a0, j0),
            bracket=(0.0, dt)
        )
        return sol.root, "root"

    @classmethod
    def _solve_zero_jerk_S_eq(
        cls,
        ds: float,
        v0: float,
        a0: float,
        j0: float,
        v_floor: float,
        dt_floor: float
    ) -> Tuple[float, SolveStatus]:
        """
        Solves 1 / 2 * a0 * dt ** 2 + v0 * dt - ds = 0
        """
        if abs(a0) < 1e-12:
            dt, solve_status = cls._solve_lin_zero_jerk_S_eq(ds, v0, v_floor, dt_floor)
        else:
            dt, solve_status = cls._solve_quad_zero_jerk_S_eq(ds, v0, a0, j0, dt_floor)
        return dt, solve_status

    @staticmethod
    def _solve_lin_zero_jerk_S_eq(
        ds: float,
        v0: float,
        v_floor: float,
        dt_floor: float
    ) -> Tuple[float, SolveStatus]:
        """
        Solves v0 * dt - ds = 0
        """
        if v0 <= v_floor:
            return dt_floor, "hit_v_zero"
        dt = ds / v0
        return dt, "root"

    @classmethod
    def _solve_quad_zero_jerk_S_eq(
        cls,
        ds: float,
        v0: float,
        a0: float,
        j0: float,
        dt_floor: float
    ) -> Tuple[float, SolveStatus]:
        """
        Solves 1 / 2 * a0 * dt ** 2 + v0 * dt - ds = 0
        """
        try:
            r1, r2 = solve_quadratic_eq(a=1 / 2 * a0, b=v0, c=-ds)
            roots = [r for r in (r1, r2) if r > 0.0 and math.isfinite(r)]
            if roots:
                return min(roots), "root"
            else:
                dt = cls._solve_dt_to_zero_v(v0, a0, j0)
                return max(dt, dt_floor), "hit_v_zero"
        except ValueError:
            dt = cls._solve_dt_to_zero_v(v0, a0, j0)
            return max(dt, dt_floor), "hit_v_zero"

    @classmethod
    def _solve_dt_to_zero_v(
        cls,
        v0: float,
        a0: float,
        j0: float,
    ) -> Optional[float]:
        """
        Solves 1 / 2 * j0 * dt ** 2 + a0 * dt - (0.0 - v0) = 0
        """
        if abs(j0) < 1e-12:
            if a0 < 0.0 < v0:
                # self.v0 > 0 and velocity decreasing -> crossing dt-axis
                dt = cls._solve_lin_dt_to_zero_v(v0, a0)
                return dt
            else:
                # velocity won't cross dt-axis
                return None
        dt = cls._solve_quad_dt_to_zero_v(v0, a0, j0)
        return dt

    @staticmethod
    def _solve_lin_dt_to_zero_v(
        v0: float,
        a0: float, *,
        a_floor: float = 1e-9
    ) -> Optional[float]:
        """
        Solves a0 * dt - (0.0 - v0) = 0
        """
        if abs(a0) < a_floor:
            return None
        dv = -v0
        dt = dv / a0
        return dt

    @staticmethod
    def _solve_quad_dt_to_zero_v(
        v0: float,
        a0: float,
        j0: float,
    ) -> Optional[float]:
        """
        Solves 1 / 2 * j0 * dt ** 2 + a0 * dt - (0.0 - v0) = 0
        """
        dv = -v0
        try:
            r1, r2 = solve_quadratic_eq(a=1 / 2 * j0, b=a0, c=-dv)
            roots = [r for r in (r1, r2) if r > 0 and math.isfinite(r)]
            if roots:
                return min(roots)
            else:
                return None
        except ValueError:
            return None

    @classmethod
    def _solve_dt_to_accel_limit(
        cls,
        a0: float,
        j0: float
    ) -> Optional[float]:
        """
        Solve a0 + j * dt = +a_max and a0 + j * dt = -a_max; pick the smallest
        positive.
        """
        if abs(j0) < 1e-12:
            return None
        dt1 = (cls.a_max - a0) / j0
        dt2 = (-cls.a_max - a0) / j0
        cands = [dt for dt in (dt1, dt2) if dt > 0]
        return min(cands) if cands else None


    @classmethod
    def _v_limit_at(cls, s: float) -> float:
        """
        Piecewise-linear interpolation of v_lim(s).
        """
        if not cls._s_vals:
            return float("inf")
        # binary search
        lo, hi = 0, len(cls._s_vals) - 1
        if s <= cls._s_vals[0]:
            return cls._v_lims[0]
        if s >= cls._s_vals[hi]:
            return cls._v_lims[hi]
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if cls._s_vals[mid] <= s:
                lo = mid
            else:
                hi = mid
        s0, s1 = cls._s_vals[lo], cls._s_vals[hi]
        v0, v1 = cls._v_lims[lo], cls._v_lims[hi]
        w = 0.0 if s1 == s0 else (s - s0) / (s1 - s0)
        return (1.0 - w) * v0 + w * v1

    @classmethod
    def _jerk_policy(cls, s: float, v: float, a: float) -> float:
        """
        Controls jerk in the current moving step with regard to acceleration
        and velocity limits. Can only return one of {-j_max, 0.0, +j_max}.

        Rules (in order):
        1)  Enforce |a| <= a_max hard.
        2)  Compare v to the local limit v_lim(s) with a small hysteresis
            margin.
            -   v > v_lim + dv -> brake  (j = -j_max)
            -   v < v_lim - dv -> accel  (j = +j_max)
            -   else           -> coast  (j = 0)

        Parameters
        ----------
        s:
            Start position of the current moving step.
        v:
            Initial velocity at the start position s.
        a:
            Initial acceleration at the start position s.
        """
        jmax = cls.j_max
        # Hard accel limits first
        if a > cls.a_max:
            return -jmax
        if a < -cls.a_max:
            return +jmax

        # Velocity limit-aware control with hysteresis
        v_lim = cls._v_limit_at(s)
        if not math.isfinite(v_lim):
            return 0.0
        dv = max(cls._v_margin_rel * max(v_lim, 1.0), 1e-6)
        if v > v_lim + dv:
            return -jmax
        if v < v_lim - dv:
            return +jmax
        return 0.0

    @classmethod
    def integrate(
        cls,
        s1: float,
        s0: float,
        v0: float,
        a0: float,
        t0: float, *,
        j_imposed: Optional[float] = None,
        max_substeps: int = 16,
        s_eps: float = 1e-12
    ) -> Tuple[float, float, float, float]:
        """
        Solves the equations for moving from an arc-length position s0 to the
        next arc-length position s1 ahead of s0 along a geometric path.

        The method uses piecewise integration to solve the displacement
        equation S_eq() for dt first. Knowing the initial position s0 and final
        position s1, and the initial state of motion at s0 (v0, a0, j0), dt is
        numerically solved from the cubic equation S_eq().

        Parameters
        ----------
        s1:
            End position of the arc-length segment s1 - s0.
        s0:
            Start position of the arc-length segment s1 - s0.
        v0:
            Velocity at the start position s0.
        a0:
            Acceleration at the start position s0.
        t0:
            Time at the segment's start position s0 (measured from the time
            moment the motion started at the start point of the geometric path).
        j_imposed:
            Jerk to be applied when moving from s0 to s1. If None, the jerk is
            determined internally based on the current state of the motion.
        max_substeps:
            The maximum number of integration steps to solve the motion from
            s0 to s1. Default is 16.
        s_eps:
            Allows a small deviation when determining if the final position s1
            has been reached. Default is 1e-12.

        Returns
        -------
        t1:
            Time moment the end position s1 of the segment is reached.
        v1:
            Velocity reached at the end position s1 of the segment.
        a1:
            Acceleration reached at the end position s1 of the segment.
        j:
            Jerk.
        """
        s = s0
        v = v0
        a = a0
        t = t0
        j = float("nan")

        for _ in range(max_substeps):
            if s1 - s <= s_eps:
                break

            if j_imposed is None:
                j = cls._jerk_policy(s, v, a)
            else:
                j = j_imposed

            # Solve for the time step needed to reach s1: either we reach s1
            # (solve_status == 'root'), or we hit a time cap before s1. If a
            # time cap is hit, it means the cubic equation could not be solved
            # to go directly from s to s1 with the current jerk j. In that case,
            # the state of motion is solved for the position reached at the time
            # cap and jerk can be re-evaluated in a next iteration of the
            # for-loop based on the solved state of motion at the time cap.
            dt, solve_status = cls._solve_S_eq(s1 - s, v, a, j)

            s = cls.position(dt, s, v, a, j)
            v = cls.velocity(dt, v, a, j)
            a = cls.acceleration(dt, a, j)
            t += dt

            if solve_status == "root":
                # We reached s1 inside this substep with the correct jerk sign.
                break
            # else: we hit a time cap -> continue loop: jerk policy can flip
            # sign for the next sub-step

        else:
            warnings.warn(
                "`CubicMotionSolver.integrate()` reached the maximum number of "
                "sub-steps.", category=UserWarning
            )

        return t, max(v, 0.0), a, j

    @staticmethod
    def scurved_acceleration(
        v1: float,
        v0: float,
        a0: float,
        j_max: float,
        a_t_max: float
    ) -> Dict[str, float]:
        """
        Solves for the acceleration distance and time when speed is raised from
        an initial velocity v0 to a final velocity v1 using a two (triangular)
        or three staged (trapezoidal) acceleration profile.

        Parameters
        ----------
        v1:
            Requested final velocity after acceleration.
        v0:
            Initial velocity at the moment acceleration begins.
        a0:
            Initial acceleration at the moment acceleration begins.
        j_max:
            Maximum jerk (absolute value).
        a_t_max:
            Maximum (tangential) acceleration that can be applied (absolute
            value).

        Returns
        -------
        Dict[str, float] with keys:
            "ds_tot":
                Total acceleration distance.
            "ds1":
                Acceleration distance during first phase where j0 = +j_max.
            "ds2":
                Acceleration distance during second phase where j0 = 0 and
                a = +a_t_max. May be missing if second phase is not needed.
            "ds3":
                Acceleration distance during final phase where j0 = -j_max. At
                the end of this phase acceleration is brought back to zero
                while the demanded end velocity is reached.
            "dt_tot":
                Total acceleration time.
            "dt1":
                Time of first acceleration phase where j0 = +j_max.
            "dt2":
                Time of second acceleration phase where j0 = 0 and a = +a_t_max.
                May be missing if second phase is not needed.
            "dt3":
                Time of final acceleration phase where j0 = -j_max. At the end
                of this phase acceleration is brought back to zero while the
                demanded end velocity is reached.
        """
        # Required speed change.
        dv = v1 - v0

        # Keep a0 inside the admissible band for planning; otherwise dv1_max can
        # become negative and break the case logic.
        a0 = max(min(a0, a_t_max), -a_t_max)

        # Maximum speed change that is possible during 1st and 3rd acceleration
        # phase.
        dv1_max = 1 / (2 * j_max) * (a_t_max ** 2 - a0 ** 2)
        dv3_max = 1 / (2 * j_max) * a_t_max ** 2

        if abs(dv1_max + dv3_max) < abs(dv):
            # An intermediate acceleration phase will be needed to achieve the
            # requested final velocity where acceleration is limited to a_t_max.

            # Time and distance in the 1st phase.
            dt1 = (a_t_max - a0) / j_max
            ds1 = v0 * dt1 + 1 / 2 * a0 * dt1 ** 2 + 1 / 6 * j_max * dt1 ** 3

            # Velocity change, time and distance in the 2nd phase.
            dv2 = dv - (dv1_max + dv3_max)
            dt2 = dv2 / a_t_max
            v_i2 = v0 + dv1_max
            ds2 = v_i2 * dt2 + 1 / 2 * a_t_max * dt2 ** 2

            # Time and distance in the 3rd phase.
            v_i3 = v_i2 + dv2
            dt3 = (0 - a_t_max) / -j_max
            ds3 = v_i3 * dt3 + 1 / 2 * a_t_max * dt3 ** 2 + 1 / 6 * -j_max * dt3 ** 3

            # Just a check to see if the calculations are right...
            dv_ = dv1_max + dv2 + dv3_max
            if abs(dv_ - dv) > 1e-3:
                warnings.warn(
                    "Calculated speed change differs from required.",
                    category=UserWarning
                )

            return {
                "ds_tot": ds1 + ds2 + ds3,
                "ds1": ds1,
                "ds2": ds2,
                "ds3": ds3,
                "dt_tot": dt1 + dt2 + dt3,
                "dt1": dt1,
                "dt2": dt2,
                "dt3": dt3
            }

        else:
            # We don't need the intermediate phase to reach the requested
            # final velocity.

            # initial phase
            a1 = math.sqrt(a0 ** 2 / 2 + j_max * dv)
            dt1 = (a1 - a0) / j_max
            ds1 = v0 * dt1 + 1 / 2 * a0 * dt1 ** 2 + 1 / 6 * j_max * dt1 ** 3
            dv1 = 1 / (2 * j_max) * (a1 ** 2 - a0 ** 2)

            # final phase
            v_i3 = v0 + dv1
            dt3 = (0.0 - a1) / -j_max
            ds3 = v_i3 * dt3 + 1 / 2 * a1 * dt3 ** 2 + 1 / 6 * -j_max * dt3 ** 3
            dv3 = 1 / (2 * j_max) * a1 ** 2

            # Just a check to see if the calculations are right...
            dv_ = dv1 + dv3
            if abs(dv_ - dv) > 1e-3:
                warnings.warn(
                    "Calculated speed change differs from required.",
                    category=UserWarning
                )

            return {
                "ds_tot": ds1 + ds3,
                "ds1": ds1,
                "ds3": ds3,
                "dt_tot": dt1 + dt3,
                "dt1": dt1,
                "dt3": dt3
            }

    @staticmethod
    def scurved_deceleration(
        v1: float,
        v0: float,
        a0: float,
        j_max: float,
        a_t_max: float
    ) -> dict:
        """
        Jerk-limited deceleration from (v0,a0) to (v1<=v0, a=0).
        If |a0| > a_t_max, we first 'bleed' acceleration back to the nearest
        admissible boundary with a single jerk pulse (pre-phase), then run the
        standard three-phase S-deceleration.
        """
        out = {}
        J, A = j_max, a_t_max

        # Pre-phase: bring a0 back inside [-A, +A] with minimal distance.
        v, a = v0, a0
        dt_tot = 0.0
        ds_tot = 0.0

        eps = 1e-12
        if a > A + eps:
            # reduce a with j = -J to +A
            dt_pre = (a - A) / J
            j_pre = -J
            ds_pre = v * dt_pre + 0.5 * a * dt_pre ** 2 + (j_pre * dt_pre ** 3) / 6.0
            v += a * dt_pre + 0.5 * j_pre * dt_pre ** 2
            a = A
            out["dt_pre"], out["ds_pre"], out["j_pre"] = dt_pre, abs(ds_pre), j_pre
            dt_tot += dt_pre
            ds_tot += abs(ds_pre)
        elif a < -A - eps:
            # increase a with j = +J to -A
            dt_pre = (-A - a) / J
            j_pre = +J
            ds_pre = v * dt_pre + 0.5 * a * dt_pre ** 2 + (j_pre * dt_pre ** 3) / 6.0
            v += a * dt_pre + 0.5 * j_pre * dt_pre ** 2
            a = -A
            out["dt_pre"], out["ds_pre"], out["j_pre"] = dt_pre, abs(ds_pre), j_pre
            dt_tot += dt_pre
            ds_tot += abs(ds_pre)

        # Main decel from (v,a) to v1 with terminal a=0.
        acc = CubicMotionSolver.scurved_acceleration(
            v1=-v1, v0=-v, a0=-a, j_max=J, a_t_max=A
        )

        # Map times & distances; distances are positive lengths
        dt1 = abs(acc.get("dt1", 0.0))
        dt2 = abs(acc.get("dt2", 0.0))
        dt3 = abs(acc.get("dt3", 0.0))

        ds1 = abs(acc.get("ds1", 0.0))
        ds2 = abs(acc.get("ds2", 0.0))
        ds3 = abs(acc.get("ds3", 0.0))

        out["dt1"], out["dt3"] = dt1, dt3
        out["ds1"], out["ds3"] = ds1, ds3
        if dt2 > 0.0:
            out["dt2"], out["ds2"] = dt2, ds2

        out["dt_tot"] = dt_tot + dt1 + dt2 + dt3
        out["ds_tot"] = ds_tot + ds1 + ds2 + ds3
        return out

    @staticmethod
    def accel_distance(
        v0: float,
        a0: float,
        v_target: float,
        a_max: float,
        j_max: float
    ) -> tuple[float, float]:
        """
        Calculates the minimal distance needed to accelerate from (v0, a0) to
        v_target.

        Two cases:
        -   Triangular jerk: we never hit a_max; single phase with j = +j_max.
        -   Trapezoidal: ramp a from a0 up to +a_max with j = +j_max, then hold
            a = +a_max.

        Returns
        -------
        ds_target:
            Required distance to accelerate from v0 to v_target.
        a_target:
            Acceleration obtained at acceleration distance ds.
        """
        A = a_max
        J = j_max

        if v_target <= v0:
            return 0.0, a0

        dv = v_target - v0

        # Time to reach A (could be 0 if a0 ≈ A)
        dt_ramp = max(0.0, (A - a0) / J)

        # dv gained while ramping a from a0 to A with j = +J
        dv_ramp = a0 * dt_ramp + 0.5 * J * dt_ramp ** 2

        if dv <= dv_ramp + 1e-12:
            # Triangular case: j = +J for a time dt; never reach A.
            # dv = a0 * dt + 0.5 * J * dt^2  -> 0.5 * J * dt^2 + a0 * dt - dv = 0
            disc = a0 ** 2 + 2.0 * J * dv
            dt = (-a0 + (disc if disc > 0.0 else 0.0) ** 0.5) / J
            # ds = v0 * dt + 0.5 * a0  * dt^2 + (1/6) * J * dt^3
            ds = v0 * dt + 0.5 * a0 * dt ** 2 + (J * dt ** 3) / 6.0
            a = a0 + J * dt
            return ds, a

        # Trapezoidal case: ramp to A, then constant A distance during
        # ramp.
        ds_ramp = v0 * dt_ramp + 0.5 * a0 * dt_ramp ** 2 + (J * dt_ramp ** 3) / 6.0
        v_after = v0 + dv_ramp
        dv_left = dv - dv_ramp
        dt_flat = dv_left / A  # constant-acc phase with a = +A
        ds_flat = v_after * dt_flat + 0.5 * A * dt_flat ** 2
        return ds_ramp + ds_flat, A


@dataclass
class SpeedPlan:
    """
    Committed S‑curve acceleration/deceleration plan toward a target (s_k, v_k).

    kind:
        Either 'accel' or 'decel'.
    s_star:
        Arc‑length where the deceleration must start so that we land on
        (s_k, v_k) with a=0 (triangular or trapezoidal depending on limits).
    s_p1_end:
        End of phase 1 (j = −j_max). If the plan is triangular this equals
        s_goal and s_p2_end is None.
    s_p2_end:
        End of phase 2 (plateau, a = −a_max). None for triangular plans.
    s_goal:
        The target arc‑length (usually the location of the next drop in the
        eroded speed limit). When s >= s_goal the plan is finished and can be
        cleared.
    v_goal:
        Target speed at s_goal (usually the eroded speed limit value there).
    """
    kind: str
    s_star: float
    s_p1_end: float
    s_p2_end: Optional[float]
    s_goal: float
    v_goal: float


class SCurvedMotionPlanner(MotionPlanner):
    """
    S-curved multipoint trajectory planner (jerk-bounded).

    High-level idea
    ---------------
    First, a speed limit envelope is calculated along the path. Then, motion is
    driven by S-curved acceleration/deceleration plans towards the next speed
    limit change (either a drop or rise of the speed limit curve) segment per
    segment by iterating over the sampled points that were taken along the path.

    1)  Speed limit envelope construction
    -    v_lims_ini :
            Raw speed limit from path geometry (v_max, curvature / normal
            acceleration limits, trapezoidal tangential limits).
    -    v_lims_fwd:
            Forward feasibility pass, jerk-aware, propagates a.
            From (v[i-1], a[i-1]) we find the largest reachable v[i] within ds,
            without forcing a[i] back to 0 (no flare-out towards v[i]).
    -    v_lims_bwd:
            Backward feasibility pass, jerk-aware, brakes to local minima and
            to the terminal stop, ending with a = 0 at those goals.
    -    Final envelope:
            v_lims = min(v_lims_ini, v_lims_fwd, v_lims_bwd).

    2)  Per-segment loop [s0 -> s1]
    -   _look_ahead() :
            Scans v_lims to find the next speed limit change ahead: either a
            drop (limit decreases) or a rise (limit increases).
    -   _get_decel_plan() :
            If it is a drop, build a deceleration plan to land exactly on
            (s_k, v_k) with a = 0.
    -   _get_accel_plan(...) :
            If it is a rise, build an acceleration plan to reach (s_k, v_k)
            without forcing a to 0 (free terminal acceleration).
    -   Commit the plan if its start point s* lies inside the current segment
        [s0, s1] or immediately if s* is already < s0.

    3)  Executing a committed plan  (_solve_with_speed_plan)
    -   Before s*: default jerk policy.
    -   Phase P1: jerk pulse (±j_max).
    -   Phase P2: plateau with j = 0 and a = ±a_max (if present).
    -   Phase P3: only for decel plans; use +j_max to flare a back to 0 so we
        match the lower plateau exactly at the local speed limit.
    -   At the next segment [s0, s1], the current plan is cleared.

    4)  No active plan
    -   Fall back to the limit-aware jerk policy with hysteresis:
        brake if v > v_lims(s) + δv, accelerate if v < v_lims(s) − δv,
        else coast (j = 0).

    5)  Terminal braking
        The “tail-cut” braking routine handles the final stopping segment with
        closed-form S-deceleration so that v(L) = 0 and a(L) = 0 exactly at the
        end of the path.
    """
    s_vals: List[float] = []
    v_vals: List[float] = []
    a_vals: List[float] = []
    j_vals: List[float] = []
    t_vals: List[float] = []
    a_max: float = 0.0
    j_max: float = 0.0
    v_lims_ini: List[float] = []
    v_lims_fwd: List[float] = []
    v_lims_bwd: List[float] = []
    v_lims: List[float] = []
    poses: List[Pose] = []

    @classmethod
    def parameterize_path(
        cls,
        path: FilletPath,
        cons: MotionConstraints,
        v_start: float = 0.0,
        v_end: float = 0.0,
        a_start: float = 0.0
    ) -> MultipointMotionProfile:
        """
        Computes the velocities and accelerations at the sampled arc-length
        positions along a geometric path, assuming that jerk (i.e. the first
        derivative of acceleration with respect to time) can change
        instantaneously between path segments.
        """
        # Initialization of the planner.
        cls._init(
            path=path,
            v_start=v_start,
            v_end=v_end,
            a_start=a_start,
            cons=cons,
        )

        # Solve the equations of motion segment per segment by iterating over
        # the sampled points that were taken along the path. After the final
        # brake to the end point of the path, the for-loop is quit.
        for cur_index in range(1, len(cls.s_vals)):
            speed_plan = cls._look_ahead(cur_index)
            finished = cls._solve_motion(cur_index, speed_plan)
            if finished:
                break

        # Return results
        return MultipointMotionProfile(
            cls.s_vals,  # sampled arc-length positions
            cls.t_vals,  # time moments the positions are reached starting from t0 = 0.
            cls.v_vals,  # speed values at the sampled arc-length positions
            cls.a_vals,  # acceleration values at the sampled arc-length positions.
            cls.poses
        )

    @classmethod
    def _init(
        cls,
        path: FilletPath,
        v_start: float,
        v_end: float,
        a_start: float,
        cons: MotionConstraints
    ) -> None:
        """
        Initializes the `SCurvedMotionPlanner` and calculates the speed limit
        envelope.

        Parameters
        ----------
        path:
            The geometric path that is to be parameterized.
        v_start:
            Initial velocity at the start point of the path.
        v_end:
            Final velocity at the end point of the path.
        a_start:
            Initial acceleration in the start point of the path.
        cons:
            See class MotionConstraints.
        """
        cls.path = path
        s_vals = cls.path.sampler.points
        poses = cls.path.sampler.poses

        n = len(s_vals)
        if n == 0 or n != len(poses):
            raise ValueError(
                "s_vals and poses must be non-empty and of equal length."
            )

        cls.s_vals = s_vals
        cls.poses = poses
        cls.v_vals = [v_start]
        cls.a_vals = [a_start]
        cls.j_vals = [0.0]
        cls.t_vals = [0.0]
        cls.current_plan = None
        cls.a_max = cons.a_t_max
        cls.j_max = cons.j_max
        cls.get_speed_limit_envelope(s_vals, poses, v_start, v_end, cons)
        # --> velocity limits at the sampled points along the path
        CubicMotionSolver.init(
            a_max=cons.a_t_max,
            j_max=cons.j_max,
            s_vals=s_vals,
            v_lims=cls.v_lims
        )

    @classmethod
    def get_speed_limit_envelope(
        cls,
        s_vals: List[float],
        poses: List[Pose],
        v_start: float,
        v_end: float,
        cons: MotionConstraints
    ) -> List[float]:
        """
        Builds the jerk-aware speed limit envelope.
        """
        # raw envelope from curvature + trapezoidal accel limits
        cls.v_lims_ini = super().get_speed_limit_envelope(
            s_vals, poses, v_start, v_end, cons
        )

        # forward jerk-aware feasibility (can we reach the speed limit?)
        cls.v_lims_fwd = cls._erode_speed_limit_envelope_fwd(
            s_vals, cls.v_lims_ini, cons, v_start
        )

        # backward jerk-aware feasibility (can we brake down to local minima in
        # the speed limit curve?)
        cls.v_lims_bwd = cls._erode_speed_limit_envelope_bwd(
            s_vals, cls.v_lims_ini, cons, v_end
        )

        # final: pointwise min of raw, forward, and backward
        cls.v_lims = [
            min(cls.v_lims_ini[i], cls.v_lims_fwd[i], cls.v_lims_bwd[i])
            for i in range(len(s_vals))
        ]
        return cls.v_lims

    @classmethod
    def _solve_motion(cls, cur_index: int, plan: Optional[SpeedPlan]) -> int:
        """
        Solves the equations of motion to go from arc-length position s0 to
        position s1 along the geometric path. Calculates the time t1, momentary
        velocity v1 and momentary acceleration a1 at the next point s1 ahead.
        """
        s1 = cls.s_vals[cur_index]
        s0 = cls.s_vals[cur_index - 1]
        v0 = cls.v_vals[cur_index - 1]
        a0 = cls.a_vals[cur_index - 1]
        t0 = cls.t_vals[cur_index - 1]

        # First, check the required braking distance to the end of the path.
        bd = CubicMotionSolver.scurved_deceleration(
            cls.v_lims[-1],
            v0, a0,
            cls.j_max, cls.a_max
        )
        ds_end_brake = bd["ds_tot"]
        s_end = cls.s_vals[-1]
        ds_end1 = s_end - s1

        # Start final braking process to come to a stop at the end of the path.
        # From here, we go directly to the end of the path using S_curved
        # deceleration.
        if ds_end_brake >= ds_end1:
            d = cls._solve_final_brake(ds_end_brake)
            for key in d:
                cls._print_to_screen(cur_index, f"FINAL BRAKE-{key.upper()}")
                cur_index += 1
            return 1

        # The look-ahead algorithm did not return a local speed plan: no need
        # to accelerate/decelerate in the current segment with regard to a
        # rise/drop of the speed limit curve, move to next position s1 with
        # default jerk policy.
        if plan is None:
            t1, v1, a1, j1 = CubicMotionSolver.integrate(
                s1, s0, v0, a0, t0,
                j_imposed=None
            )
            cls.t_vals.append(t1)
            cls.v_vals.append(v1)
            cls.a_vals.append(a1)
            cls.j_vals.append(j1)

            cls._print_to_screen(cur_index, "FLOATING")
            return 0

        # The look-ahead algorithm did return a local speed plan for
        # rising/lowering the speed in the current segment s0 -> s1.
        return cls._solve_with_speed_plan(cur_index, plan)

    @classmethod
    def _look_ahead(cls, cur_index: int) -> Optional[SpeedPlan]:
        """
        Plans the acceleration/deceleration to the nearest rise/drop in the
        speed limit envelope. Only returns the speed plan if the start position
        of acceleration/deceleration is smaller than the current position s1
        for which the state of motion is determined.
        """
        s1 = cls.s_vals[cur_index]
        s0 = cls.s_vals[cur_index - 1]

        # Find the next rise or drop in the speed limit envelope ahead of
        # the current index (note: s0 is at index cur_index-1).
        nxt = cls._find_next_change_in_speed_limit(i0 = max(cur_index - 1, 0))
        if nxt is None:
            return None

        kind, k, s_k, v_k_lim = nxt

        if kind == "drop":
            plan = cls._get_decel_plan(cur_index, s_k, v_k_lim)
        else:
            # Plan local acceleration from s0 to s_k.
            plan = cls._get_accel_plan(cur_index, s_k, v_k_lim)

        if plan is None:
            return None

        # Only return plan if acceleration/deceleration start point s* < s1.
        if plan.s_star <= s1:
            plan.s_star = max(plan.s_star, s0)  # never go backward in s
            return plan

        return None

    @classmethod
    def _solve_final_brake(
        cls,
        ds_end_brake: float
    ) -> Dict[str, Tuple[float, float, float, float, float]]:
        """
        Executes the final S-deceleration so we land exactly at the path end
        with v=0 and a=0. If the required start s* lies before the current
        [s0, s1] segment, we rewind to the correct segment and start there.
        """
        results: Dict[str, Tuple[float, float, float, float, float]] = {}

        # Where the terminal brake must start.
        s_end = cls.s_vals[-1]
        s_star = s_end - ds_end_brake

        # Locate the segment where s_star falls.
        i_base = cls._find_segment_index_for_s(s_star)
        s_base = cls.s_vals[i_base]
        v_base = cls.v_vals[i_base]
        a_base = cls.a_vals[i_base]
        j_base = cls.j_vals[i_base]
        t_base = cls.t_vals[i_base]

        # Move from s_base to s_star.
        if s_star > s_base + 1e-12:
            t_star, v_star, a_star, j_star = CubicMotionSolver.integrate(
                s_star,
                s_base, v_base, a_base, t_base,
                j_imposed=None
            )
        else:
            # Start exactly at s_base
            s_star, v_star, a_star, j_star, t_star = s_base, v_base, a_base, j_base, t_base

        # Compute the three-phase S-brake from s_star to s_end.
        bd = CubicMotionSolver.scurved_deceleration(
            v1=0.0, v0=v_star, a0=a_star, j_max=cls.j_max, a_t_max=cls.a_max
        )

        # If a at s_star (a_star) exceeds a_max, the S_brake has a pre-phase
        # in which a is brought back to a_max.
        if "dt_pre" in bd and bd["dt_pre"] > 0.0:
            dt_pre = bd["dt_pre"]; ds_pre = bd["ds_pre"]
            s_pre_end = s_star + ds_pre
            t_pre_end = t_star + dt_pre
            v_pre_end = CubicMotionSolver.velocity(dt_pre, v_star, a_star, bd["j_pre"])
            a_pre_end = CubicMotionSolver.acceleration(dt_pre, a_star, bd["j_pre"])

            # Now indicate the motion state at the end of the pre-phase as _star.
            s_star = s_pre_end
            t_star = t_pre_end
            v_star = v_pre_end
            a_star = a_pre_end
            j_star = bd["j_pre"]

        # Keep the motion state at the start of the true S-brake.
        results["s_star"] = (s_star, v_star, a_star, j_star, t_star)

        # Phase 1: j = -j_max until either a=-a_max or the triangular apex
        dt1 = bd["dt1"]; ds1 = bd["ds1"]
        s_brake1 = s_star + ds1
        t_brake1 = t_star + dt1
        v_brake1 = CubicMotionSolver.velocity(dt1, v_star, a_star, -cls.j_max)
        a_brake1 = CubicMotionSolver.acceleration(dt1, a_star, -cls.j_max)
        j_brake1 = -cls.j_max

        # Keep the motion state at the end of phase 1 of the S-brake.
        results["s_brake1"] = (s_brake1, v_brake1, a_brake1, j_brake1, t_brake1)

        if "ds2" not in bd:
            # Triangular: directly flare back with +j_max to a=0 at the end
            dt3 = bd["dt3"]
            t_end = t_brake1 + dt3
            v_end = CubicMotionSolver.velocity(dt3, v_brake1, a_brake1, +cls.j_max)
            a_end = CubicMotionSolver.acceleration(dt3, a_brake1, +cls.j_max)
            j_end = cls.j_max

            # Keep the motion state at the end of the S-brake.
            results["s_end"] = (s_end, v_end, a_end, j_end, t_end)

        else:
            # Trapezoidal: hold a = -a_max, then flare back
            dt2 = bd["dt2"]; ds2 = bd["ds2"]
            s_brake2 = s_brake1 + ds2
            t_brake2 = t_brake1 + dt2
            v_brake2 = CubicMotionSolver.velocity(dt2, v_brake1, a_brake1, 0.0)
            a_brake2 = CubicMotionSolver.acceleration(dt2, a_brake1, 0.0)
            j_brake2 = 0.0

            # Keep the motion state at the end of the intermediate phase 2 of
            # the S-brake.
            results["s_brake2"] = (s_brake2, v_brake2, a_brake2, j_brake2, t_brake2)

            dt3 = bd["dt3"]
            t_end = t_brake2 + dt3
            v_end = CubicMotionSolver.velocity(dt3, v_brake2, a_brake2, +cls.j_max)
            a_end = CubicMotionSolver.acceleration(dt3, a_brake2, +cls.j_max)
            j_end = cls.j_max

            # Keep the motion state at the end of the S-brake.
            results["s_end"] = (s_end, v_end, a_end, j_end, t_end)

        # Splice: replace the tail from i_base onward by the final-brake points
        s_list = [results[k][0] for k in ("s_star", "s_brake1")] + \
                 ([results["s_brake2"][0]] if "s_brake2" in results else []) + \
                 [results["s_end"][0]]
        v_list = [results[k][1] for k in ("s_star", "s_brake1")] + \
                 ([results["s_brake2"][1]] if "s_brake2" in results else []) + \
                 [results["s_end"][1]]
        a_list = [results[k][2] for k in ("s_star", "s_brake1")] + \
                 ([results["s_brake2"][2]] if "s_brake2" in results else []) + \
                 [results["s_end"][2]]
        j_list = [results[k][3] for k in ("s_star", "s_brake1")] + \
                 ([results["s_brake2"][3]] if "s_brake2" in results else []) + \
                 [results["s_end"][3]]
        t_list = [results[k][4] for k in ("s_star", "s_brake1")] + \
                 ([results["s_brake2"][4]] if "s_brake2" in results else []) + \
                 [results["s_end"][4]]


        poses_list = [cls.path.sample_at_s(s) for s in s_list]

        cls.s_vals = cls.s_vals[:i_base + 1] + s_list
        cls.v_vals = cls.v_vals[:i_base + 1] + v_list
        cls.a_vals = cls.a_vals[:i_base + 1] + a_list
        cls.j_vals = cls.j_vals[:i_base + 1] + j_list
        cls.t_vals = cls.t_vals[:i_base + 1] + t_list
        cls.poses = cls.poses[:i_base + 1] + poses_list

        return results

    @classmethod
    def _find_segment_index_for_s(cls, s: float) -> int:
        """
        Returns i such that s_vals[i] <= s <= s_vals[i+1].
        Clamps to the valid range [0, n-2].
        """
        n = len(cls.s_vals)
        if n < 2:
            return 0
        if s <= cls.s_vals[0]:
            return 0
        if s >= cls.s_vals[-1]:
            return n - 2
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if cls.s_vals[mid] <= s:
                lo = mid
            else:
                hi = mid
        return lo

    @classmethod
    def _erode_speed_limit_envelope_bwd(
        cls,
        s_vals: List[float],
        v_lims: List[float],
        cons: MotionConstraints,
        v_end: float,
        *,
        v_eps: float = 1e-9,
        max_iter: int = 40
    ) -> List[float]:
        """
        Erodes the speed envelope v_lim(s) with a backward pass.

        In zones of the path where the speed limit decreases (i.e., where the
        speed limit curve slopes downward), each point is checked to determine
        whether there is sufficient braking distance to slow down to the point
        where the speed limit in that zone is lowest (local minimum). If not,
        a new speed limit is calculated that allows slowing down to the local
        minimum in that zone.
        """
        n = len(s_vals)
        if n == 0:
            return []

        v_hat = list(v_lims)
        a_max = cons.a_t_max
        j_max = cons.j_max

        # Start with the terminal goal (end of the path).
        goal_idx = n - 1
        s_goal = s_vals[goal_idx]
        v_goal = max(v_hat[goal_idx], v_end)

        # Walk backward through the grid.
        for i in range(n - 2, -1, -1):
            s_i = s_vals[i]
            v_lim_i = v_hat[i]

            # If the raw envelope drops here, move the goal upstream.
            if v_lims[i] < v_lims[i + 1] - v_eps:
                goal_idx = i
                s_goal = s_vals[goal_idx]
                v_goal = max(v_lims[goal_idx], 0.0)

            # Total distance available to brake from s_i to the current goal.
            ds_tot = max(s_goal - s_i, 0.0)

            # If there is essentially no distance, just clamp to the goal value.
            if ds_tot <= 0.0:
                v_hat[i] = min(v_lim_i, v_goal)
                continue

            # Upper guess from jerk-less bound (safe, fast).
            v_hi = math.sqrt(max(v_goal ** 2 + 2.0 * a_max * ds_tot, 0.0))
            v_hi = max(v_hi, v_lim_i)

            # f(v) = ds_brake(v -> v_goal | a0=0) - ds_tot  (we want f(v)=0)
            def f(v):
                bd = CubicMotionSolver.scurved_deceleration(
                    v1=v_goal, v0=v, a0=0.0, j_max=j_max, a_t_max=a_max
                )
                return bd["ds_tot"] - ds_tot

            # Ensure bracketing: enlarge v_hi until f(v_hi) >= 0  (enough distance)
            f_hi = f(v_hi)
            grow = 0
            while f_hi < 0.0 and grow < max_iter:
                v_hi *= 1.5
                f_hi = f(v_hi)
                grow += 1

            # Bisection on [v_goal, v_hi]
            lo, hi = v_goal, v_hi
            for _ in range(max_iter):
                mid = 0.5 * (lo + hi)
                if mid - lo < 1e-12:
                    break
                fm = f(mid)
                if fm >= 0.0:
                    hi = mid
                else:
                    lo = mid

            v_prev_max = hi
            v_hat[i] = min(v_lim_i, v_prev_max)

        return v_hat

    @classmethod
    def _erode_speed_limit_envelope_fwd(
        cls,
        s_vals: list[float],
        v_lims: list[float],
        cons: MotionConstraints,
        v_start: float,
        *,
        v_eps: float = 1e-9,
        max_iter: int = 50
    ) -> list[float]:
        """
        Erodes the speed envelope v_lim(s) with a forward pass.

        For each step s[i-1] -> s[i], finds the largest speed v[i] such that the
        required distance to accelerate from v[i-1] to v[i] is smaller or equal
        to the available distance s[i] - s[i-1].
        """
        n = len(s_vals)
        if n == 0:
            return []

        a_max = cons.a_t_max
        j_max = cons.j_max

        v_hat = [0.0] * n
        v_hat[0] = min(max(v_start, 0.0), max(v_lims[0], 0.0))
        a0 = 0.0

        for i in range(1, n):
            ds = max(s_vals[i] - s_vals[i - 1], 0.0)
            v0 = max(v_hat[i - 1], 0.0)
            v1_lim = max(v_lims[i], 0.0)

            # No distance or no incentive to accelerate.
            if ds <= 0.0 or v1_lim <= v0 + v_eps:
                v_hat[i] = min(v1_lim, v0)
                continue

            # If the speed limit at s1 is smaller than the speed at s0.
            if v1_lim <= v0 + v_eps:
                v_hat[i] = v1_lim
                continue

            # If the required acceleration distance is smaller than the
            # available distance ds -> ok.
            ds_need, a1_lim = CubicMotionSolver.accel_distance(
                v0, a0, v1_lim, a_max, j_max
            )
            if ds_need <= ds + 1e-12:
                v_hat[i] = v1_lim
                a0 = min(a1_lim, a_max)  # propagate for next step
                continue

            # Otherwise: find the speed at s1 such that the required
            # acceleration distance for this speed equals the available
            # distance.
            lo, hi = v0, v1_lim
            a1_lo = a0
            # a1_hi = a1_lim  -> not used directly, but kept for clarity
            for _ in range(max_iter):
                mid = 0.5 * (lo + hi)
                if mid - lo < 1e-12:
                    break
                ds_mid, a_end_mid = CubicMotionSolver.accel_distance(
                    v0, a0, mid, a_max, j_max
                )
                if ds_mid <= ds:
                    lo, a1_lo = mid, a_end_mid
                else:
                    hi = mid
            v_hat[i] = lo
            a0 = min(a1_lo, a_max)  # propagate for next step

        return v_hat

    @classmethod
    def _find_next_change_in_speed_limit(
        cls,
        i0: int,
        *,
        eps: float = 1e-9
    ) -> Optional[Tuple[str, int, float, float]]:
        """
        Finds the next rise/drop ahead of index i0 on the speed limit envelope.
        Returns the first index k > i0 where v_lim rises or drops. Returns None
        if there is no rise/drop ahead.
        """
        n = len(cls.s_vals)
        for k in range(i0 + 1, n):
            if cls.v_lims[k] < cls.v_lims[k - 1] - eps:
                return "drop", k, cls.s_vals[k], cls.v_lims[k]
            if cls.v_lims[k] > cls.v_lims[k - 1] + eps:
                return "rise", k, cls.s_vals[k], cls.v_lims[k]
        return None

    @classmethod
    def _get_decel_plan(
        cls,
        cur_index: int,
        s_k: float,
        v_k: float
    ) -> Optional[SpeedPlan]:
        """
        Returns an S-deceleration plan to lower the speed from v0 to v_k at
        position s_k ahead of s0 if v_k < v0, else returns None.
        """
        s0 = cls.s_vals[cur_index - 1]
        s1 = cls.s_vals[cur_index]
        v0 = cls.s_vals[cur_index - 1]
        v1_lim = cls.v_lims[cur_index]

        plan = cls._create_decel_plan(cur_index, s_k, v_k)
        if plan is None:
            return None

        # Ideally, the start of the S-deceleration falls inside this segment
        # [s0, s1].
        if s0 - 1e-12 <= plan.s_star <= s1 + 1e-12:
            return plan

        # The ideal start has been missed -> create a new plan to the first
        # reachable goal ahead.
        if plan.s_star < s0 - 1e-12:
            catch_up = cls._find_catch_up_target(cur_index)
            if catch_up is not None:
                k2, s_goal, v_goal = catch_up
                plan2 = cls._create_decel_plan(cur_index, s_goal, v_goal)
                if plan2 is not None:
                    plan2.s_star = s0  # this will start deceleration immediately
                    return plan2

            # Nothing ahead is exactly reachable -> emergency decel to the end
            # of this segment.
            s_goal = s1
            v_goal = min(v1_lim, v0)
            plan3 = cls._create_decel_plan(cur_index, s_goal, v_goal)
            if plan3 is not None:
                plan3.s_star = s0
                return plan3

        return None

    @classmethod
    def _create_decel_plan(
        cls,
        cur_index: int,
        s_k: float,
        v_k: float
    ) -> Optional[SpeedPlan]:
        """
        Computes an S-deceleration plan to lower the speed v0 at s0 to v_k at
        position s_k. Returns None if v_k >= v0.

        Returns
        -------
        SpeedPlan:
            Dataclass that holds the jerk switching positions in the S-curved
            deceleration process:
            s_star:
                Position ahead of s0 where braking must start (j = -j_max). This
                is the start of the 1st phase of the braking process.
            s_p1_end:
                Position ahead of s_star where jerk is switched from j = -j_max
                to j = 0 or to j = +j_max. This is the end of the 1st phase of
                the braking process, and may be the start of the final phase of
                the braking process, or, if necessary, the start of a 2nd,
                intermediate phase of the braking process (in the intermediate
                phase acceleration is kept limited to a = -a_max).
            s_p2_end:
                Position ahead of s_p1_end where jerk is switched from j = 0 to
                j = +j_max. s_p2_end can be None, if it is possible to switch
                immediately from j = -j_max to j = +j_max (i.e. when the
                required speed reduction to v_k can be accomplished without
                exceeding the lower acceleration limit a = -a_max).
            s_goal:
                Position where the braking process is completed, i.e. position
                s_k. In this position acceleration is brought to zero (a_k = 0)
                and jerk is then switched from j = +jmax to j = 0.
            v_goal:
                Speed at s_goal, i.e. the speed at position s_k.
        """
        v0 = cls.v_vals[cur_index - 1]
        a0 = cls.a_vals[cur_index - 1]

        # Only create S-deceleration plan when v0 is above the target speed v_k.
        if v0 <= v_k + 1e-9:
            return None

        bd = CubicMotionSolver.scurved_deceleration(
            v1=v_k,
            v0=v0,
            a0=a0,
            j_max=cls.j_max,
            a_t_max=cls.a_max
        )
        s_star = s_k - bd["ds_tot"]

        if "ds2" in bd.keys():
            # Trapezoidal deceleration profile.
            ds1 = bd["ds1"]
            ds2 = bd["ds2"]
            s_p1_end = s_star + ds1
            s_p2_end = s_p1_end + ds2

            return SpeedPlan(
                kind="decel",
                s_star=s_star,
                s_p1_end=s_p1_end,
                s_p2_end=s_p2_end,
                s_goal=s_k,
                v_goal=v_k
            )
        else:
            # Triangular deceleration profile.
            ds1 = bd["ds1"]
            s_p1_end = s_star + ds1

            return SpeedPlan(
                kind="decel",
                s_star=s_star,
                s_p1_end=s_p1_end,
                s_p2_end=None,
                s_goal=s_k,
                v_goal=v_k
            )

    @classmethod
    def _find_catch_up_target(
        cls,
        cur_index: int,
        *,
        v_margin: float = 1e-6
    ) -> Optional[Tuple[float, float, float]]:
        """
        Scans forward from cur_index to find the first sample (k, s_k, v_k)
        such that an S-deceleration from (v0, a0) fits in the available distance
        s_k - s0. Returns (k, s_k, v_k), or None if no sample is found.
        """
        s0 = cls.s_vals[cur_index - 1]
        v0 = cls.v_vals[cur_index - 1]
        a0 = cls.a_vals[cur_index - 1]
        n = len(cls.s_vals)

        for k in range(cur_index + 1, n):
            s_k = cls.s_vals[k]
            v_k = cls.v_lims[k]
            ds_avail = s_k - s0

            # No deceleration distance is available -> begin next iteration.
            if ds_avail <= 0.0:
                continue

            # v0 <= v_k: cannot decelerate -> begin next iteration
            if v0 <= v_k + v_margin:
                continue

            # v0 > v_k: check whether enough braking distance is available to
            # decelerate from v0 to v_k: no -> begin next iteration, otherwise
            # return (k, s_k, v_k).
            bd = CubicMotionSolver.scurved_deceleration(
                v1=v_k, v0=v0, a0=a0, j_max=cls.j_max, a_t_max=cls.a_max
            )
            ds_need = bd["ds_tot"]
            if ds_need <= ds_avail - 1e-12:
                return k, s_k, v_k

        return None

    @classmethod
    def _get_accel_plan(
        cls,
        cur_index: int,
        s_k: float,
        v_k: float
    ) -> Optional[SpeedPlan]:
        """
        Returns an S-acceleration plan to raise the speed from v0 to v_k at
        position s_k ahead of s0 if v_k > v0, else returns None.
        """
        v0 = cls.v_vals[cur_index - 1]
        a0 = cls.a_vals[cur_index - 1]

        # Acceleration is only possible if v0 < v_k.
        if v0 >= v_k - 1e-9:
            return None

        # Acceleration distance to reach v_k from (v0, a0) without flare-out
        # (phase 3 with j = -j to go to a = 0 is omitted here).
        ds, a_k = CubicMotionSolver.accel_distance(v0, a0, v_k, cls.a_max, cls.j_max)
        s_star = s_k - ds

        # Split distances for phases so _solve_with_speed_plan can follow them.
        # Recompute the breakdown (same formulas as in CubicMotionSolver.accel_distance):
        J, A = cls.j_max, cls.a_max
        dt_ramp = max(0.0, (A - a0) / J)
        dv_ramp = a0 * dt_ramp + 0.5 * J * dt_ramp ** 2
        dv_need = v_k - v0

        if dv_need <= dv_ramp + 1e-12:
            # Triangular: only phase 1 (+j_max) up to s_k; no phase-2, no phase-3.
            disc = a0 ** 2 + 2.0 * J * dv_need
            dt1 = (-a0 + (disc if disc > 0.0 else 0.0) ** 0.5) / J
            ds1 = v0 * dt1 + 0.5 * a0 * dt1 ** 2 + (J * dt1 ** 3) / 6.0
            s_p1_end = s_star + ds1

            return SpeedPlan(
                kind="accel",
                s_star=max(s_star, 0.0),
                s_p1_end=s_p1_end,
                s_p2_end=None,
                s_goal=s_k,
                v_goal=v_k
            )

        # Trapezoidal: phase-1 (+j_max) to a_max, then phase-2 (a=+a_max) to s_k.
        ds1 = v0 * dt_ramp + 0.5 * a0 * dt_ramp ** 2 + (J * dt_ramp ** 3) / 6.0
        v_after = v0 + dv_ramp
        dv_left = dv_need - dv_ramp
        dt2 = dv_left / A
        ds2 = v_after * dt2 + 0.5 * A * dt2 ** 2
        s_p1_end = s_star + ds1
        s_p2_end = s_p1_end + ds2

        return SpeedPlan(
            kind="accel",
            s_star=max(s_star, 0.0),
            s_p1_end=s_p1_end,
            s_p2_end=s_p2_end,
            s_goal=s_k,
            v_goal=v_k
        )

    @classmethod
    def _solve_with_speed_plan(cls, cur_index: int, plan: SpeedPlan) -> int:
        s1 = cls.s_vals[cur_index]
        s0 = cls.s_vals[cur_index - 1]
        v0 = cls.v_vals[cur_index - 1]
        a0 = cls.a_vals[cur_index - 1]
        j0 = cls.j_vals[cur_index - 1]
        t0 = cls.t_vals[cur_index - 1]

        # Follow the plan inside s0->s1 by splitting at phase boundaries.
        s = s0; v = v0; a = a0; j = j0; t = t0

        # Helper to run one sub-chunk with imposed jerk and commit results
        # locally.
        def take_chunk(s_target: float, j_imposed: Optional[float]) -> None:
            nonlocal s, v, a, j, t
            t, v, a, j = CubicMotionSolver.integrate(
                s_target, s, v, a, t,
                j_imposed=j_imposed
            )
            # Advance local s to exact target.
            s = s_target

        # If we haven't reached the plan's start yet, move to s_star first.
        if s < plan.s_star:
            # Before s_star, we use default jerk-policy.
            take_chunk(s_target=min(s1, plan.s_star), j_imposed=None)
            if s >= s1 - 1e-12:
                cls.t_vals.append(t)
                cls.v_vals.append(v)
                cls.a_vals.append(a)
                cls.j_vals.append(j)

                cls._print_to_screen(cur_index, "FLOATING")
                return 0

        # Phase 1: from s_star up to s_p1_end.
        if s < plan.s_p1_end:
            j1 = -cls.j_max if plan.kind == "decel" else cls.j_max
            take_chunk(s_target=min(s1, plan.s_p1_end), j_imposed=j1)
            if s >= s1 - 1e-12:
                cls.t_vals.append(t)
                cls.v_vals.append(v)
                cls.a_vals.append(a)
                cls.j_vals.append(j)

                cls._print_to_screen(cur_index, plan.kind.upper() + "-P1")
                return 0

        # Phase 2: from s_p1_end up to s_p2_end (if present).
        if plan.s_p2_end is not None and s < plan.s_p2_end:
            take_chunk(s_target=min(s1, plan.s_p2_end), j_imposed=0.0)
            if s >= s1 - 1e-12:
                cls.t_vals.append(t)
                cls.v_vals.append(v)
                cls.a_vals.append(a)
                cls.j_vals.append(j)

                cls._print_to_screen(cur_index, plan.kind.upper() + "-P2")
                return 0

        # Phase 3: Only for decel plans (flare back to a=0 from s_p1_end or
        # s_p2_end up to s1). Note that accel plans have no P3 .
        if plan.kind == "decel" and s < plan.s_goal:
            j3 = cls.j_max
            take_chunk(s_target=min(s1, plan.s_goal), j_imposed=j3)
            if s >= s1 - 1e-12:
                cls.t_vals.append(t)
                cls.v_vals.append(v)
                cls.a_vals.append(a)
                cls.j_vals.append(j)

                cls._print_to_screen(cur_index, plan.kind.upper() + "-P3")
                return 0

        # If s_goal was before s1, finish the remainder of [s -> s1] with
        # default jerk-policy.
        if s < s1 - 1e-12:
            t, v, a, j = CubicMotionSolver.integrate(
                s1, s, v, a, t,
                j_imposed=None
            )

        cls.t_vals.append(t)
        cls.v_vals.append(v)
        cls.a_vals.append(a)
        cls.j_vals.append(j)

        cls._print_to_screen(cur_index, "FLOATING")
        return 0

    @classmethod
    def _print_to_screen(cls, cur_index: int, plan: str, turn_off: bool = True):
        if not turn_off:
            t = cls.t_vals[cur_index]
            s = cls.s_vals[cur_index]
            v = cls.v_vals[cur_index]
            v_lim = cls.v_lims[cur_index]
            a = cls.a_vals[cur_index]
            j = cls.j_vals[cur_index]
            print(
                f"{cur_index:<4d} | "
                f"{plan:<20s}: "
                f"t={t:8.3f}, "
                f"s={s:8.3f}, "
                f"v={v:8.3f} [{v_lim:8.3f}], "
                f"a={a:8.3f}, "
                f"j={j:+10.3f}"
            )
