from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Iterable, Union, Optional

# ---------------------
# Basic types & helpers
# ---------------------
EPS = 1e-12  # small guard to avoid division by zero and NaNs


@dataclass(frozen=True)
class Point:
    """Tuple-like 2D point with configurable rounding.

    This class behaves like a lightweight tuple (supports indexing and
    iteration) but rounds coordinates on creation to reduce floating-point
    noise in equality comparisons and test assertions.

    Use `Point.set_decimals(n)` to change the rounding precision globally.
    """
    x: float
    y: float

    # Class-wide rounding precision (number of decimals). Defaults to 9.
    _DECIMALS: int = field(default=9, init=False, repr=False, compare=False)

    def __post_init__(self):
        object.__setattr__(self, 'x', round(float(self.x), self._DECIMALS))
        object.__setattr__(self, 'y', round(float(self.y), self._DECIMALS))

    # --- tuple-like API ---
    def __iter__(self):
        yield self.x
        yield self.y

    def __len__(self):
        return 2

    def __getitem__(self, idx: int) -> float:
        if idx == 0:
            return self.x
        if idx == 1:
            return self.y
        raise IndexError("Point only has indices 0 and 1")

    def as_tuple(self) -> Tuple[float, float]:
        """Return (x, y) as a plain tuple of floats."""
        return self.x, self.y

    @classmethod
    def set_decimals(cls, n: int) -> None:
        """Set global rounding precision (number of decimals)."""
        if not isinstance(n, int) or n < 0:
            raise ValueError("decimals must be a non-negative integer")
        cls._DECIMALS = n

    def __str__(self) -> str:
        return f"({self.x}, {self.y})"


def _dot(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by


def _crossz(ax: float, ay: float, bx: float, by: float) -> float:
    """2D cross product z-component (a x b)."""
    return ax * by - ay * bx


def _norm(ax: float, ay: float) -> float:
    return math.hypot(ax, ay)


def _unit(ax: float, ay: float) -> Tuple[float, float]:
    n = _norm(ax, ay)
    if n == 0.0:
        return 0.0, 0.0
    return ax / n, ay / n


def _add_line(segments: List["Segment"], a: Point, b: Point) -> None:
    """Append a non-degenerate straight segment to a segments list."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    if math.hypot(dx, dy) > EPS:
        segments.append(LineSeg(a, b))


# -----------------------------------
# Data classes for poses and segments
# -----------------------------------

@dataclass
class Pose:
    """
    Geometric pose along the path.

    Pose represents a local state along the path. It bundles exactly what you
    need to steer further or derive speed limits:
    - "where am I" (x,y),
    - "where am I pointing" (theta), and
    - "how sharply does the path bend" (kappa).
    That last one is key for speed planning in corners.

    Attributes
    ----------
    x: float
    y: float
        The Cartesian position on the path (in your work units, e.g., meters or
        mm).
    theta: float
        The tangent direction (heading) of the path in radians at that point.
        Convention here: 0 radians points along the +x-axis; positive is
        counterclockwise (CCW). Practically: this is the direction your
        tool/cart is moving at that point.
    kappa: float
        The curvature κ in [1/length] that point.
        - Straight ahead: κ=0.
        - Arc with radius: ∣κ∣=1/r.
        The sign indicates the direction of the curve: in this code, positive
        for a left turn (CCW), negative for a right turn (CW).
        Important physics: lateral acceleration a_lat = v^2 * ∣κ∣. This is used
        to determine the speed limit in curves: v ≤ sqrt(a_lat_max/∣κ∣)
    """
    x: float
    y: float
    theta: float   # heading [rad]
    kappa: float   # curvature [1/length]


@dataclass
class LineSeg:
    """Straight segment with zero curvature."""
    p0: Point  # start point
    p1: Point  # end point
    length: float = field(init=False)
    heading: float = field(init=False)  # orientation of the segment

    def __post_init__(self):
        dx = self.p1[0] - self.p0[0]
        dy = self.p1[1] - self.p0[1]
        self.length = math.hypot(dx, dy)
        self.heading = math.atan2(dy, dx)

    def sample(self, s: float) -> Pose:
        """Sample pose at distance s from p0 (0 <= s <= length)."""
        if self.length == 0.0:
            return Pose(self.p0[0], self.p0[1], self.heading, 0.0)
        t = max(0.0, min(1.0, s / self.length))
        x = self.p0[0] + (self.p1[0] - self.p0[0]) * t
        y = self.p0[1] + (self.p1[1] - self.p0[1]) * t
        return Pose(x, y, self.heading, 0.0)


@dataclass
class ArcSeg:
    """
    Circular arc defined by center, radius, start angle and a signed central
    angle.

    Terminology:
      - `p0` / `p1`:
            incoming / outgoing tangency points on the legs (aka T_in / T_out).
      - `ang0`:
            start polar angle (rad) of the radius vector at the arc start
            (at p0).
      - `sweep`:
            signed central angle of the arc (aka "arc angle"), in radians.
            Convention: +CCW (left turn), −CW (right turn).
            Magnitude is always < pi (the shorter way around).
    """
    c: Point
    r: float
    ang0: float   # start angle (rad)
    sweep: float  # signed angle (rad), magnitude < pi

    def __post_init__(self):
        self.length = abs(self.sweep) * self.r

    @property
    def p0(self) -> Point:
        return Point(
            self.c[0] + self.r * math.cos(self.ang0),
            self.c[1] + self.r * math.sin(self.ang0)
        )

    @property
    def p1(self) -> Point:
        return Point(
            self.c[0] + self.r * math.cos(self.ang0 + self.sweep),
            self.c[1] + self.r * math.sin(self.ang0 + self.sweep)
        )

    def sample(self, s: float) -> Pose:
        """Sample pose at arc-length s from start point along the sweep."""
        s_clamped = max(0.0, min(self.length, s))
        # angle progressed along the arc (signed)
        dphi = (s_clamped / self.r) * (1.0 if self.sweep >= 0.0 else -1.0)
        ang = self.ang0 + dphi
        x = self.c[0] + self.r * math.cos(ang)
        y = self.c[1] + self.r * math.sin(ang)
        # tangent heading: +90° for CCW, -90° for CW
        theta = ang + (math.pi / 2.0 if self.sweep >= 0.0 else -math.pi / 2.0)
        kappa = (1.0 / self.r) * (1.0 if self.sweep >= 0.0 else -1.0)
        return Pose(x, y, theta, kappa)

    @classmethod
    def from_corner(
        cls,
        p0: Point,
        p1: Point,
        p2: Point,
        r: float,
        angle_epsilon_deg: float = 1.0
    ) -> Optional[ArcSeg]:
        """
        Build a fillet (circular arc) at corner p1, given neighbors p0 and p2.

        Returns
        -------
        Optional["ArcSeg"]
            The fillet arc at corner p1 if it can be constructed; None otherwise.
            Notes: arc.p0 == t_in (incoming tangency), arc.p1 == t_out (outgoing).

        Notes
        -----
        -   Uses theta defined as the angle between unit directions away from p1.
            With this convention: theta = pi for straight, 0 for U-turn.
        -   Local radius may be reduced so that tangency points stay on their legs.
        """
        # Direction (unit) vectors pointing AWAY from the corner p1 along each leg:
        d1x, d1y = _unit(p0[0] - p1[0], p0[1] - p1[1])  # from p1 to p0
        d2x, d2y = _unit(p2[0] - p1[0], p2[1] - p1[1])  # from p1 to p2
        if _norm(d1x, d1y) == 0.0 or _norm(d2x, d2y) == 0.0:
            return None

        # Turn angle theta between d1 and d2 (pi = straight, 0 = U-turn)
        cos_th = max(-1.0, min(1.0, _dot(d1x, d1y, d2x, d2y)))
        theta = math.acos(cos_th)
        # Ignore tiny turns (near 0°) and near-straight corners (near 180°)
        theta_deg = math.degrees(theta)
        if (theta_deg < angle_epsilon_deg) or ((180.0 - theta_deg) < angle_epsilon_deg):
            return None

        # Distance from corner to each tangency point
        # Limit r locally if segments are short
        L1_max = _norm(p1[0] - p0[0], p1[1] - p0[1])
        L2_max = _norm(p2[0] - p1[0], p2[1] - p1[1])
        r_max = min(L1_max, L2_max) * math.tan(theta / 2.0)
        r_loc = min(r, r_max)
        tan_half = max(math.tan(theta / 2.0), EPS)
        L = r_loc / tan_half

        # Tangency points on each leg
        t_in = (p1[0] + d1x * L, p1[1] + d1y * L)
        t_out = (p1[0] + d2x * L, p1[1] + d2y * L)

        # Center lies along angle bisector
        bx, by = _unit(d1x + d2x, d1y + d2y)
        dist_center = r_loc / max(math.sin(theta / 2.0), EPS)
        cx = p1[0] + bx * dist_center
        cy = p1[1] + by * dist_center

        # Arc start/end angles + sweep sign (left turn = CCW)
        ang_in = math.atan2(t_in[1] - cy, t_in[0] - cx)
        ang_out = math.atan2(t_out[1] - cy, t_out[0] - cx)

        # Turn sense via cross product using *path* directions
        # Incoming path direction at p1 is (p1 - p0) = -d1; outgoing is (p2 - p1) = d2
        # Left turn (CCW) if cross(-d1, d2) > 0
        left_turn = _crossz(-d1x, -d1y, d2x, d2y) > 0.0

        # Compute signed sweep in range (-pi, pi)
        def wrap(d: float) -> float:
            return (d + math.pi) % (2.0 * math.pi) - math.pi

        sweep = wrap(ang_out - ang_in)
        if left_turn:
            if sweep < 0.0:
                sweep += 2.0 * math.pi  # choose the small positive arc
            if sweep > math.pi:
                sweep -= 2.0 * math.pi  # keep magnitude < pi
        else:
            if sweep > 0.0:
                sweep -= 2.0 * math.pi
            if sweep < -math.pi:
                sweep += 2.0 * math.pi

        return cls(Point(cx, cy), r_loc, ang_in, sweep)


Segment = Union[LineSeg, ArcSeg]


@dataclass
class FilletPath:
    """
    Piecewise linear + circular-arc path created by rounding polyline corners.

    Usage:
        path = FilletPath.from_polyline(points, r=0.010)
        for seg in path.segments: ...
    """
    segments: List[Segment]
    sampler: "Sampler" = field(init=False)

    def __post_init__(self):
        self.sampler = Sampler(self)

    @property
    def length(self) -> float:
        return sum(s.length for s in self.segments)

    def sample_at_s(self, s: float) -> Pose:
        """Sample pose at global arc-length s measured from path start."""
        s = max(0.0, min(self.length, s))
        acc = 0.0
        for seg in self.segments:
            seg_len = getattr(seg, "length")
            if s <= acc + seg_len:
                return seg.sample(s - acc)
            acc += seg_len
        # fallback: end of last segment
        return self.segments[-1].sample(self.segments[-1].length)

    @staticmethod
    def from_polyline(
        points: Iterable[Point],
        r: float,
        closed: bool = False,
        angle_epsilon_deg: float = 1.0
    ) -> "FilletPath":
        """
        Build a filleted path from a list of points.

        Parameters
        ----------
        points :
            Polyline vertices.
        r :
            Desired fillet radius (will be reduced locally if geometry is tight).
        closed :
            Whether the polyline is closed (wrap around).
        angle_epsilon_deg :
            Below this turn angle we skip a fillet (near-straight).

        Returns
        -------
        FilletPath
        """
        pts = list(points)
        n = len(pts)
        if n < 2:
            raise ValueError("Need at least two points.")

        segs: List[Segment] = []

        # Cursor along the polyline where the next segment should start
        start_pt: Optional[Point] = pts[0]

        # Iterate real corners only for open paths (not the start and end point
        # of the open path); all corners for closed paths
        if closed:
            idx_iter = range(n)

            def idx(i_: int) -> int:
                return i_ % n
        else:
            if n < 3:
                # Only a single straight is possible
                _add_line(segs, pts[0], pts[-1])
                return FilletPath(segs)

            idx_iter = range(1, n - 1)

            def idx(i_: int) -> int:
                return max(0, min(n - 1, i_))

        for i in idx_iter:
            i_prev = idx(i - 1)
            i_curr = idx(i)
            i_next = idx(i + 1)

            p0 = pts[i_prev]
            p1 = pts[i_curr]
            p2 = pts[i_next]

            # Try to build a fillet arc at corner p1
            arc = ArcSeg.from_corner(p0, p1, p2, r, angle_epsilon_deg)
            if arc is None:
                # No fillet here: follow the polyline up to the corner point p1
                _add_line(segs, start_pt, p1)
                start_pt = p1
                continue

            # Add the incoming straight from current cursor to t_in
            _add_line(segs, start_pt, arc.p0)
            # Add the arc itself
            segs.append(arc)
            # Update cursor for the next connection
            start_pt = arc.p1

        # Finish connection
        if closed and segs:
            _add_line(segs, start_pt, pts[0])
        elif not closed:
            _add_line(segs, start_pt, pts[-1])

        return FilletPath(segs)


class Sampler:

    def __init__(self, fillet_path: FilletPath) -> None:
        self.fillet_path = fillet_path
        self.points: List[float] = []
        self.poses: List[Pose] = []

    def _s_grid(
        self,
        ds: float,
        include_endpoint: bool = True
    ) -> List[float]:
        """
        Return a monotonic list of arc-length positions from 0 to total
        length.

        Parameters
        ----------
        ds : float
            Desired spacing in arc-length units. Must be > 0.
        include_endpoint : bool
            If True, ensure the final element equals the total length (unless it
            already coincides within EPS).

        Notes
        -----
        -   Values are generated as [0, ds, 2*ds, ...] up to (but not exceeding)
            the total length. The last value is appended as exactly `length` when
            `include_endpoint` is True.
        -   This method does not return poses – it only returns the arc-length
            grid. Use :meth:`sample_uniform` or :meth:`iter_sample_uniform` to
            get poses.
        """
        if ds <= 0.0:
            raise ValueError("ds must be > 0.")

        L = self.fillet_path.length
        # Degenerate path
        if L <= EPS:
            return [0.0]

        out: List[float] = []
        s = 0.0
        while s < L - EPS:
            out.append(s)
            s += ds
        if include_endpoint:
            if not out or abs(out[-1] - L) > EPS:
                out.append(L)
        return out

    def sample_uniform(
        self,
        ds: float,
        include_endpoint: bool = True
    ) -> List[Pose]:
        """
        Sample poses at uniform arc-length spacing.

        Parameters
        ----------
        ds : float
            Desired spacing in arc-length units. Must be > 0.
        include_endpoint : bool
            If True, ensure a sample exactly at the path end is included.

        Returns
        -------
        List[Pose]
            Poses sampled at arc-lengths from :meth:`_s_grid`.
        """
        self.points = self._s_grid(ds, include_endpoint)
        self.poses = [
            self.fillet_path.sample_at_s(s)
            for s in self.points
        ]
        return self.poses

    def iter_sample_uniform(self, ds: float, include_endpoint: bool = True):
        """
        Generator that yields (s, Pose) at uniform arc-length spacing.

        Useful when you want to stream samples without materializing a full
        list.
        """
        self.points = self._s_grid(ds, include_endpoint)
        for s in self.points:
            yield s, self.fillet_path.sample_at_s(s)

    def sample_by_count(self, count: int) -> List[Pose]:
        """
        Sample exactly `count` poses uniformly along arc-length, including
        endpoints.

        Parameters
        ----------
        count : int
            Number of points to sample. If count <= 0, raises ValueError.
            If count == 1, returns a single pose at s=0.

        Returns
        -------
        List[Pose]
        """
        if count <= 0:
            raise ValueError("count must be > 0")

        L = self.fillet_path.length
        if count == 1:
            return [self.fillet_path.sample_at_s(0.0)]

        step = L / (count - 1)
        self.points = [i * step for i in range(count - 1)] + [L]
        self.poses = [self.fillet_path.sample_at_s(s) for s in self.points]
        return self.poses

    def get_path_coordinates(self) -> Tuple[List[float], List[float]]:
        x_coords, y_coords = [], []
        if self.poses:
            x_coords, y_coords = zip(*[(pose.x, pose.y) for pose in self.poses])
            return x_coords, y_coords
        return x_coords, y_coords
