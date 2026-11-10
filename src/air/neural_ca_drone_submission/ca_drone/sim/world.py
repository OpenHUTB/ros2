"""World model for CA-Sim.

Coordinates follow ROS REP-103: x forward, y left, z up.  All lengths are in
metres and all angles in radians.  The world is a collection of vertical
extruded obstacles (axis-aligned boxes and cylinders) laid out on a ground
plane.  A small number of obstacles may oscillate along one axis, which yields
the *dynamic* scenes used to test reactive avoidance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple, Union

import numpy as np

# A muted, high-contrast palette used by the software renderer.
COLORS = {
    "concrete": (150, 154, 162),
    "concrete_dark": (112, 118, 128),
    "brick": (176, 112, 84),
    "teal": (64, 148, 152),
    "steel": (104, 126, 154),
    "sand": (188, 168, 120),
    "rust": (170, 96, 62),
    "mover": (206, 120, 58),
}


@dataclass
class Box:
    x: float
    y: float
    z0: float
    sx: float
    sy: float
    sz: float
    color: Tuple[int, int, int] = COLORS["concrete"]
    tag: str = "box"
    moving: bool = False

    @property
    def center(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z0 + 0.5 * self.sz], dtype=float)

    @property
    def half(self) -> np.ndarray:
        return np.array([0.5 * self.sx, 0.5 * self.sy, 0.5 * self.sz], dtype=float)

    def aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        c = self.center
        h = self.half
        return c - h, c + h

    def distance(self, p: np.ndarray) -> float:
        lo, hi = self.aabb()
        q = np.maximum(np.maximum(lo - p, p - hi), 0.0)
        return float(np.linalg.norm(q))

    def contains_xy(self, xy: np.ndarray, pad: float = 0.0) -> bool:
        return (abs(xy[0] - self.x) <= 0.5 * self.sx + pad and
                abs(xy[1] - self.y) <= 0.5 * self.sy + pad)


@dataclass
class Cylinder:
    x: float
    y: float
    z0: float
    radius: float
    height: float
    color: Tuple[int, int, int] = COLORS["steel"]
    tag: str = "cylinder"
    moving: bool = False

    @property
    def center(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z0 + 0.5 * self.height])

    def distance(self, p: np.ndarray) -> float:
        radial = math.hypot(p[0] - self.x, p[1] - self.y)
        dr = max(0.0, radial - self.radius)
        dz = max(0.0, self.z0 - p[2], p[2] - (self.z0 + self.height))
        return math.hypot(dr, dz)

    def contains_xy(self, xy: np.ndarray, pad: float = 0.0) -> bool:
        return math.hypot(xy[0] - self.x, xy[1] - self.y) <= self.radius + pad


Obstacle = Union[Box, Cylinder]


@dataclass
class Mover:
    """An obstacle that oscillates sinusoidally along one axis."""

    base: Obstacle
    axis: int = 1
    amplitude: float = 3.0
    omega: float = 0.5
    phase: float = 0.0

    def at(self, t: float) -> Obstacle:
        offset = self.amplitude * math.sin(self.omega * t + self.phase)
        o = _clone(self.base)
        if self.axis == 0:
            o.x += offset
        else:
            o.y += offset
        o.moving = True
        return o


def _clone(o: Obstacle) -> Obstacle:
    if isinstance(o, Box):
        return Box(o.x, o.y, o.z0, o.sx, o.sy, o.sz, o.color, o.tag, o.moving)
    return Cylinder(o.x, o.y, o.z0, o.radius, o.height, o.color, o.tag, o.moving)


@dataclass
class World:
    name: str
    start: np.ndarray
    goal: np.ndarray
    static: List[Obstacle] = field(default_factory=list)
    movers: List[Mover] = field(default_factory=list)
    bounds: Tuple[float, float, float, float] = (0.0, 52.0, -22.0, 22.0)
    flight_z: float = 3.0
    t: float = 0.0

    def reset(self) -> None:
        self.t = 0.0

    def step(self, dt: float) -> None:
        self.t += dt

    def obstacles(self) -> List[Obstacle]:
        return list(self.static) + [m.at(self.t) for m in self.movers]

    def clearance(self, p: np.ndarray) -> float:
        if not self.static and not self.movers:
            return 99.0
        return min(o.distance(p) for o in self.obstacles())

    def collides(self, p: np.ndarray, radius: float = 0.35) -> bool:
        return self.clearance(p) <= radius

    def inside(self, p: np.ndarray, margin: float = 1.0) -> bool:
        x0, x1, y0, y1 = self.bounds
        return (x0 + margin <= p[0] <= x1 - margin and
                y0 + margin <= p[1] <= y1 - margin)

    # ------------------------------------------------------------------ grids
    def occupancy(self, x0: float, x1: float, y0: float, y1: float,
                  resolution: float, z: float, inflation: float = 0.45) -> np.ndarray:
        """Rasterise obstacles at flight altitude into a boolean BEV grid."""
        nx = int(round((x1 - x0) / resolution))
        ny = int(round((y1 - y0) / resolution))
        grid = np.zeros((nx, ny), dtype=bool)
        xs = x0 + (np.arange(nx) + 0.5) * resolution
        ys = y0 + (np.arange(ny) + 0.5) * resolution
        for o in self.obstacles():
            if o.z0 > z + 0.6 or o.z0 + _height(o) < z - 0.6:
                continue
            if isinstance(o, Box):
                ix = np.where(np.abs(xs - o.x) <= 0.5 * o.sx + inflation)[0]
                iy = np.where(np.abs(ys - o.y) <= 0.5 * o.sy + inflation)[0]
                if len(ix) and len(iy):
                    grid[np.ix_(ix, iy)] = True
            else:
                gx, gy = np.meshgrid(xs, ys, indexing="ij")
                mask = (gx - o.x) ** 2 + (gy - o.y) ** 2 <= (o.radius + inflation) ** 2
                grid |= mask
        return grid

    def raycast(self, origin: np.ndarray, directions: np.ndarray,
                max_range: float = 25.0) -> Tuple[np.ndarray, np.ndarray]:
        """Vectorised ray casting.

        Parameters
        ----------
        origin : (3,) array
        directions : (N, 3) unit vectors

        Returns
        -------
        depth : (N,) distance to the closest surface (``max_range`` if none)
        obj_id : (N,) index into ``obstacles()`` or -1 for ground/sky
        """
        n = directions.shape[0]
        depth = np.full(n, max_range, dtype=np.float64)
        obj_id = np.full(n, -1, dtype=np.int32)
        obs = self.obstacles()

        # ground plane z = 0
        dz = directions[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            tg = -origin[2] / dz
        valid = (dz < -1e-6) & (tg > 0) & (tg < depth)
        depth[valid] = tg[valid]

        for idx, o in enumerate(obs):
            if isinstance(o, Box):
                lo, hi = o.aabb()
                t0 = np.full(n, 0.0)
                t1 = np.full(n, np.inf)
                hit = np.ones(n, dtype=bool)
                for k in range(3):
                    d = directions[:, k]
                    with np.errstate(divide="ignore", invalid="ignore"):
                        inv = 1.0 / d
                        ta = (lo[k] - origin[k]) * inv
                        tb = (hi[k] - origin[k]) * inv
                    tmin = np.minimum(ta, tb)
                    tmax = np.maximum(ta, tb)
                    hit &= np.isfinite(tmin) & np.isfinite(tmax)
                    t0 = np.maximum(t0, tmin)
                    t1 = np.minimum(t1, tmax)
                ok = hit & (t0 <= t1) & (t1 > 0.05) & (t0 < depth)
                depth[ok] = t0[ok]
                obj_id[ok] = idx
            else:
                dx = directions[:, 0]
                dy = directions[:, 1]
                ox = origin[0] - o.x
                oy = origin[1] - o.y
                a = dx * dx + dy * dy
                b = 2.0 * (ox * dx + oy * dy)
                c = ox * ox + oy * oy - o.radius * o.radius
                disc = b * b - 4.0 * a * c
                ok = (a > 1e-9) & (disc >= 0)
                if not np.any(ok):
                    continue
                sq = np.sqrt(np.maximum(disc, 0.0))
                t = (-b - sq) / (2.0 * np.maximum(a, 1e-9))
                z = origin[2] + t * dz
                ok &= (t > 0.05) & (z >= o.z0) & (z <= o.z0 + o.height) & (t < depth)
                depth[ok] = t[ok]
                obj_id[ok] = idx
        return depth, obj_id


def _height(o: Obstacle) -> float:
    return o.sz if isinstance(o, Box) else o.height


# --------------------------------------------------------------------------- scenes
def _box(x, y, sx, sy, sz=6.0, color="concrete", z0=0.0) -> Box:
    c = COLORS[color] if isinstance(color, str) else tuple(color)
    return Box(x, y, z0, sx, sy, sz, c)


def _cyl(x, y, r, h=6.0, color="steel") -> Cylinder:
    c = COLORS[color] if isinstance(color, str) else tuple(color)
    return Cylinder(x, y, 0.0, r, h, c)


def _sparse() -> World:
    objs = [
        _box(9, 5, 3.0, 3.0, color="brick"),
        _box(15, -5.5, 3.5, 3.5, color="concrete"),
        _box(22, 4, 2.5, 4.0, color="teal"),
        _box(29, -5, 4.0, 3.0, color="sand"),
        _box(36, 5, 3.0, 3.0, color="concrete_dark"),
        _box(42, -4, 2.5, 3.5, color="brick"),
        _cyl(18, -1, 1.1, 6.0, "steel"),
        _cyl(33, 0.5, 1.4, 6.0, "rust"),
        _cyl(45, 4, 1.0, 6.0, "steel"),
    ]
    return World("sparse", np.array([2.0, 0.0, 3.0]), np.array([49.0, 0.0, 3.0]), objs)


def _dense() -> World:
    rng = np.random.default_rng(7)
    objs: List[Obstacle] = []
    for i in range(7):
        x = 8.0 + i * 5.5
        for j in (-1, 1):
            if rng.random() < 0.85:
                y = j * (2.6 + 1.6 * rng.random()) + rng.normal(0, 0.3)
                sx = 1.4 + rng.random() * 1.4
                sy = 1.4 + rng.random() * 1.4
                objs.append(_box(x + rng.normal(0, 0.6), y, sx, sy,
                                 color=("brick", "teal", "concrete_dark")[i % 3]))
        if rng.random() < 0.7:
            objs.append(_cyl(x + 2.5, rng.uniform(-4.5, 4.5), 0.8 + 0.5 * rng.random(),
                             6.0, "rust"))
    return World("dense", np.array([2.0, 0.0, 3.0]), np.array([49.0, 0.0, 3.0]), objs)


def _dynamic() -> World:
    w = _sparse()
    w.name = "dynamic"
    w.movers = [
        Mover(_cyl(12, 0, 0.9, 6.0, "mover"), axis=1, amplitude=6.0, omega=0.55, phase=0.0),
        Mover(_cyl(24, 0, 1.0, 6.0, "mover"), axis=1, amplitude=7.0, omega=0.42, phase=1.4),
        Mover(_box(31, 0, 1.6, 1.6, 6.0, COLORS["mover"]), axis=1, amplitude=4.5,
              omega=0.7, phase=2.2),
        Mover(_cyl(39, 0, 0.8, 6.0, "mover"), axis=1, amplitude=5.5, omega=0.5, phase=3.1),
    ]
    return w


def _corridor() -> World:
    objs: List[Obstacle] = []
    # Staggered walls with 5.6 m doors: a slalom that requires global planning.
    # The doors are wide enough for a 0.32 m-radius quadrotor to follow a
    # smoothed path at cruise speed, which keeps the benchmark about planning
    # rather than about an impossible corner.
    for x, gap in ((9.0, -2.6), (17.0, 2.6), (25.0, -2.6), (33.0, 2.6), (41.0, -2.2)):
        left = gap - 2.8
        right = gap + 2.8
        objs.append(_box(x, (-22.0 + left) / 2, 1.0, max(0.5, left + 22.0), 6.0,
                         "concrete"))
        objs.append(_box(x, (right + 22.0) / 2, 1.0, max(0.5, 22.0 - right), 6.0,
                         "concrete_dark"))
    objs.append(_cyl(21.0, 6.0, 1.2, 6.0, "rust"))
    objs.append(_cyl(37.0, -6.0, 1.0, 6.0, "steel"))
    return World("corridor", np.array([2.0, 0.0, 3.0]), np.array([49.0, 0.0, 3.0]), objs)


def _unseen() -> World:
    """Held-out scene: never used to collect training data.

    Two offset gates on *opposite* sides compared with the corridor scene, plus
    a deterministic pillar field.  It is the zero-shot generalisation check for
    the four learnt modules.
    """
    objs: List[Obstacle] = []
    for x, gap in ((14.0, 3.4), (30.0, -3.4)):
        left = gap - 3.0
        right = gap + 3.0
        objs.append(_box(x, (-22.0 + left) / 2, 1.0, max(0.5, left + 22.0), 6.0,
                         "concrete_dark"))
        objs.append(_box(x, (right + 22.0) / 2, 1.0, max(0.5, 22.0 - right), 6.0,
                         "concrete"))
    pillars = [
        (7.0, 2.5, 0.9), (9.5, -2.2, 0.8), (20.0, 3.2, 1.0), (22.0, -3.4, 0.9),
        (24.0, 0.6, 0.7), (38.0, 3.0, 0.9), (41.0, -2.8, 1.0), (44.0, 1.5, 0.8),
    ]
    for i, (x, y, radius) in enumerate(pillars):
        objs.append(_cyl(x, y, radius, 6.0, ("rust", "steel", "teal")[i % 3]))
    return World("unseen", np.array([2.0, 0.0, 3.0]), np.array([49.0, 0.0, 3.0]), objs)


SCENES: Dict[str, callable] = {
    "sparse": _sparse,
    "dense": _dense,
    "dynamic": _dynamic,
    "corridor": _corridor,
    "unseen": _unseen,
}


def make_world(name: str = "sparse") -> World:
    if name not in SCENES:
        raise ValueError(f"unknown scene {name!r}; choose from {sorted(SCENES)}")
    return SCENES[name]()
