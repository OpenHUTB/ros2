"""Dataset collection and feature engineering for the UAVNN aerial project.

The module has four jobs:

1. a fast, *altitude-aware* analytic replica of the UAV lidar (72 beams,
   360 deg) used to build noise-free targets for the denoising network;
2. feature builders shared by every training tool (18 lidar sectors with 4
   beams each, relative goal polar features and no-fly-zone clearance);
3. a geometric expert (A* + pure pursuit + altitude hold + NFZ avoidance)
   that flies the MuJoCo scenes and produces supervised demonstrations, plus
   a *reactive* vector-field policy whose labels are a pure function of the
   features so behaviour cloning is well posed;
4. dataset plumbing (collect / save / load / split) and the discrete
   8-action airspace environment used by the RL comparison.

The expert used for data collection is intentionally self-contained here so
data collection still works when the concurrent SLAM / planning / control
modules are unavailable.  Those modules are imported lazily and used when
possible.
"""

from __future__ import annotations

import glob
import heapq
import inspect
import json
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import grid_to_world, polyline_length, wrap_angle
from .scenes import Scene, get_scene

# --------------------------------------------------------------------- helper
_PI = math.pi
_TWO_PI = 2.0 * _PI

# Exactly the beam angles used by :class:`uav_numpy_nn_slam.sim.UAVSim`.
_SCAN_ANGLES = np.linspace(-_PI, _PI, C.LIDAR_BEAMS, endpoint=False)
# Body-frame directions of the 8 discrete planner / RL actions.
_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (_PI / 4.0)

N_SECTORS = 18                        # 72 beams / 4 beams per sector
SECTOR_BEAMS = C.LIDAR_BEAMS // N_SECTORS
NFZ_MARGIN = 2.0                      # extra safety margin around a no-fly zone
GRID_INFLATION = 1.4                  # obstacle inflation used by the expert
CLEARANCE_SLOW = 10.0                 # m, lidar clearance where the expert slows
CRUISE_SPEED = 0.90 * C.MAX_VX        # m/s, expert forward speed in free space


def scan_angles() -> np.ndarray:
    """Beam angles relative to the heading (rad), 72 of them in [-pi, pi)."""
    return _SCAN_ANGLES.copy()


def action_angles() -> np.ndarray:
    """Body-frame bearing of the 8 discrete actions (rad)."""
    return _ACTION_ANGLES.copy()


def sector_angles(n_sectors: int = N_SECTORS) -> np.ndarray:
    """Relative bearing of each lidar sector centre."""
    per = C.LIDAR_BEAMS // int(n_sectors)
    return _SCAN_ANGLES.reshape(int(n_sectors), per).mean(axis=1)


_SECTOR_ANGLES = sector_angles(N_SECTORS)


def _wrap_vec(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a, dtype=np.float64) + _PI) % _TWO_PI - _PI


# ---------------------------------------------------------------- scene rays
def wall_rects(scene: Scene) -> List[Tuple[float, float, float, float]]:
    """Border walls exactly as they are placed in the MJCF model (0.6 m thick)."""
    w, h = float(scene.width), float(scene.height)
    return [
        (0.0, h / 2.0, 0.6, h / 2.0),        # x = 0 wall
        (w, h / 2.0, 0.6, h / 2.0),          # x = width wall
        (w / 2.0, 0.0, w / 2.0, 0.6),        # y = 0 wall
        (w / 2.0, h, w / 2.0, 0.6),          # y = height wall
    ]


def _ray_rects(origin: np.ndarray, dirs: np.ndarray,
               rects: Sequence[Sequence[float]], max_range: float) -> np.ndarray:
    """Vectorised 2-D ray / axis-aligned-box intersection for a fan of rays.

    ``dirs`` is (N, 2) with unit rows.  Returns the closest hit distance per
    ray (``max_range`` when nothing is hit).
    """
    n = dirs.shape[0]
    best = np.full(n, float(max_range))
    ox0, oy0 = float(origin[0]), float(origin[1])
    dx, dy = dirs[:, 0], dirs[:, 1]
    for rect in rects:
        cx, cy, hw, hh = (float(rect[0]), float(rect[1]),
                          float(rect[2]), float(rect[3]))
        ox, oy = ox0 - cx, oy0 - cy
        tmin = np.full(n, -np.inf)
        tmax = np.full(n, np.inf)
        miss = np.zeros(n, dtype=bool)
        for o, d, half in ((ox, dx, hw), (oy, dy, hh)):
            parallel = np.abs(d) < 1e-9
            outside = parallel & (np.abs(o) > half)
            miss |= outside
            safe = np.where(parallel, 1.0, d)
            t1 = (-half - o) / safe
            t2 = (half - o) / safe
            lo = np.where(parallel, -np.inf, np.minimum(t1, t2))
            hi = np.where(parallel, np.inf, np.maximum(t1, t2))
            tmin = np.maximum(tmin, lo)
            tmax = np.minimum(tmax, hi)
        hit = (~miss) & (tmax >= 0.0) & (tmin <= tmax)
        dist = np.where(hit, np.maximum(tmin, 0.0), np.inf)
        best = np.minimum(best, dist)
    return best


def _ray_circles(origin: np.ndarray, dirs: np.ndarray,
                 circles: Sequence[Sequence[float]],
                 max_range: float) -> np.ndarray:
    """Vectorised 2-D ray / circle intersection for a fan of rays."""
    n = dirs.shape[0]
    best = np.full(n, float(max_range))
    ox0, oy0 = float(origin[0]), float(origin[1])
    dx, dy = dirs[:, 0], dirs[:, 1]
    for circle in circles:
        cx, cy, r = float(circle[0]), float(circle[1]), float(circle[2])
        ox, oy = ox0 - cx, oy0 - cy
        b = ox * dx + oy * dy
        c = ox * ox + oy * oy - r * r
        disc = b * b - c
        ok = disc >= 0.0
        sq = np.sqrt(np.maximum(disc, 0.0))
        t = -b - sq
        inside = c < 0.0
        t = np.where(inside, 0.0, t)
        hit = inside | (ok & (t >= -1e-9))
        dist = np.where(hit, np.maximum(t, 0.0), np.inf)
        best = np.minimum(best, dist)
    return best


def raycast_scan(pose: Sequence[float], scene: Scene,
                 angles: Optional[np.ndarray] = None) -> np.ndarray:
    """Noise-free lidar scan from the analytic scene geometry.

    This is the *altitude aware* replica of :meth:`UAVSim.lidar`: the four
    horizontal beams of the real sensor only hit a building when the vehicle
    flies below the building roof, so obstacles are filtered by height before
    the ray cast.  Border walls (90 m tall) are always included.
    """
    angles = _SCAN_ANGLES if angles is None else np.asarray(angles, dtype=np.float64)
    alt = float(pose[2])
    rects = [r for r in scene.rects if float(r[4]) > alt + 1e-3]
    rects += wall_rects(scene)
    circles = [cv for cv in scene.circles if float(cv[3]) > alt + 1e-3]
    world_angles = float(pose[3]) + angles
    dirs = np.stack([np.cos(world_angles), np.sin(world_angles)], axis=1)
    origin = np.array([float(pose[0]), float(pose[1])], dtype=np.float64)
    ranges = _ray_rects(origin, dirs, rects, C.LIDAR_RANGE_MAX)
    if circles:
        ranges = np.minimum(
            ranges, _ray_circles(origin, dirs, circles, C.LIDAR_RANGE_MAX))
    return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)


def lidar_noise_scan(clean: np.ndarray,
                     rng: np.random.Generator) -> np.ndarray:
    """Apply the exact :meth:`UAVSim.lidar` noise model to a clean scan."""
    ranges = np.asarray(clean, dtype=np.float64).copy()
    ranges = ranges * (1.0 + 0.008 * rng.normal(size=ranges.shape))
    ranges += 0.05 * rng.normal(size=ranges.shape)
    outlier = rng.random(ranges.shape) < 0.04
    ranges[outlier] = rng.uniform(2.0, C.LIDAR_RANGE_MAX, size=int(outlier.sum()))
    dropout = rng.random(ranges.shape) < 0.03
    ranges[dropout] = C.LIDAR_RANGE_MAX
    return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)


# Kept for compatibility with the ROV code / older scripts.
sonar_noise_scan = lidar_noise_scan


def augment_scans(scenes: Sequence[str], n_per_scene: int = 2000,
                  seed: int = C.DEFAULT_SEED,
                  resolution: float = C.PLAN_RESOLUTION,
                  alt_range: Optional[Tuple[float, float]] = None):
    """Synthesise extra noisy / clean scan pairs from random free poses.

    The clean scan is computed analytically from the scene geometry and the
    noise follows the simulator model, so the synthetic pairs have exactly the
    same distribution as the collected data while covering the whole map.
    """
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    lo = C.MIN_ALT + 1.0 if alt_range is None else float(alt_range[0])
    hi = C.MAX_ALT - 1.0 if alt_range is None else float(alt_range[1])
    noisy, clean = [], []
    for scene_name in scenes:
        scene = get_scene(scene_name)
        grid = build_planning_grid(scene, resolution)
        free = np.argwhere(~grid)
        if len(free) == 0:
            continue
        for _ in range(int(n_per_scene)):
            r, c = free[int(rng.integers(len(free)))]
            x, y = grid_to_world(int(c), int(r), resolution)
            pose = (x, y, float(rng.uniform(lo, hi)),
                    float(rng.uniform(-_PI, _PI)))
            cl = raycast_scan(pose, scene)
            clean.append(cl)
            noisy.append(lidar_noise_scan(cl, rng))
    return (np.asarray(noisy, dtype=np.float64),
            np.asarray(clean, dtype=np.float64))


# ------------------------------------------------------------- planning grid
def build_planning_grid(scene: Scene, resolution: float = C.PLAN_RESOLUTION,
                        inflate_radius: float = GRID_INFLATION,
                        nfz_margin: float = NFZ_MARGIN) -> np.ndarray:
    """Occupancy grid (True = blocked) for the expert / A* / random sampling.

    Contains buildings, the outer border, the no-fly zones with an extra
    ``nfz_margin`` and a square safety inflation of ``inflate_radius``.
    """
    grid = scene.occupancy(resolution)
    w, h = float(scene.width), float(scene.height)
    xs = (np.arange(grid.shape[1]) + 0.5) * resolution
    ys = (np.arange(grid.shape[0]) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    grid |= (gx < 0.30) | (gx > w - 0.30) | (gy < 0.30) | (gy > h - 0.30)
    for cx, cy, r in scene.nfz:
        rr = float(r) + float(nfz_margin)
        grid |= (gx - cx) ** 2 + (gy - cy) ** 2 <= rr * rr
    cells = int(round(float(inflate_radius) / float(resolution)))
    if cells > 0:
        padded = np.pad(grid, cells, mode="constant", constant_values=True)
        out = np.zeros_like(grid)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                out |= padded[cells + dr:cells + dr + grid.shape[0],
                              cells + dc:cells + dc + grid.shape[1]]
        grid = out
    return grid


class AStarPlanner:
    """8-connected A* on a boolean occupancy grid."""

    def __init__(self, grid: np.ndarray, resolution: float = C.PLAN_RESOLUTION):
        self.grid = np.asarray(grid, dtype=bool)
        self.resolution = float(resolution)
        self.h, self.w = self.grid.shape

    # -------------------------------------------------------------- helpers
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int(math.floor(float(x) / self.resolution))
        r = int(math.floor(float(y) / self.resolution))
        return max(0, min(self.h - 1, r)), max(0, min(self.w - 1, c))

    def nearest_free(self, r: int, c: int,
                     max_radius: int = 40) -> Optional[Tuple[int, int]]:
        """BFS ring search for the closest free cell."""
        if not self.grid[r, c]:
            return r, c
        seen = {(r, c)}
        frontier = [(r, c)]
        for _ in range(max_radius):
            nxt = []
            for rr, cc in frontier:
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = rr + dr, cc + dc
                        if (nr, nc) in seen:
                            continue
                        if 0 <= nr < self.h and 0 <= nc < self.w:
                            if not self.grid[nr, nc]:
                                return nr, nc
                            seen.add((nr, nc))
                            nxt.append((nr, nc))
            if not nxt:
                break
            frontier = nxt
        return None

    # ----------------------------------------------------------------- plan
    def plan(self, start_xy: Sequence[float],
             goal_xy: Sequence[float],
             max_expansions: int = 400000) -> Optional[List[Tuple[float, float]]]:
        sr, sc = self.world_to_cell(start_xy[0], start_xy[1])
        gr, gc = self.world_to_cell(goal_xy[0], goal_xy[1])
        start = self.nearest_free(sr, sc)
        goal = self.nearest_free(gr, gc)
        if start is None or goal is None:
            return None
        start, goal = tuple(start), tuple(goal)
        if start == goal:
            return [grid_to_world(start[1], start[0], self.resolution)]
        sq2 = math.sqrt(2.0)
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, sq2), (-1, 1, sq2), (1, -1, sq2), (1, 1, sq2)]
        g_score = np.full((self.h, self.w), np.inf, dtype=np.float64)
        parent = np.full((self.h, self.w, 2), -1, dtype=np.int32)
        g_score[start] = 0.0
        heap = [(0.0, 0.0, start[0], start[1])]
        found = False
        expansions = 0
        while heap:
            _f, base, r, c = heapq.heappop(heap)
            if (r, c) == goal:
                found = True
                break
            if base > g_score[r, c] + 1e-9:
                continue
            expansions += 1
            if expansions > int(max_expansions):
                return None
            for dr, dc, cost in moves:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.h and 0 <= nc < self.w):
                    continue
                if self.grid[nr, nc]:
                    continue
                if dr != 0 and dc != 0:
                    if self.grid[r + dr, c] or self.grid[r, c + dc]:
                        continue
                ng = base + cost
                if ng + 1e-12 < g_score[nr, nc]:
                    g_score[nr, nc] = ng
                    parent[nr, nc] = (r, c)
                    hr, hc = abs(goal[0] - nr), abs(goal[1] - nc)
                    hh = (hr + hc) + (sq2 - 2.0) * min(hr, hc)
                    heapq.heappush(heap, (ng + hh, ng, nr, nc))
        if not found:
            return None
        cells = []
        cur = goal
        while cur != (-1, -1):
            cells.append(cur)
            pr, pc = parent[cur[0], cur[1]]
            if pr < 0:
                break
            cur = (int(pr), int(pc))
        cells.reverse()
        return [grid_to_world(c, r, self.resolution) for r, c in cells]


# ---------------------------------------------------------------- features
def scan_features(scans: np.ndarray) -> np.ndarray:
    """Normalise lidar ranges (metres) to [0, 1]."""
    s = np.asarray(scans, dtype=np.float64)
    span = C.LIDAR_RANGE_MAX - C.LIDAR_RANGE_MIN
    return np.clip((s - C.LIDAR_RANGE_MIN) / span, 0.0, 1.0)


def sector_stats(scans: np.ndarray, n_sectors: int = N_SECTORS,
                 stats: Sequence[str] = ("min", "mean")) -> np.ndarray:
    """Per-sector statistics of the normalised lidar scan.

    With the default 18 sectors and the ``min``/``mean`` statistics this maps
    the 72 beams to 36 features (each sector covers 4 adjacent beams).
    """
    s = scan_features(scans)
    if s.ndim == 1:
        s = s[None, :]
    per = C.LIDAR_BEAMS // int(n_sectors)
    blocks = s.reshape(s.shape[0], int(n_sectors), per)
    out = []
    for name in stats:
        if name == "min":
            out.append(blocks.min(axis=2))
        elif name == "max":
            out.append(blocks.max(axis=2))
        elif name == "mean":
            out.append(blocks.mean(axis=2))
        elif name == "std":
            out.append(blocks.std(axis=2))
        else:
            raise ValueError("unknown sector statistic %r" % (name,))
    return np.concatenate(out, axis=1)


def sector_features(scans: np.ndarray,
                    n_sectors: int = N_SECTORS) -> np.ndarray:
    """Minimum normalised range per sector (T x n_sectors)."""
    return sector_stats(scans, n_sectors, ("min",))


def _sector_min_metres(scan: np.ndarray, n_sectors: int = N_SECTORS) -> np.ndarray:
    scan = np.asarray(scan, dtype=np.float64).reshape(-1)
    per = C.LIDAR_BEAMS // int(n_sectors)
    return scan.reshape(int(n_sectors), per).min(axis=1)


def goal_features(pose: np.ndarray, goal: np.ndarray) -> np.ndarray:
    """Relative polar goal features: [dist, sin(bearing), cos(bearing), dz].

    ``goal`` may be a single ``(x, y, altitude)`` triple or a per-sample
    ``(T, 3)`` array (as stored in a collected dataset).
    """
    p = np.asarray(pose, dtype=np.float64)
    single = p.ndim == 1
    p = np.atleast_2d(p)
    g = np.asarray(goal, dtype=np.float64)
    if g.ndim == 1:
        g = g.reshape(1, 3)
    else:
        g = g.reshape(g.shape[0], -1)[:, :3]
    if g.shape[0] == 1:
        g = np.repeat(g, p.shape[0], axis=0)
    elif g.shape[0] != p.shape[0]:
        raise ValueError("goal has %d rows but pose has %d"
                         % (g.shape[0], p.shape[0]))
    dx = g[:, 0] - p[:, 0]
    dy = g[:, 1] - p[:, 1]
    dist = np.hypot(dx, dy)
    bearing = np.arctan2(dy, dx) - p[:, 3]
    bearing = (bearing + _PI) % _TWO_PI - _PI
    feats = np.stack([
        np.clip(dist / 150.0, 0.0, 1.0),
        np.sin(bearing),
        np.cos(bearing),
        np.clip((g[:, 2] - p[:, 2]) / 10.0, -1.0, 1.0),
    ], axis=1)
    return feats[0] if single else feats


def nfz_features(pose: np.ndarray, scene: Optional[Scene]) -> np.ndarray:
    """No-fly-zone features for the two closest zones (6 dims).

    Per zone: ``[clearance/25 clipped, sin(bearing), cos(bearing)]`` where the
    clearance is the signed distance to the zone boundary (negative = inside)
    and the bearing is relative to the current heading.  Pads with
    ``[1, 0, 0]`` when a scene has fewer than two zones.
    """
    p = np.asarray(pose, dtype=np.float64)
    single = p.ndim == 1
    p = np.atleast_2d(p)
    out = np.zeros((p.shape[0], 6), dtype=np.float64)
    out[:, 0] = 1.0
    out[:, 3] = 1.0
    if scene is not None and scene.nfz:
        zones = [(float(cx), float(cy), float(r)) for cx, cy, r in scene.nfz]
        for i in range(p.shape[0]):
            x, y, yaw = float(p[i, 0]), float(p[i, 1]), float(p[i, 3])
            ordered = sorted(zones,
                             key=lambda z: math.hypot(x - z[0], y - z[1]) - z[2])
            for j in range(min(2, len(ordered))):
                cx, cy, r = ordered[j]
                clearance = math.hypot(x - cx, y - cy) - r
                out[i, 3 * j] = float(np.clip(clearance / 25.0, -1.0, 1.0))
                bearing = wrap_angle(math.atan2(cy - y, cx - x) - yaw)
                out[i, 3 * j + 1] = math.sin(bearing)
                out[i, 3 * j + 2] = math.cos(bearing)
    return out[0] if single else out


CONTROL_DIM = 2 * N_SECTORS + 4 + 6      # 36 sector stats + goal polar + NFZ


def control_features(pose: np.ndarray, goal: np.ndarray, scan: np.ndarray,
                     scene: Optional[Scene] = None) -> np.ndarray:
    """Full control-MLP input vector (46 dims for the default layout)."""
    p = np.asarray(pose, dtype=np.float64)
    single = p.ndim == 1
    p2 = np.atleast_2d(p)
    s = np.asarray(scan, dtype=np.float64)
    if s.ndim == 1:
        s = s[None, :]
    sectors = sector_stats(s, N_SECTORS, ("min", "mean"))
    gf = goal_features(p2, goal)
    nf = nfz_features(p2, scene)
    out = np.concatenate([sectors, gf, nf], axis=1)
    return out[0] if single else out


def obs_features(pose: np.ndarray, goal: Sequence[float]) -> np.ndarray:
    """Pose/goal observation vector: goal polar + heading sin/cos (6 dims)."""
    p = np.asarray(pose, dtype=np.float64)
    single = p.ndim == 1
    p = np.atleast_2d(p)
    gf = goal_features(p, goal)
    feats = np.concatenate(
        [gf, np.stack([np.sin(p[:, 3]), np.cos(p[:, 3])], axis=1)], axis=1)
    return feats[0] if single else feats


def image_to_input(images: np.ndarray) -> np.ndarray:
    """uint8 HWC images -> float32 NCHW in [0, 1]."""
    img = np.asarray(images)
    single = img.ndim == 3
    if single:
        img = img[None, ...]
    img = img.astype(np.float32) / 255.0
    if single:
        return np.transpose(img[0], (2, 0, 1))
    return np.transpose(img, (0, 3, 1, 2))


def discretize_heading(err: float, n_actions: int = 8) -> int:
    """Map a signed heading error (rad, + = left) to one of 8 directions."""
    step = _TWO_PI / n_actions
    return int(math.floor(float(err) / step + 0.5)) % n_actions


def quantize_direction(angle: float) -> int:
    """Quantise a signed relative bearing (rad) to the 8 action directions."""
    return discretize_heading(float(angle), 8)


# ------------------------------------------------------ reactive expert
def reactive_action(pose: Sequence[float], goal: Sequence[float],
                    scan: np.ndarray, scene: Optional[Scene] = None,
                    n_sectors: int = N_SECTORS) -> np.ndarray:
    """Vector-field / sector-histogram expert, a pure function of the state.

    Candidate travel directions are the sector centres (body frame).  Each is
    scored by its angular distance to the goal bearing plus an obstacle penalty
    derived from the minimum range inside the sector plus a no-fly-zone
    repulsion term.  A softmax over the scores gives a smooth steering
    direction, which makes the resulting labels easy to imitate.

    Returns the *body frame* command ``(vf, vl, yaw_rate)`` in m/s and rad/s
    (the vertical command is handled by an altitude PID in the benchmark).
    """
    p = np.asarray(pose, dtype=np.float64).reshape(-1)
    g = np.asarray(goal, dtype=np.float64).reshape(-1)[:3]
    yaw = float(p[3])
    dx, dy = float(g[0]) - float(p[0]), float(g[1]) - float(p[1])
    dist_goal = math.hypot(dx, dy)
    bearing = wrap_angle(math.atan2(dy, dx) - yaw)

    ang = (_SECTOR_ANGLES if int(n_sectors) == N_SECTORS
           else sector_angles(int(n_sectors)))
    ranges = _sector_min_metres(scan, int(n_sectors))
    aligned = np.abs(_wrap_vec(ang - bearing))
    obstacle = 2.5 * np.clip(1.0 - ranges / CLEARANCE_SLOW, 0.0, 1.0) ** 1.5
    cost = aligned + obstacle

    # ---- no-fly-zone repulsion (never fly towards a zone we are close to)
    if scene is not None and scene.nfz:
        for cx, cy, r in scene.nfz:
            centre = math.hypot(float(cx) - p[0], float(cy) - p[1])
            clearance = centre - float(r)
            if clearance < 14.0:
                nb = wrap_angle(math.atan2(float(cy) - p[1],
                                           float(cx) - p[0]) - yaw)
                towards = np.clip(np.cos(ang - nb), 0.0, 1.0)
                cost = cost + (6.5 * np.clip(1.0 - clearance / 14.0, 0.0, 1.0)
                               * towards ** 2)
    weights = np.exp(-cost / 0.55)
    total = float(weights.sum())
    if not np.isfinite(total) or total <= 1e-9:
        weights = np.full_like(weights, 1.0 / len(weights))
    else:
        weights = weights / total
    steer = float(np.sum(weights * ang))
    yaw_cmd = float(np.clip(1.8 * steer, -C.MAX_YAW_RATE, C.MAX_YAW_RATE))

    # ---- speed: slow down for close clearance in the chosen direction.
    # The clearance is interpolated on the circular sector curve instead of
    # taking a hard minimum over a fixed set of sectors: a hard minimum makes
    # the label jump discontinuously whenever the steering crosses a sector
    # boundary, which is impossible for a smooth regression network to fit.
    ang_ext = np.concatenate([ang, ang + _TWO_PI])
    r_ext = np.concatenate([ranges, ranges])
    clear = float(np.interp(steer, ang_ext, r_ext))
    speed = CRUISE_SPEED * float(np.clip((clear - 2.0) / 10.0, 0.12, 1.0))
    speed *= 1.0 - 0.45 * min(1.0, abs(steer) / 1.4)
    if dist_goal < 3.0:
        speed *= float(np.clip(dist_goal / 3.0, 0.12, 1.0))
    return np.array([speed * math.cos(steer), speed * math.sin(steer),
                     yaw_cmd], dtype=np.float64)


def reactive_command(pose: Sequence[float], goal: Sequence[float],
                     scan: np.ndarray, scene: Optional[Scene] = None) -> np.ndarray:
    """World-frame ``(vx, vy, yaw_rate)`` of the reactive expert."""
    yaw = float(np.asarray(pose, dtype=np.float64)[3])
    body = reactive_action(pose, goal, scan, scene)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * body[0] - s * body[1],
                     s * body[0] + c * body[1],
                     body[2]], dtype=np.float64)


def reactive_labels(pose: np.ndarray, goal: np.ndarray, scans: np.ndarray,
                    scene: Optional[Scene] = None) -> np.ndarray:
    """Normalised control labels ``(vf/MAX_VX, vl/MAX_VY, yaw/MAX_YAW)``."""
    pose = np.atleast_2d(np.asarray(pose, dtype=np.float64))
    scans = np.asarray(scans, dtype=np.float64)
    goal_arr = np.asarray(goal, dtype=np.float64)
    if goal_arr.ndim == 1:
        goal_arr = np.repeat(goal_arr.reshape(1, 3), len(pose), axis=0)
    out = np.zeros((len(pose), 3), dtype=np.float64)
    for i in range(len(pose)):
        body = reactive_action(pose[i], goal_arr[i], scans[i], scene)
        out[i, 0] = float(np.clip(body[0] / C.MAX_VX, -1.0, 1.0))
        out[i, 1] = float(np.clip(body[1] / C.MAX_VY, -1.0, 1.0))
        out[i, 2] = float(np.clip(body[2] / C.MAX_YAW_RATE, -1.0, 1.0))
    return out


def reactive_commands(pose: np.ndarray, goal: np.ndarray,
                      scans: np.ndarray) -> np.ndarray:
    """Body-frame reactive expert commands for a batch of recorded states."""
    pose = np.atleast_2d(np.asarray(pose, dtype=np.float64))
    scans = np.asarray(scans, dtype=np.float64)
    goal_arr = np.asarray(goal, dtype=np.float64)
    if goal_arr.ndim == 1:
        goal_arr = np.repeat(goal_arr.reshape(1, 3), len(pose), axis=0)
    out = np.zeros((len(pose), 3), dtype=np.float64)
    for i in range(len(pose)):
        out[i] = reactive_action(pose[i], goal_arr[i], scans[i], None)
    return out


def augment_control(scenes: Sequence[str], n_per_scene: int = 2000,
                    seed: int = C.DEFAULT_SEED,
                    resolution: float = C.PLAN_RESOLUTION,
                    alt_spread: float = 4.0):
    """Synthetic states labelled by the reactive expert.

    Returns ``(features, control_labels, action_labels)`` where the labels are
    the normalised body-frame command and the quantised 8-direction planner
    target, both produced by :func:`reactive_action`.
    """
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    xs, ys, acts = [], [], []
    for scene_name in scenes:
        scene = get_scene(scene_name)
        grid = build_planning_grid(scene, resolution)
        free = np.argwhere(~grid)
        if len(free) == 0:
            continue
        goal = np.asarray(scene.goal, dtype=np.float64)
        alt_lo = max(C.MIN_ALT, float(scene.alt) - alt_spread)
        alt_hi = min(C.MAX_ALT, float(scene.alt) + alt_spread)
        for _ in range(int(n_per_scene)):
            r, c = free[int(rng.integers(len(free)))]
            x, y = grid_to_world(int(c), int(r), resolution)
            pose = np.array([x, y, float(rng.uniform(alt_lo, alt_hi)),
                             float(rng.uniform(-_PI, _PI))])
            clean = raycast_scan(pose, scene)
            scan = lidar_noise_scan(clean, rng)
            body = reactive_action(pose, goal, scan, scene)
            xs.append(control_features(pose, goal, scan, scene))
            ys.append(reactive_labels(pose[None, :], goal[None, :],
                                      scan[None, :], scene)[0])
            acts.append(quantize_direction(math.atan2(body[1], body[0])))
    return (np.asarray(xs, dtype=np.float64),
            np.asarray(ys, dtype=np.float64),
            np.asarray(acts, dtype=np.int64))


# ------------------------------------------------------------------- expert
class GeometricExpert:
    """Deterministic A* + pure-pursuit + altitude-hold expert controller.

    This is the fallback expert used for behaviour cloning.  If the concurrent
    ``control.MissionController`` is importable it can be used instead (see
    :func:`make_expert`), but the geometric expert is always available.

    Commands are returned in the *world frame* expected by
    :meth:`uav_numpy_nn_slam.sim.UAVSim.step`: ``(vx, vy, vz(+up), yaw_rate)``.
    """

    kind = "geometric_expert"

    def __init__(self, scene_name: str, resolution: float = C.PLAN_RESOLUTION):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.resolution = float(resolution)
        self.grid = build_planning_grid(self.scene, self.resolution)
        self.planner = AStarPlanner(self.grid, self.resolution)
        self.path: Optional[List[Tuple[float, float]]] = None
        self.path_arr = np.zeros((0, 2))
        self.path_length = 0.0
        self.start_alt = float(self.scene.start[2])
        self.goal = np.asarray(self.scene.goal, dtype=np.float64)
        self.last_info: Dict[str, float] = {}
        self.replans = 0
        self.reset()

    # ------------------------------------------------------------------ api
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y, alt = self.scene.start[:3]
        else:
            x, y, alt = float(pose[0]), float(pose[1]), float(pose[2])
        self.start_alt = alt
        self.replans = 0
        self._plan(x, y)

    def _plan(self, x: float, y: float) -> None:
        self.path = self.planner.plan((x, y), self.goal[:2])
        self.replans += 1
        if self.path is None:
            self.path_arr = np.zeros((0, 2))
            self.path_length = 0.0
        else:
            self.path_arr = np.asarray(self.path, dtype=np.float64)
            self.path_length = max(polyline_length(self.path), 1e-6)

    def _replan_if_needed(self, x: float, y: float) -> None:
        if len(self.path_arr) == 0:
            self._plan(x, y)
            return
        d = np.hypot(self.path_arr[:, 0] - x, self.path_arr[:, 1] - y)
        if float(d.min()) > 3.0:
            self._plan(x, y)

    def _lookahead_target(self, x: float, y: float,
                          lookahead: float = 5.0) -> Tuple[float, float]:
        if len(self.path_arr) == 0:
            return float(self.goal[0]), float(self.goal[1])
        d = np.hypot(self.path_arr[:, 0] - x, self.path_arr[:, 1] - y)
        i = int(np.argmin(d))
        travelled = 0.0
        j = i
        while j + 1 < len(self.path_arr) and travelled < lookahead:
            travelled += math.hypot(self.path_arr[j + 1, 0] - self.path_arr[j, 0],
                                    self.path_arr[j + 1, 1] - self.path_arr[j, 1])
            j += 1
        return float(self.path_arr[j, 0]), float(self.path_arr[j, 1])

    def update(self, pose: Sequence[float], goal: Sequence[float],
               scan: Optional[np.ndarray] = None,
               beam_angles: Optional[np.ndarray] = None,
               dt: float = 1.0 / C.CONTROL_HZ) -> np.ndarray:
        x, y, alt, yaw = [float(v) for v in pose[:4]]
        goal = np.asarray(goal, dtype=np.float64)
        self.goal = goal
        self._replan_if_needed(x, y)
        tx, ty = self._lookahead_target(x, y)
        dist_goal = math.hypot(float(goal[0]) - x, float(goal[1]) - y)
        if dist_goal < 2.0:
            tx, ty = float(goal[0]), float(goal[1])
        desired = math.atan2(ty - y, tx - x)
        err = wrap_angle(desired - yaw)
        yaw_cmd = float(np.clip(1.8 * err, -C.MAX_YAW_RATE, C.MAX_YAW_RATE))

        speed = CRUISE_SPEED * (1.0 - 0.50 * min(1.0, abs(err) / 1.4))
        if dist_goal < 3.0:
            speed *= float(np.clip(dist_goal / 3.0, 0.10, 1.0))
        # emergency stop: something is directly in front of the vehicle
        if scan is not None:
            sc = np.asarray(scan, dtype=np.float64)
            front = np.abs(_SCAN_ANGLES - err) <= 0.55
            if bool(front.any()):
                front_min = float(sc[front].min())
                if front_min < 2.5:
                    speed *= float(np.clip((front_min - 0.8) / 1.7, 0.0, 1.0))
        vx = speed * math.cos(desired)
        vy = speed * math.sin(desired)

        # ---- altitude hold / ramp towards the goal altitude
        if self.path_length > 1e-6:
            progress = 1.0 - min(1.0, dist_goal / self.path_length)
        else:
            progress = 0.0
        target_alt = (self.start_alt
                      + (float(goal[2]) - self.start_alt) * progress)
        vz = float(np.clip(1.5 * (target_alt - alt), -C.MAX_VZ, C.MAX_VZ))

        self.last_info = {
            "desired_heading": desired,
            "heading_err": float(err),
            "target_alt": float(target_alt),
            "dist_goal": float(dist_goal),
            "yaw_cmd": float(yaw_cmd),
            "speed": float(speed),
        }
        return np.array([vx, vy, vz, yaw_cmd], dtype=np.float64)


def make_expert(scene_name: str, prefer_nav: bool = False):
    """Return a MissionController when available (and requested) else expert."""
    if prefer_nav:
        try:
            from . import control as nav_control  # type: ignore
            factory = getattr(nav_control, "MissionController", None)
            if factory is not None:
                return _try_mission_controller(factory, scene_name)
        except Exception:
            pass
    return GeometricExpert(scene_name)


def _try_mission_controller(factory, scene_name: str):
    """Best-effort construction of the concurrent MissionController.

    The constructor signature is owned by the navigation engineer, so known
    parameter names are mapped while any unknown optional parameter keeps its
    default.  Raises ``RuntimeError`` when construction is impossible.
    """
    scene = get_scene(scene_name)
    grid = build_planning_grid(scene)
    try:
        sig = inspect.signature(factory.__init__)
        params = list(sig.parameters.values())[1:]
    except (TypeError, ValueError):
        params = []
    kwargs: Dict[str, object] = {}
    known = {
        "scene_or_map": scene,
        "scene": scene,
        "scene_name": scene_name,
        "map": grid,
        "occupancy": grid,
        "grid": grid,
        "resolution": C.PLAN_RESOLUTION,
        "replan_interval": 1.0,
    }
    for param in params:
        if param.name in known:
            kwargs[param.name] = known[param.name]
        elif param.default is inspect.Parameter.empty:
            raise RuntimeError("MissionController has required parameter %r "
                               "unknown to the benchmark" % param.name)
    try:
        return factory(**kwargs)
    except Exception as exc:
        raise RuntimeError("MissionController could not be constructed: %s"
                           % exc) from exc


# ------------------------------------------------------- discrete RL env
class DiscreteAirspaceEnv:
    """Lightweight discrete-action abstraction of the UAV airspace task.

    The UAV is a point mass moving ``step_size`` metres per step in one of 8
    world-frame directions on the scene occupancy grid (buildings + border)
    while no-fly zones are traversable but heavily penalised.  This keeps RL
    training fast while using the exact same scene geometry as the MuJoCo
    benchmark.
    """

    def __init__(self, scene_name: str = "city",
                 step_size: float = 1.0, max_steps: int = 70,
                 goal_radius: float = 1.5,
                 resolution: float = 0.25,
                 inflate_radius: float = 1.0,
                 random_start: bool = True,
                 seed: int = 2026):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.resolution = float(resolution)
        # collision map: buildings + outer border, inflated by the UAV radius.
        # The no-fly zones are traversable in this abstraction but penalised.
        base = self.scene.occupancy(resolution)
        self._occ = self._inflate(base,
                                  int(round(inflate_radius / resolution)))
        self.step_size = float(step_size)
        self.max_steps = int(max_steps)
        self.goal_radius = float(goal_radius)
        self.random_start = bool(random_start)
        self.start_xy = np.array(self.scene.start[:2], dtype=np.float64)
        self.goal_xy = np.array(self.scene.goal[:2], dtype=np.float64)
        self.angles = _ACTION_ANGLES
        self.rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        self._offsets = np.arange(-1.5, 1.51, 0.5)   # 7 values, 0.5 m apart
        self.state_dim = len(self._offsets) ** 2 + 3 + 3
        self.n_actions = 8
        self.pos = self.start_xy.copy()
        self.steps = 0
        self.collisions = 0
        self.nfz_violations = 0
        self.prev_dist = 0.0
        self.done = False

    # -------------------------------------------------------------- helpers
    @staticmethod
    def _inflate(grid: np.ndarray, cells: int) -> np.ndarray:
        if cells <= 0:
            return grid
        padded = np.pad(grid, cells, mode="constant", constant_values=True)
        out = np.zeros_like(grid)
        h, w = grid.shape
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                out |= padded[cells + dr:cells + dr + h,
                              cells + dc:cells + dc + w]
        return out

    def _cell(self, xy: np.ndarray, grid: Optional[np.ndarray] = None
              ) -> Tuple[int, int]:
        g = self._occ if grid is None else grid
        c = int(math.floor(float(xy[0]) / self.resolution))
        r = int(math.floor(float(xy[1]) / self.resolution))
        c = max(0, min(g.shape[1] - 1, c))
        r = max(0, min(g.shape[0] - 1, r))
        return r, c

    def blocked(self, xy: np.ndarray) -> bool:
        r, c = self._cell(xy)
        return bool(self._occ[r, c])

    def in_nfz(self, xy: np.ndarray) -> bool:
        return bool(self.scene.inside_nfz(float(xy[0]), float(xy[1])))

    def observation(self) -> np.ndarray:
        ox = self.pos[0] + self._offsets
        oy = self.pos[1] + self._offsets
        cols = np.clip((ox / self.resolution).astype(np.int64), 0,
                       self._occ.shape[1] - 1)
        rows = np.clip((oy / self.resolution).astype(np.int64), 0,
                       self._occ.shape[0] - 1)
        patch = self._occ[np.ix_(rows, cols)].astype(np.float64).reshape(-1)
        dx = self.goal_xy[0] - self.pos[0]
        dy = self.goal_xy[1] - self.pos[1]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        goal = np.array([np.clip(dist / 120.0, 0.0, 1.0),
                         math.sin(bearing), math.cos(bearing)])
        clear = self.scene.nfz_clearance(float(self.pos[0]),
                                         float(self.pos[1]))
        nfz_bearing = bearing
        nfz_clear = clear
        if self.scene.nfz:
            zones = sorted(self.scene.nfz,
                           key=lambda z: math.hypot(self.pos[0] - z[0],
                                                    self.pos[1] - z[1]) - z[2])
            cx, cy, _r = zones[0]
            nfz_bearing = math.atan2(cy - self.pos[1], cx - self.pos[0])
            nfz_clear = math.hypot(self.pos[0] - cx, self.pos[1] - cy) - _r
        nfz = np.array([np.clip(nfz_clear / 40.0, -0.5, 1.0),
                        math.sin(nfz_bearing), math.cos(nfz_bearing)])
        return np.concatenate([patch, goal, nfz], axis=0)

    def sample_free_start(self, min_goal_dist: float = 10.0,
                          max_goal_dist: float = 30.0) -> np.ndarray:
        """Random free start position at a reasonable distance from the goal."""
        h, w = self._occ.shape
        for _ in range(500):
            r = int(self.rng.integers(1, h - 1))
            c = int(self.rng.integers(1, w - 1))
            if self._occ[r, c]:
                continue
            xy = np.array(grid_to_world(c, r, self.resolution))
            if self.scene.inside_nfz(float(xy[0]), float(xy[1])):
                continue
            d = float(np.hypot(*(self.goal_xy - xy)))
            if min_goal_dist <= d <= max_goal_dist:
                return xy
        return self.start_xy.copy()

    def reset(self, xy: Optional[Sequence[float]] = None) -> np.ndarray:
        if xy is not None:
            self.pos = np.asarray(xy, dtype=np.float64).copy()
        elif self.random_start:
            self.pos = self.sample_free_start()
        else:
            self.pos = self.start_xy.copy()
        self.steps = 0
        self.collisions = 0
        self.nfz_violations = 0
        self.done = False
        self._inside_nfz = self.in_nfz(self.pos)
        self.prev_dist = float(np.hypot(*(self.goal_xy - self.pos)))
        return self.observation()

    def step(self, action: int):
        self.steps += 1
        direction = self.angles[int(action) % 8]
        delta = self.step_size * np.array([math.cos(direction),
                                           math.sin(direction)])
        new_pos = self.pos + delta
        reward = -0.02
        if self.blocked(new_pos):
            self.collisions += 1
            reward -= 1.0
        else:
            self.pos = new_pos
        inside = self.in_nfz(self.pos)
        if inside and not self._inside_nfz:
            self.nfz_violations += 1
        if inside:
            reward -= 0.05          # per-step penalty while inside a NFZ
        self._inside_nfz = inside
        dist = float(np.hypot(*(self.goal_xy - self.pos)))
        reward += 1.5 * (self.prev_dist - dist)
        self.prev_dist = dist
        success = dist <= self.goal_radius
        if success:
            reward += 10.0
            self.done = True
        elif self.steps >= self.max_steps or self.collisions >= 15:
            self.done = True
        info = {"success": bool(success), "collisions": self.collisions,
                "nfz_violations": self.nfz_violations, "distance": dist}
        return self.observation(), float(reward), bool(self.done), info


# Alias kept so old imports keep working.
DiscretePoolEnv = DiscreteAirspaceEnv


def sample_free_pose(scene: Scene, grid: np.ndarray,
                     rng: np.random.Generator,
                     resolution: float = C.PLAN_RESOLUTION,
                     goal_xy: Optional[Sequence[float]] = None,
                     yaw_spread: float = 0.9,
                     alt_spread: float = 4.0) -> Tuple[float, float, float, float]:
    """Sample a collision-free (and NFZ-free) pose inside the scene.

    When ``goal_xy`` is given the sampled heading points roughly at the goal,
    which keeps demonstrated surge values informative for the camera CNN.
    """
    free = np.argwhere(~grid)
    if len(free) == 0:
        return tuple(float(v) for v in scene.start)
    r, c = free[int(rng.integers(len(free)))]
    x, y = grid_to_world(int(c), int(r), resolution)
    if goal_xy is None:
        yaw = float(rng.uniform(-_PI, _PI))
    else:
        bearing = math.atan2(float(goal_xy[1]) - y, float(goal_xy[0]) - x)
        yaw = wrap_angle(bearing + float(rng.uniform(-yaw_spread, yaw_spread)))
    lo = max(C.MIN_ALT + 2.0, float(scene.alt) - alt_spread)
    hi = min(C.MAX_ALT - 2.0, float(scene.alt) + alt_spread)
    alt = float(rng.uniform(lo, max(lo + 0.2, hi)))
    return x, y, alt, yaw


# ----------------------------------------------------------------- collect
def collect(scene_name: str, seconds: float = 50.0, seed: int = 0,
            noise: bool = True, sample_hz: float = 10.0,
            episodes: int = 2, prefer_nav_expert: bool = False,
            random_start: bool = True) -> Dict[str, np.ndarray]:
    """Run expert episodes in :class:`UAVSim` and return supervised arrays.

    Besides the raw sensor data the expert commands and the expert's label
    set are stored: ``labels`` holds the normalised reactive control target
    ``(vf/MAX_VX, vl/MAX_VY, yaw/MAX_YAW_RATE)`` and ``actions`` holds the
    quantised 8-direction planner target -- both are pure functions of the
    features so behaviour cloning is well posed.
    """
    from .sim import UAVSim

    sim = UAVSim(scene_name, seed=seed, noise=noise)
    expert = make_expert(scene_name, prefer_nav=prefer_nav_expert)
    scene = sim.scene
    beam_angles = sim.lidar_beam_angles()
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    max_steps = max(1, int(round(float(seconds) / dt)))
    rng = np.random.default_rng((int(seed) + 1234) & 0xFFFFFFFF)
    start_grid = build_planning_grid(scene) if random_start else None

    rows: Dict[str, List[np.ndarray]] = {k: [] for k in (
        "pose", "obs", "scans", "scans_clean", "cmds", "cmds_body", "labels",
        "actions", "images", "episode", "time", "nfz", "nfz_clearance",
        "dist_goal")}
    episode_success: List[float] = []
    total_collisions = 0
    total_nfz = 0

    for ep in range(int(episodes)):
        if random_start and ep > 0 and start_grid is not None:
            sim.reset(sample_free_pose(scene, start_grid, rng,
                                       goal_xy=scene.goal[:2]))
        else:
            sim.reset()
        if hasattr(expert, "reset"):
            expert.reset(sim.pose)
        goal = sim.goal()
        last_scan: Optional[np.ndarray] = None
        success = False
        for k in range(max_steps):
            pose = sim.pose.copy()
            if k % sample_every == 0:
                scan = sim.lidar()
                clean = raycast_scan(pose, scene)
                image = sim.camera()
                cmd = expert.update(pose, goal, scan, beam_angles, dt)
                body_cmd = reactive_action(pose, goal, scan, scene)
                action = quantize_direction(
                    math.atan2(body_cmd[1], body_cmd[0]))
                rows["pose"].append(pose)
                rows["obs"].append(control_features(pose, goal, scan, scene))
                rows["scans"].append(scan)
                rows["scans_clean"].append(clean)
                rows["cmds"].append(np.asarray(cmd, dtype=np.float64))
                rows["cmds_body"].append(np.asarray(body_cmd, dtype=np.float64))
                rows["labels"].append(np.asarray(
                    reactive_labels(pose[None, :], goal[None, :],
                                    scan[None, :], scene)[0],
                    dtype=np.float64))
                rows["actions"].append(np.asarray(action, dtype=np.int64))
                rows["images"].append(image)
                rows["episode"].append(np.asarray(ep, dtype=np.int64))
                rows["time"].append(np.asarray(sim.time, dtype=np.float64))
                rows["nfz"].append(nfz_features(pose, scene))
                rows["nfz_clearance"].append(
                    np.asarray(sim.nfz_clearance(), dtype=np.float64))
                rows["dist_goal"].append(np.asarray(
                    math.hypot(float(pose[0]) - goal[0],
                               float(pose[1]) - goal[1]), dtype=np.float64))
                last_scan = scan
            else:
                cmd = expert.update(pose, goal, last_scan, beam_angles,
                                    dt * sample_every)
                scan = last_scan
            sim.step(cmd, dt)
            p = sim.pose
            dist_goal = math.hypot(float(p[0]) - goal[0],
                                   float(p[1]) - goal[1])
            if dist_goal < 2.0 and abs(float(p[2]) - float(goal[2])) < 1.5:
                success = True
                break
        total_collisions += int(sim.collisions)
        total_nfz += int(sim.nfz_violations)
        episode_success.append(1.0 if success else 0.0)
        print("  episode %d/%d scene=%s success=%s collisions=%d nfz=%d "
              "time=%.1fs" % (ep + 1, episodes, scene_name, success,
                              sim.collisions, sim.nfz_violations, sim.time))

    out: Dict[str, np.ndarray] = {}
    for key, value in rows.items():
        if key == "images":
            out[key] = np.asarray(value, dtype=np.uint8)
        elif key == "actions":
            out[key] = np.asarray(value, dtype=np.int64)
        else:
            out[key] = np.asarray(value, dtype=np.float64)
    out["goal_xyz"] = np.asarray(sim.goal(), dtype=np.float64)
    out["goal"] = np.tile(np.asarray(sim.goal(), dtype=np.float64),
                          (len(out["pose"]), 1))
    out["scene"] = np.asarray(scene_name)
    out["collisions"] = np.asarray(total_collisions, dtype=np.int64)
    out["nfz_violations"] = np.asarray(total_nfz, dtype=np.int64)
    out["success"] = np.asarray(episode_success, dtype=np.float64)
    out["episodes"] = np.asarray(int(episodes), dtype=np.int64)
    out["sample_hz"] = np.asarray(float(sample_hz))
    out["seed"] = np.asarray(int(seed), dtype=np.int64)
    out["random_start"] = np.asarray(bool(random_start))
    return out


# ----------------------------------------------------------- data plumbing
def save_dataset(data: Dict[str, np.ndarray], path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, **data)


def load_collected(data_dir: Optional[str] = None,
                   scenes: Optional[Sequence[str]] = None
                   ) -> Dict[str, Dict[str, np.ndarray]]:
    data_dir = data_dir or C.RESULTS_DIR
    out: Dict[str, Dict[str, np.ndarray]] = {}
    for path in sorted(glob.glob(os.path.join(data_dir, "dataset_*.npz"))):
        name = os.path.basename(path)[len("dataset_"):-len(".npz")]
        if scenes is not None and name not in scenes:
            continue
        with np.load(path, allow_pickle=False) as data:
            payload = {k: data[k] for k in data.files}
        payload["scene"] = np.asarray(name)
        out[name] = payload
    return out


def dataset_metadata(data: Dict[str, np.ndarray]) -> Dict[str, object]:
    success = np.asarray(data["success"], dtype=np.float64)
    labels = np.asarray(data["labels"], dtype=np.float64)
    return {
        "scene": str(data["scene"]),
        "samples": int(len(data["pose"])),
        "episodes": int(data["episodes"]),
        "sample_hz": float(data["sample_hz"]),
        "seed": int(data["seed"]),
        "collisions": int(data["collisions"]),
        "nfz_violations": int(data["nfz_violations"]) if "nfz_violations" in data else 0,
        "episode_success_rate": float(success.mean()) if success.size else 0.0,
        "random_start": bool(data["random_start"]) if "random_start" in data else False,
        "beams": int(data["scans"].shape[1]),
        "scan_shape": list(data["scans"].shape),
        "clean_scan_shape": list(data["scans_clean"].shape),
        "image_shape": list(data["images"].shape),
        "feature_shape": list(data["obs"].shape),
        "cmd_shape": list(data["cmds"].shape),
        "label_mean": [round(float(v), 4) for v in labels.mean(axis=0)]
        if labels.size else [],
        "action_counts": {str(i): int((data["actions"] == i).sum())
                          for i in range(8)},
    }


def concat_datasets(datasets: Dict[str, Dict[str, np.ndarray]],
                    keys: Sequence[str]) -> Dict[str, np.ndarray]:
    """Concatenate per-scene datasets and add a globally unique episode id."""
    out: Dict[str, np.ndarray] = {}
    for key in keys:
        out[key] = np.concatenate([d[key] for d in datasets.values()], axis=0)
    episode = []
    for scene_idx, (name, data) in enumerate(sorted(datasets.items())):
        episode.append(np.asarray(data["episode"], dtype=np.int64)
                       + 1000 * scene_idx)
    out["episode"] = np.concatenate(episode, axis=0)
    return out


def split_by_episode(episode_ids: np.ndarray, val_fraction: float = 0.25,
                     seed: int = 0):
    """Return (train_mask, val_mask) holding out complete episodes."""
    uniq = np.unique(np.asarray(episode_ids, dtype=np.int64))
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    rng.shuffle(uniq)
    n_val = max(1, int(round(len(uniq) * float(val_fraction))))
    val_eps = set(int(e) for e in uniq[:n_val].tolist())
    mask = np.asarray([int(e) in val_eps for e in episode_ids], dtype=bool)
    return ~mask, mask, sorted(val_eps)


def load_training_data(data_dir: Optional[str] = None,
                       scenes: Sequence[str] = C.SCENES,
                       episodes: int = 2, seconds: float = 50.0,
                       seed: int = C.DEFAULT_SEED, force: bool = False,
                       keys: Optional[Sequence[str]] = None
                       ) -> Dict[str, np.ndarray]:
    """Ensure datasets exist and return one concatenated array dictionary."""
    datasets = ensure_datasets(data_dir=data_dir, scenes=scenes,
                               episodes=episodes, seconds=seconds,
                               seed=seed, force=force)
    if not datasets:
        raise RuntimeError("no dataset available")
    keys = keys or ["pose", "goal", "obs", "scans", "scans_clean", "images",
                    "cmds", "cmds_body", "labels", "actions", "episode",
                    "nfz"]
    return concat_datasets(datasets, keys)


def ensure_datasets(data_dir: Optional[str] = None,
                    scenes: Sequence[str] = C.SCENES,
                    episodes: int = 2, seconds: float = 50.0,
                    seed: int = C.DEFAULT_SEED,
                    force: bool = False,
                    random_start: bool = True
                    ) -> Dict[str, Dict[str, np.ndarray]]:
    """Collect any missing scene dataset and write per-scene metadata JSON."""
    data_dir = data_dir or C.RESULTS_DIR
    os.makedirs(data_dir, exist_ok=True)
    existing = load_collected(data_dir, scenes)
    missing = [s for s in scenes if s not in existing]
    if force:
        missing = list(scenes)
    for scene_name in missing:
        t0 = time.time()
        print("collecting scene %r (%d episode(s), %.0f s each)"
              % (scene_name, episodes, seconds))
        data = collect(scene_name, seconds=seconds, seed=seed,
                       noise=True, episodes=episodes,
                       random_start=random_start)
        save_dataset(data, os.path.join(data_dir,
                                        "dataset_%s.npz" % scene_name))
        meta = dataset_metadata(data)
        meta["seconds_wall"] = round(time.time() - t0, 2)
        with open(os.path.join(data_dir, "dataset_%s.json" % scene_name), "w",
                  encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2, sort_keys=True)
        existing[scene_name] = data
        print("  -> %d samples in %.1fs" % (meta["samples"],
                                            meta["seconds_wall"]))
    return existing
