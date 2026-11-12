"""Dataset collection and feature engineering for the GVNN-Road project.

The module has four jobs:

1. a fast, *exact* analytic replica of the road-vehicle lidar (72 beams,
   360 deg, 0.2 m ray marching against curbs / parked cars / barriers / cones /
   pedestrians) used to build noise-free targets for the denoising network;
2. feature builders shared by every training tool (18 lidar sectors with 4
   beams each, relative goal polar, current speed and traffic-light features);
3. a geometric expert (A* + pure pursuit + speed control + red-light stop +
   pedestrian yielding) that drives the MuJoCo-free bicycle simulator and
   produces supervised demonstrations, plus a *reactive* lane-following policy
   whose labels are a pure function of the features so behaviour cloning is
   well posed;
4. dataset plumbing (collect / save / load / split) and the discrete
   8-action road environment used by the RL comparison.

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

# Exactly the beam angles used by :class:`road_numpy_nn_slam.sim.RoadSim`.
_SCAN_ANGLES = np.linspace(-_PI, _PI, C.LIDAR_BEAMS, endpoint=False)
# Body-frame directions of the 8 discrete planner / RL actions.
_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (_PI / 4.0)

N_SECTORS = 18                        # 72 beams / 4 beams per sector
SECTOR_BEAMS = C.LIDAR_BEAMS // N_SECTORS
MARCH_STEP = 0.20                     # m, must match RoadSim.lidar()
MARCH_DISTS = np.arange(MARCH_STEP, C.LIDAR_RANGE_MAX, MARCH_STEP)
GRID_INFLATION = C.CAR_RADIUS + 0.40  # m, keep 0.40 m extra clearance from obstacles
LANE_MARGIN = 1.5                     # m, keep the lane-centred path off curbs
OFFROAD_COST = 6.0                    # A* cost multiplier off the road network
LANE_EDGE_COST = 2.5                  # A* cost multiplier near the road edge
CRUISE_SPEED = 0.72 * C.MAX_SPEED     # m/s, expert cruise speed on a clear road
LIGHT_STOP_DIST = 26.0                # m, start braking for a red light
LIGHT_HOLD = 1.5                      # m, stop line stand-off
LIGHT_STOPLINE = 5.0                  # m, the sim's stop line is 5 m before the light
PED_BRAKE_DIST = 14.0                 # m, pedestrian yield distance
PED_LATERAL = 3.6                     # m, pedestrian corridor half width


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
def _drivable_points(scene: Scene, px: np.ndarray, py: np.ndarray) -> np.ndarray:
    """Vectorised ``scene.is_drivable`` for arrays of points."""
    ok = np.zeros(px.shape, dtype=bool)
    for cx, cy, hw, hh in scene.roads:
        ok |= (np.abs(px - cx) <= hw) & (np.abs(py - cy) <= hh)
    return ok


def _obstacle_points(scene: Scene, px: np.ndarray, py: np.ndarray,
                     t: float = 0.0) -> np.ndarray:
    """Vectorised ``_point_in_obstacles`` (parked, barriers, cones, peds)."""
    hit = np.zeros(px.shape, dtype=bool)
    for cx, cy, hw, hh in scene.parked:
        hit |= (np.abs(px - cx) <= hw) & (np.abs(py - cy) <= hh)
    for cx, cy, hw, hh in scene.barriers:
        hit |= (np.abs(px - cx) <= hw) & (np.abs(py - cy) <= hh)
    for cx, cy, r in scene.cones:
        hit |= (px - cx) ** 2 + (py - cy) ** 2 <= r * r
    for cx, cy in scene.pedestrian_positions(t):
        hit |= (px - cx) ** 2 + (py - cy) ** 2 <= 0.45 ** 2
    return hit


def raycast_scan(pose: Sequence[float], scene: Scene,
                 angles: Optional[np.ndarray] = None,
                 t: float = 0.0) -> np.ndarray:
    """Noise-free lidar scan identical to :meth:`RoadSim.lidar` (noise off).

    The simulator marches in 0.2 m steps from 0.2 m up to the maximum range
    and stops at the first curb / obstacle point; this function evaluates all
    72 x 199 samples at once with NumPy, which is ~100x faster than the
    per-beam Python loop and reproduces it exactly.
    """
    angles = _SCAN_ANGLES if angles is None else np.asarray(angles, dtype=np.float64)
    yaw = float(pose[3])
    rel = yaw + angles
    dx, dy = np.cos(rel), np.sin(rel)
    px = float(pose[0]) + np.outer(dx, MARCH_DISTS)
    py = float(pose[1]) + np.outer(dy, MARCH_DISTS)
    blocked = (~_drivable_points(scene, px, py)) | \
        _obstacle_points(scene, px, py, t)
    any_hit = blocked.any(axis=1)
    idx = blocked.argmax(axis=1)
    ranges = np.where(any_hit, MARCH_DISTS[idx], C.LIDAR_RANGE_MAX)
    return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)


def lidar_noise_scan(clean: np.ndarray,
                     rng: np.random.Generator) -> np.ndarray:
    """Apply the exact :meth:`RoadSim.lidar` noise model to a clean scan."""
    ranges = np.asarray(clean, dtype=np.float64).copy()
    ranges = ranges * (1.0 + 0.01 * rng.normal(size=ranges.shape))
    ranges += 0.03 * rng.normal(size=ranges.shape)
    outlier = rng.random(ranges.shape) < 0.04
    ranges[outlier] = rng.uniform(2.0, C.LIDAR_RANGE_MAX,
                                  size=int(outlier.sum()))
    dropout = rng.random(ranges.shape) < 0.03
    ranges[dropout] = C.LIDAR_RANGE_MAX
    return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)


# Kept for compatibility with the shared ROV / UAV code.
sonar_noise_scan = lidar_noise_scan


def _erode_bool(grid: np.ndarray, cells: int) -> np.ndarray:
    """Erode a boolean mask by a square structuring element."""
    if cells <= 0:
        return grid.copy()
    padded = np.pad(grid, cells, mode="constant", constant_values=False)
    out = np.ones_like(grid)
    h, w = grid.shape
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            out &= padded[cells + dr:cells + dr + h,
                          cells + dc:cells + dc + w]
    return out


def _planning_layers(scene: Scene, resolution: float,
                     inflate_radius: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(blocked, cost)`` grids for the planner / sampling.

    ``blocked`` covers parked cars, barriers and cones inflated by the vehicle
    radius (curbs are *not* blocking: the simulator allows off-road driving
    with extra drag, and the construction scene can only be passed that way).
    The cost map is 1.0 in the lane interior, ``LANE_EDGE_COST`` in the band
    next to a curb (which keeps the path centred on the road) and
    ``OFFROAD_COST`` off the road network.
    """
    w = int(round(scene.width / resolution))
    h = int(round(scene.height / resolution))
    xs = (np.arange(w) + 0.5) * resolution
    ys = (np.arange(h) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    drivable = _drivable_points(scene, gx, gy)
    blocked = np.zeros((h, w), dtype=bool)
    for cx, cy, hw, hh in list(scene.parked) + list(scene.barriers):
        blocked |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
    for cx, cy, r in scene.cones:
        blocked |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
    margin = 0.5
    blocked |= (gx < margin) | (gx > scene.width - margin) | \
        (gy < margin) | (gy > scene.height - margin)
    cells = int(round(float(inflate_radius) / float(resolution)))
    if cells > 0:
        padded = np.pad(blocked, cells, mode="constant", constant_values=True)
        out = np.zeros_like(blocked)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                out |= padded[cells + dr:cells + dr + h,
                              cells + dc:cells + dc + w]
        blocked = out
    interior = _erode_bool(drivable, int(round(LANE_MARGIN / resolution)))
    cost = np.where(drivable,
                    np.where(interior, 1.0, LANE_EDGE_COST),
                    OFFROAD_COST)
    return blocked, cost


def build_planning_grid(scene: Scene, resolution: float = C.PLAN_RESOLUTION,
                        inflate_radius: float = GRID_INFLATION) -> np.ndarray:
    """Blocked grid (True = blocked) for A*, sampling and the SLAM metrics."""
    blocked, _cost = _planning_layers(scene, resolution, inflate_radius)
    return blocked


def build_cost_map(scene: Scene, resolution: float = C.PLAN_RESOLUTION,
                   inflate_radius: float = GRID_INFLATION) -> np.ndarray:
    """Per-cell traversal cost (1.0 on road, OFFROAD_COST off road)."""
    _blocked, cost = _planning_layers(scene, resolution, inflate_radius)
    return cost


def augment_scans(scenes: Sequence[str], n_per_scene: int = 2000,
                  seed: int = C.DEFAULT_SEED,
                  resolution: float = C.PLAN_RESOLUTION):
    """Synthesise extra noisy / clean scan pairs from random drivable poses."""
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
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
            pose = (x, y, 0.0, float(rng.uniform(-_PI, _PI)))
            t = float(rng.uniform(0.0, C.LIGHT_CYCLE))
            cl = raycast_scan(pose, scene, t=t)
            clean.append(cl)
            noisy.append(lidar_noise_scan(cl, rng))
    return (np.asarray(noisy, dtype=np.float64),
            np.asarray(clean, dtype=np.float64))


class AStarPlanner:
    """8-connected weighted A* on a boolean occupancy grid."""

    def __init__(self, grid: np.ndarray, resolution: float = C.PLAN_RESOLUTION,
                 cost: Optional[np.ndarray] = None):
        self.grid = np.asarray(grid, dtype=bool)
        self.resolution = float(resolution)
        self.cost = (np.ones_like(self.grid, dtype=np.float64)
                     if cost is None else np.asarray(cost, dtype=np.float64))
        self.h, self.w = self.grid.shape

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int(math.floor(float(x) / self.resolution))
        r = int(math.floor(float(y) / self.resolution))
        return max(0, min(self.h - 1, r)), max(0, min(self.w - 1, c))

    def cell_to_world(self, r: int, c: int) -> Tuple[float, float]:
        return grid_to_world(int(c), int(r), self.resolution)

    def is_free(self, x: float, y: float) -> bool:
        r, c = self.world_to_cell(x, y)
        return not bool(self.grid[r, c])

    def nearest_free(self, r: int, c: int,
                     max_radius: int = 60) -> Optional[Tuple[int, int]]:
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

    def plan(self, start_xy: Sequence[float], goal_xy: Sequence[float],
             max_expansions: int = 600000
             ) -> Optional[List[Tuple[float, float]]]:
        sr, sc = self.world_to_cell(start_xy[0], start_xy[1])
        gr, gc = self.world_to_cell(goal_xy[0], goal_xy[1])
        start = self.nearest_free(sr, sc)
        goal = self.nearest_free(gr, gc)
        if start is None or goal is None:
            return None
        start, goal = tuple(start), tuple(goal)
        if start == goal:
            return [self.cell_to_world(start[0], start[1])]
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
            for dr, dc, step in moves:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.h and 0 <= nc < self.w):
                    continue
                if self.grid[nr, nc]:
                    continue
                if dr != 0 and dc != 0:
                    if self.grid[r + dr, c] or self.grid[r, c + dc]:
                        continue
                ng = base + step * 0.5 * (self.cost[r, c] + self.cost[nr, nc])
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
        return [self.cell_to_world(r, c) for r, c in cells]


# ---------------------------------------------------------------- features
def scan_features(scans: np.ndarray) -> np.ndarray:
    """Normalise lidar ranges (metres) to [0, 1]."""
    s = np.asarray(scans, dtype=np.float64)
    span = C.LIDAR_RANGE_MAX - C.LIDAR_RANGE_MIN
    return np.clip((s - C.LIDAR_RANGE_MIN) / span, 0.0, 1.0)


def sector_stats(scans: np.ndarray, n_sectors: int = N_SECTORS,
                 stats: Sequence[str] = ("min", "mean")) -> np.ndarray:
    """Per-sector statistics of the normalised lidar scan (18 x 2 = 36)."""
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
    """Relative goal polar features: [dist, sin(bearing), cos(bearing)]."""
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
    feats = np.stack([np.clip(dist / 200.0, 0.0, 1.0),
                      np.sin(bearing), np.cos(bearing)], axis=1)
    return feats[0] if single else feats


def light_features(pose: np.ndarray, scene: Optional[Scene],
                   t: float = 0.0) -> np.ndarray:
    """Nearest traffic light ahead: [distance/40, green, yellow, red] (4 dims).

    All zeros when no light is on the current approach, so the network learns a
    "no light -> keep going" pattern.
    """
    p = np.asarray(pose, dtype=np.float64)
    single = p.ndim == 1
    p = np.atleast_2d(p)
    out = np.zeros((p.shape[0], 4), dtype=np.float64)
    if scene is not None and scene.lights:
        for i in range(p.shape[0]):
            light = scene.nearest_light(float(p[i, 0]), float(p[i, 1]),
                                        float(p[i, 3]),
                                        float(t) if np.isscalar(t) else float(t[i]))
            if light is None:
                continue
            out[i, 0] = float(np.clip(light["distance"] / 40.0, 0.0, 1.0))
            state = light.get("state", "green")
            out[i, 1] = 1.0 if state == "green" else 0.0
            out[i, 2] = 1.0 if state == "yellow" else 0.0
            out[i, 3] = 1.0 if state == "red" else 0.0
    return out[0] if single else out


CONTROL_DIM = 2 * N_SECTORS + 3 + 1 + 4      # 44 dims


def control_features(pose: np.ndarray, goal: np.ndarray, scan: np.ndarray,
                     scene: Optional[Scene] = None, speed: float = 0.0,
                     t: float = 0.0) -> np.ndarray:
    """Full control-MLP input vector (44 dims for the default layout)."""
    p = np.asarray(pose, dtype=np.float64)
    single = p.ndim == 1
    p2 = np.atleast_2d(p)
    s = np.asarray(scan, dtype=np.float64)
    if s.ndim == 1:
        s = s[None, :]
    sectors = sector_stats(s, N_SECTORS, ("min", "mean"))
    gf = goal_features(p2, goal)
    sp = np.clip(np.asarray(speed, dtype=np.float64) / C.MAX_SPEED, -1.0, 1.0)
    sp = np.full((p2.shape[0], 1), float(sp)) if sp.ndim == 0 else \
        sp.reshape(-1, 1)
    lf = light_features(p2, scene, t)
    out = np.concatenate([sectors, gf, sp, lf], axis=1)
    return out[0] if single else out


def obs_features(pose: np.ndarray, goal: Sequence[float]) -> np.ndarray:
    """Pose/goal observation vector: goal polar + heading sin/cos (5 dims)."""
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
    return discretize_heading(float(angle), 8)


# ------------------------------------------------------ reactive expert
def _interp_clear(ranges: np.ndarray, angle: float,
                  ang: np.ndarray) -> float:
    """Circular interpolation of the sector clearance at a relative bearing."""
    ang_ext = np.concatenate([ang, ang + _TWO_PI])
    r_ext = np.concatenate([ranges, ranges])
    return float(np.interp(float(angle), ang_ext, r_ext))


def _light_speed_limit(d_eff: float) -> float:
    """Comfortable braking curve (4 m/s^2) towards a stop line."""
    return math.sqrt(max(0.0, 2.0 * 4.0 * max(0.0, float(d_eff) - LIGHT_HOLD)))


def reactive_action(pose: Sequence[float], goal: Sequence[float],
                    scan: np.ndarray, scene: Optional[Scene] = None,
                    speed: float = 0.0, t: float = 0.0,
                    n_sectors: int = N_SECTORS) -> np.ndarray:
    """Reactive lane-following / speed policy, a pure function of the features.

    Returns ``(throttle, steer)`` in [-1, 1] (steer > 0 = left).  The policy
    only uses quantities that are also part of :func:`control_features`
    (sector minima/means, relative goal bearing, current speed, nearest-light
    distance and state), which makes the behaviour-cloning labels learnable.
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
    clear_fwd = _interp_clear(ranges, bearing, ang)
    clear_left = _interp_clear(ranges, bearing + 0.80, ang)
    clear_right = _interp_clear(ranges, bearing - 0.80, ang)
    asym = float(np.clip((clear_left - clear_right) / 12.0, -1.0, 1.0))
    blocked = float(np.clip(1.0 - clear_fwd / 10.0, 0.0, 1.0))

    # keep the vehicle centred in narrow passages: steer away from the closer
    # side and slow down when both sides are close
    steer = 1.55 * bearing + 0.90 * asym * max(0.35, blocked)
    steer = float(np.clip(steer, -1.0, 1.0))
    corridor = min(clear_left, clear_right)

    # ---- target speed from the corridor ahead and the cornering demand
    v_target = CRUISE_SPEED * float(np.clip((clear_fwd - 1.5) / 12.0,
                                            0.12, 1.0))
    v_target *= 1.0 - 0.55 * min(1.0, abs(steer))
    if corridor < 3.5:
        v_target = min(v_target, 1.6)
    if dist_goal < 6.0:
        v_target *= float(np.clip(dist_goal / 6.0, 0.0, 1.0))

    # ---- traffic light: stop before the stop line on red, prepare on yellow
    if scene is not None and scene.lights:
        light = scene.nearest_light(float(p[0]), float(p[1]), yaw, float(t))
        if light is not None:
            d_eff = float(light["distance"]) - LIGHT_STOPLINE
            state = light.get("state", "green")
            if state == "red" and d_eff > 0.0:
                v_target = min(v_target, _light_speed_limit(d_eff))
            elif state == "yellow" and d_eff > 0.0:
                # only stop on yellow if the stop line cannot be reached
                # comfortably within the remaining yellow phase
                if d_eff > 0.55 * float(speed) + 1.0:
                    v_target = min(v_target, _light_speed_limit(d_eff))

    throttle = float(np.clip((v_target - float(speed)) / 2.5, -1.0, 1.0))
    return np.array([throttle, steer], dtype=np.float64)


def reactive_commands(pose: np.ndarray, goal: np.ndarray, scans: np.ndarray,
                      speeds: Optional[np.ndarray] = None,
                      scene: Optional[Scene] = None,
                      times: Optional[np.ndarray] = None) -> np.ndarray:
    """Reactive expert commands for a batch of states (T x 2)."""
    pose = np.atleast_2d(np.asarray(pose, dtype=np.float64))
    scans = np.asarray(scans, dtype=np.float64)
    goal_arr = np.asarray(goal, dtype=np.float64)
    if goal_arr.ndim == 1:
        goal_arr = np.repeat(goal_arr.reshape(1, 3), len(pose), axis=0)
    speeds = (np.zeros(len(pose)) if speeds is None
              else np.asarray(speeds, dtype=np.float64).reshape(-1))
    times = (np.zeros(len(pose)) if times is None
             else np.asarray(times, dtype=np.float64).reshape(-1))
    out = np.zeros((len(pose), 2), dtype=np.float64)
    for i in range(len(pose)):
        out[i] = reactive_action(pose[i], goal_arr[i], scans[i], scene,
                                 speeds[i], times[i])
    return out


def reactive_labels(pose: np.ndarray, goal: np.ndarray, scans: np.ndarray,
                    speeds: np.ndarray, scene: Optional[Scene] = None,
                    times: Optional[np.ndarray] = None) -> np.ndarray:
    """Normalised control labels ``(throttle, steer)`` (already in [-1, 1])."""
    return reactive_commands(pose, goal, scans, speeds, scene, times)


def augment_control(scenes: Sequence[str], n_per_scene: int = 2000,
                    seed: int = C.DEFAULT_SEED,
                    resolution: float = C.PLAN_RESOLUTION):
    """Synthetic road states labelled by the reactive expert.

    Returns ``(features, control_labels, action_labels)``.
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
        for _ in range(int(n_per_scene)):
            r, c = free[int(rng.integers(len(free)))]
            x, y = grid_to_world(int(c), int(r), resolution)
            yaw = float(rng.uniform(-_PI, _PI))
            pose = np.array([x, y, 0.0, yaw])
            t = float(rng.uniform(0.0, C.LIGHT_CYCLE))
            speed = float(rng.uniform(0.0, C.MAX_SPEED * 0.9))
            clean = raycast_scan(pose, scene, t=t)
            scan = lidar_noise_scan(clean, rng)
            body = reactive_action(pose, goal, scan, scene, speed, t)
            xs.append(control_features(pose, goal, scan, scene, speed, t))
            ys.append(body)
            acts.append(quantize_direction(math.atan2(
                math.sin(body[1] * C.MAX_STEER), math.cos(body[1] * C.MAX_STEER))))
    return (np.asarray(xs, dtype=np.float64),
            np.asarray(ys, dtype=np.float64),
            np.asarray(acts, dtype=np.int64))


# ------------------------------------------------------------------- expert
class GeometricExpert:
    """Deterministic A* + pure-pursuit + speed-control expert controller.

    Adds the road-specific rules the A* path alone does not know about:
    stopping at red lights, yielding to pedestrians and a lidar-based safety
    slowdown.  This is the fallback expert used for behaviour cloning.
    """

    kind = "geometric_expert"

    def __init__(self, scene_name: str, resolution: float = C.PLAN_RESOLUTION):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.resolution = float(resolution)
        self.grid = build_planning_grid(self.scene, self.resolution)
        self.cost = build_cost_map(self.scene, self.resolution)
        self.planner = AStarPlanner(self.grid, self.resolution, self.cost)
        self.path: Optional[List[Tuple[float, float]]] = None
        self.path_arr = np.zeros((0, 2))
        self.path_length = 0.0
        self.goal = np.asarray(self.scene.goal, dtype=np.float64)
        self.last_info: Dict[str, float] = {}
        self.replans = 0
        self.v = 0.0
        self._target_v = 0.0
        self._prev_v = 0.0
        self._stuck = 0.0
        self._reverse = 0.0
        self.reset()

    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y = self.scene.start[0], self.scene.start[1]
        else:
            x, y = float(pose[0]), float(pose[1])
        self.v = 0.0
        self._prev_v = 0.0
        self._stuck = 0.0
        self._reverse = 0.0
        self.replans = 0
        self._plan(x, y)

    def set_speed(self, v: float) -> None:
        """Feed the measured longitudinal speed (used by the speed loop)."""
        self._prev_v = self.v
        self.v = float(v)

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
        if float(d.min()) > 3.5:
            self._plan(x, y)

    def _lookahead_target(self, x: float, y: float,
                          lookahead: float) -> Tuple[float, float]:
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
               dt: float = 1.0 / C.CONTROL_HZ, t: float = 0.0) -> np.ndarray:
        x, y, _z, yaw = [float(v) for v in pose[:4]]
        goal = np.asarray(goal, dtype=np.float64)
        self.goal = goal
        self._replan_if_needed(x, y)
        dist_goal = math.hypot(float(goal[0]) - x, float(goal[1]) - y)
        lookahead = float(np.clip(1.5 + 0.55 * self.v, 2.0, 7.0))
        tx, ty = self._lookahead_target(x, y, lookahead)
        if dist_goal < lookahead:
            tx, ty = float(goal[0]), float(goal[1])
        desired = math.atan2(ty - y, tx - x)
        err = wrap_angle(desired - yaw)
        steer = float(np.clip(1.55 * err, -1.0, 1.0))

        # ---- slow down before sharp corners (path curvature ahead)
        turn_ahead = 0.0
        if len(self.path_arr) > 3:
            dpath = np.hypot(self.path_arr[:, 0] - x, self.path_arr[:, 1] - y)
            i = int(np.argmin(dpath))
            j = min(len(self.path_arr) - 1,
                    i + max(2, int(round(10.0 / self.resolution))))
            if j > i + 1:
                a0 = math.atan2(self.path_arr[i + 1, 1] - self.path_arr[i, 1],
                                self.path_arr[i + 1, 0] - self.path_arr[i, 0])
                a1 = math.atan2(self.path_arr[j, 1] - self.path_arr[j - 1, 1],
                                self.path_arr[j, 0] - self.path_arr[j - 1, 0])
                turn_ahead = abs(wrap_angle(a1 - a0))

        # ---- narrow-passage handling: centre between the two obstacles
        corridor = float(C.LIDAR_RANGE_MAX)
        if scan is not None:
            sc = np.asarray(scan, dtype=np.float64)
            ang = (_SCAN_ANGLES if beam_angles is None
                   else np.asarray(beam_angles, dtype=np.float64))
            left_cone = (ang > 0.45) & (ang < 1.20)
            right_cone = (ang < -0.45) & (ang > -1.20)
            lm = float(sc[left_cone].min()) if bool(left_cone.any()) else C.LIDAR_RANGE_MAX
            rm = float(sc[right_cone].min()) if bool(right_cone.any()) else C.LIDAR_RANGE_MAX
            corridor = min(lm, rm)
            if corridor < 3.4:
                centre_bias = float(np.clip((lm - rm) / 4.0, -1.0, 1.0))
                steer = float(np.clip(steer + 0.55 * centre_bias, -1.0, 1.0))

        # ---- speed target: cruise, slow for corners / goal / lidar
        v_target = CRUISE_SPEED * (1.0 - 0.55 * min(1.0, abs(steer)))
        if turn_ahead > 0.55:
            v_target = min(v_target,
                           max(1.1, 3.0 - 1.8 * (turn_ahead - 0.55)))
        if corridor < 3.4:
            v_target = min(v_target, 1.6)
        if dist_goal < 8.0:
            v_target *= float(np.clip(dist_goal / 8.0, 0.0, 1.0))
        # ---- curb guard: off-road drag equals the maximum acceleration, so a
        # car that leaves the carriageway can never drive back: never let it.
        for ahead in (2.0,):
            ax = x + ahead * math.cos(yaw)
            ay = y + ahead * math.sin(yaw)
            if not self.scene.is_drivable(ax, ay):
                v_target = min(v_target, 0.8)
                break
        if scan is not None:
            ang = (_SCAN_ANGLES if beam_angles is None
                   else np.asarray(beam_angles, dtype=np.float64))
            travel = err
            cone = np.abs(_wrap_vec(ang - travel)) <= 0.35
            if bool(cone.any()):
                front = float(np.asarray(scan, dtype=np.float64)[cone].min())
                if front < 1.10:
                    v_target = 0.0
                elif front < 9.0:
                    v_target = min(v_target, max(1.2, (front - 1.0) * 1.5))
        # ---- traffic lights (stop line is 5 m before the light centre)
        light = self.scene.nearest_light(x, y, yaw, float(t))
        if light is not None:
            d_eff = float(light["distance"]) - LIGHT_STOPLINE
            state = light.get("state", "green")
            if state == "red" and d_eff > 0.0:
                v_target = min(v_target, _light_speed_limit(d_eff))
            elif state == "yellow" and d_eff > 0.0:
                if d_eff > 0.55 * self.v + 1.0:
                    v_target = min(v_target, _light_speed_limit(d_eff))
        # ---- pedestrian yielding: stop well before a crossing pedestrian and
        # back away if one gets too close (the simulator counts a hit on every
        # physics step of overlap)
        ped_near = False
        for px, py in self.scene.pedestrian_positions(float(t)):
            dxp, dyp = float(px) - x, float(py) - y
            d_ped = math.hypot(dxp, dyp)
            forward = dxp * math.cos(yaw) + dyp * math.sin(yaw)
            lateral = -dxp * math.sin(yaw) + dyp * math.cos(yaw)
            if d_ped < 3.0:
                ped_near = True
            # yield: slow down smoothly for a pedestrian near the carriageway
            if 0.0 < forward < PED_BRAKE_DIST and abs(lateral) < PED_LATERAL:
                v_target = min(v_target, max(0.0, (forward - 2.0) * 0.7))
        if ped_near:
            self._reverse = max(self._reverse, 0.8)
        # ---- stuck recovery: back away briefly when the corridor is jammed
        if self._reverse > 0.0:
            self._reverse = max(0.0, self._reverse - dt)
            self.last_info = {"heading_err": float(err),
                              "steer": float(-steer),
                              "target_speed": -1.0,
                              "dist_goal": float(dist_goal)}
            return np.array([-0.8, float(np.clip(-steer, -1.0, 1.0))],
                            dtype=np.float64)
        if self._target_v >= 1.0 and self.v < 0.25:
            self._stuck += dt
            if self._stuck > 1.5:
                self._reverse = 1.2
                self._stuck = 0.0
        else:
            self._stuck = max(0.0, self._stuck - dt)
        # ---- longitudinal control
        throttle = float(np.clip((v_target - self.v) / 2.5, -1.0, 1.0))
        self._target_v = float(v_target)
        self.last_info = {"heading_err": float(err), "steer": float(steer),
                          "target_speed": float(v_target),
                          "dist_goal": float(dist_goal)}
        return np.array([throttle, steer], dtype=np.float64)


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
    """Best-effort construction of the concurrent MissionController."""
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
        "replan_interval": 10.0,
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
class DiscreteRoadEnv:
    """Discrete-action abstraction of the road navigation task.

    The road vehicle is driven with one of 8 (throttle, steer) primitives for
    ``dt`` seconds using the same bicycle model as the simulator, on the same
    scene geometry (roads, parked cars, cones, barriers, traffic lights).
    Running a red light, hitting a pedestrian, leaving the road and colliding
    with obstacles are all penalised and counted, which keeps RL training fast
    while staying meaningful.
    """

    ACTION_TABLE = np.array([
        [1.00, 0.00],   # accelerate straight
        [1.00, 0.35],   # accelerate left
        [1.00, -0.35],  # accelerate right
        [0.50, 0.70],   # turn left
        [0.50, -0.70],  # turn right
        [0.30, 0.00],   # cruise straight
        [0.00, 0.00],   # brake
        [-0.50, 0.00],  # reverse
    ], dtype=np.float64)

    def __init__(self, scene_name: str = "downtown",
                 max_steps: int = 80, goal_radius: float = 3.0,
                 resolution: float = 0.5, dt: float = 0.5,
                 random_start: bool = True, seed: int = 2026):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.resolution = float(resolution)
        self.dt = float(dt)
        self.blocked = build_planning_grid(self.scene, resolution,
                                           inflate_radius=C.CAR_RADIUS)
        self.goal_xy = np.array(self.scene.goal[:2], dtype=np.float64)
        self.start_pose = np.array(self.scene.start, dtype=np.float64)
        self.max_steps = int(max_steps)
        self.goal_radius = float(goal_radius)
        self.random_start = bool(random_start)
        self.rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        self._offsets = np.arange(-4.0, 4.01, 1.0)     # 9 values, 1 m apart
        self.state_dim = len(self._offsets) ** 2 + 3 + 1 + 4
        self.n_actions = 8
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.delta = 0.0
        self.time = 0.0
        self.steps = 0
        self.collisions = 0
        self.red_light_violations = 0
        self.pedestrians_hit = 0
        self.prev_dist = 0.0
        self.done = False
        self._prev_light_dist: Dict[int, float] = {}
        self._passed_lights: Dict[int, float] = {}

    # -------------------------------------------------------------- helpers
    def _cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int(math.floor(x / self.resolution))
        r = int(math.floor(y / self.resolution))
        c = max(0, min(self.blocked.shape[1] - 1, c))
        r = max(0, min(self.blocked.shape[0] - 1, r))
        return r, c

    def blocked_at(self, x: float, y: float) -> bool:
        r, c = self._cell(x, y)
        return bool(self.blocked[r, c])

    def observation(self) -> np.ndarray:
        ox = self.x + self._offsets
        oy = self.y + self._offsets
        cols = np.clip((ox / self.resolution).astype(np.int64), 0,
                       self.blocked.shape[1] - 1)
        rows = np.clip((oy / self.resolution).astype(np.int64), 0,
                       self.blocked.shape[0] - 1)
        patch = self.blocked[np.ix_(rows, cols)].astype(np.float64).reshape(-1)
        dx = self.goal_xy[0] - self.x
        dy = self.goal_xy[1] - self.y
        dist = math.hypot(dx, dy)
        rel = wrap_angle(math.atan2(dy, dx) - self.yaw)
        goal = np.array([np.clip(dist / 150.0, 0.0, 1.0),
                         math.sin(rel), math.cos(rel)])
        speed = np.array([np.clip(self.v / C.MAX_SPEED, -1.0, 1.0)])
        light = light_features(np.array([self.x, self.y, 0.0, self.yaw]),
                               self.scene, self.time)
        return np.concatenate([patch, goal, speed, light], axis=0)

    def sample_free_start(self) -> np.ndarray:
        h, w = self.blocked.shape
        for _ in range(1200):
            r = int(self.rng.integers(1, h - 1))
            c = int(self.rng.integers(1, w - 1))
            if self.blocked[r, c]:
                continue
            x, y = grid_to_world(c, r, self.resolution)
            if not self.scene.is_drivable(x, y):
                continue
            d = math.hypot(self.goal_xy[0] - x, self.goal_xy[1] - y)
            if 10.0 <= d <= 45.0:
                yaw = math.atan2(self.goal_xy[1] - y,
                                 self.goal_xy[0] - x) + float(self.rng.normal(0, 0.3))
                return np.array([x, y, 0.0, wrap_angle(yaw)])
        return self.start_pose.copy()

    def reset(self) -> np.ndarray:
        pose = self.sample_free_start() if self.random_start else self.start_pose
        self.x, self.y, self.yaw = float(pose[0]), float(pose[1]), float(pose[3])
        self.v = 0.0
        self.delta = 0.0
        self.time = 0.0
        self.steps = 0
        self.collisions = 0
        self.red_light_violations = 0
        self.pedestrians_hit = 0
        self.done = False
        self._prev_light_dist = {}
        self._passed_lights = {}
        self.prev_dist = float(math.hypot(self.goal_xy[0] - self.x,
                                          self.goal_xy[1] - self.y))
        return self.observation()

    def _light_violation(self, nx: float, ny: float) -> bool:
        if not self.scene.lights:
            return False
        violated = False
        for idx, light in enumerate(self.scene.lights):
            lx, ly = float(light["x"]), float(light["y"])
            axis = str(light.get("axis", "x"))
            direction = float(light.get("dir", 1.0))
            if axis == "x":
                if abs(ny - ly) > 3.5:
                    continue
                d = (lx - nx) * direction - 5.0
            else:
                if abs(nx - lx) > 3.5:
                    continue
                d = (ly - ny) * direction - 5.0
            prev = self._prev_light_dist.get(idx, d)
            state = self.scene.light_state(light, self.time)
            if prev > 0.0 >= d and state == "red":
                if self.time - self._passed_lights.get(idx, -99.0) > 4.0:
                    self.red_light_violations += 1
                    self._passed_lights[idx] = self.time
                    violated = True
            self._prev_light_dist[idx] = d
        return violated

    def step(self, action: int):
        self.steps += 1
        throttle, steer = self.ACTION_TABLE[int(action) % 8]
        # steering actuator rate limit
        target_delta = steer * C.MAX_STEER
        step = C.STEER_RATE * self.dt
        self.delta = getattr(self, "delta", 0.0)
        self.delta += float(np.clip(target_delta - self.delta, -step, step))
        accel = throttle * C.MAX_ACCEL
        drag = C.DRAG * self.v * abs(self.v) + C.ROLLING * (
            1.0 if self.v > 0 else -1.0)
        offroad = 0.0
        if not self.scene.is_drivable(self.x, self.y):
            offroad = C.OFFROAD_DRAG
        self.v = float(np.clip(self.v + (accel - drag) * self.dt
                               - offroad * self.dt * (1.0 if self.v > 0 else -1.0),
                               -C.MAX_REVERSE, C.MAX_SPEED))
        if throttle <= 0 and self.v < 0.25:
            self.v = 0.0
        yaw_rate = (self.v / C.WHEELBASE) * math.tan(self.delta) / (
            1.0 + C.UNDERSTEER * self.v * self.v)
        self.yaw = wrap_angle(self.yaw + yaw_rate * self.dt)
        nx = self.x + self.v * math.cos(self.yaw) * self.dt
        ny = self.y + self.v * math.sin(self.yaw) * self.dt
        self.time += self.dt

        reward = -0.02
        hit = self._check_collision(nx, ny)
        if hit is not None:
            if hit == "pedestrian":
                self.pedestrians_hit += 1
                reward -= 3.0
            self.collisions += 1
            reward -= 1.5
            self.v = 0.0
            nx, ny = self.x, self.y
        if self._light_violation(nx, ny):
            reward -= 2.0
        self.x, self.y = nx, ny
        offroad_now = not self.scene.is_drivable(self.x, self.y)
        if offroad_now:
            reward -= 0.02
        dist = float(math.hypot(self.goal_xy[0] - self.x,
                                self.goal_xy[1] - self.y))
        reward += 2.0 * (self.prev_dist - dist)
        self.prev_dist = dist
        success = dist <= self.goal_radius
        if success:
            reward += 12.0
            self.done = True
        elif self.steps >= self.max_steps:
            self.done = True
        info = {"success": bool(success), "collisions": self.collisions,
                "red_light_violations": self.red_light_violations,
                "pedestrians_hit": self.pedestrians_hit,
                "off_road": bool(offroad_now), "distance": dist}
        return self.observation(), float(reward), bool(self.done), info

    def _check_collision(self, x: float, y: float) -> Optional[str]:
        r = C.CAR_RADIUS
        for cx, cy, hw, hh in self.scene.parked:
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= r * r:
                return "parked"
        for cx, cy, hw, hh in self.scene.barriers:
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= r * r:
                return "barrier"
        for cx, cy, cr in self.scene.cones:
            if (x - cx) ** 2 + (y - cy) ** 2 <= (r + cr) ** 2:
                return "cone"
        for px, py in self.scene.pedestrian_positions(self.time):
            if (x - px) ** 2 + (y - py) ** 2 <= (r + 0.45) ** 2:
                return "pedestrian"
        return None


# Alias kept so the ROV/UAV-era scripts keep working.
DiscretePoolEnv = DiscreteRoadEnv


def _jitter_start(scene: Scene, grid: np.ndarray, rng: np.random.Generator,
                  radius: float = 5.0, resolution: float = C.PLAN_RESOLUTION
                  ) -> Tuple[float, float, float, float]:
    """Small, safe offset from the scene start pose (same road, same heading).

    Random mid-route starts are avoided on purpose: leaving the carriageway is
    unrecoverable in this simulator (off-road drag equals the maximum
    acceleration), so an episode should always begin on a valid lane pose.
    """
    sx, sy = float(scene.start[0]), float(scene.start[1])
    for _ in range(80):
        x = sx + float(rng.uniform(-radius, radius))
        y = sy + float(rng.uniform(-radius, radius))
        if not scene.is_drivable(x, y):
            continue
        r = min(max(int(y / resolution), 0), grid.shape[0] - 1)
        c = min(max(int(x / resolution), 0), grid.shape[1] - 1)
        if grid[r, c]:
            continue
        yaw = wrap_angle(float(scene.start[3])
                         + float(rng.uniform(-0.18, 0.18)))
        return x, y, 0.0, yaw
    return tuple(float(v) for v in scene.start)


def sample_free_pose(scene: Scene, grid: np.ndarray,
                     rng: np.random.Generator,
                     resolution: float = C.PLAN_RESOLUTION,
                     goal_xy: Optional[Sequence[float]] = None,
                     yaw_spread: float = 0.35
                     ) -> Tuple[float, float, float, float]:
    """Sample a collision-free pose (x, y, 0, yaw) on the road network.

    The heading is aligned with the road axis at the sampled point (with a
    small random offset), so an episode never starts with the car pointing
    across the carriageway.
    """
    drivable = scene.drivable_mask(resolution)
    sample_grid = np.asarray(grid, dtype=bool) | (~drivable)
    free = np.argwhere(~sample_grid)
    if len(free) == 0:
        return tuple(float(v) for v in scene.start)
    r, c = free[int(rng.integers(len(free)))]
    x, y = grid_to_world(int(c), int(r), resolution)
    axis_yaw = 0.0
    best = None
    for cx, cy, hw, hh in scene.roads:
        if abs(x - cx) <= hw and abs(y - cy) <= hh:
            d = min(hw, hh) if (hw > hh) else min(hw, hh)
            if best is None or d > best:
                best = d
                axis_yaw = 0.0 if hw > hh else math.pi / 2.0
    if best is None:
        axis_yaw = math.atan2(0.0, 1.0)
    if rng.random() < 0.5:
        axis_yaw += math.pi
    yaw = wrap_angle(axis_yaw + float(rng.uniform(-yaw_spread, yaw_spread)))
    return x, y, 0.0, yaw


# ----------------------------------------------------------------- collect
def collect(scene_name: str, seconds: float = 60.0, seed: int = 0,
            noise: bool = True, sample_hz: float = 10.0,
            episodes: int = 2, prefer_nav_expert: bool = False,
            random_start: bool = True) -> Dict[str, np.ndarray]:
    """Run expert episodes in :class:`RoadSim` and return supervised arrays.

    Besides the raw sensor data the expert commands and the reactive
    behaviour-cloning labels are stored: ``labels`` holds the pure-function
    reactive target ``(throttle, steer)`` and ``actions`` holds the quantised
    8-direction planner target.
    """
    from .sim import RoadSim

    sim = RoadSim(scene_name, seed=seed, noise=noise)
    expert = make_expert(scene_name, prefer_nav=prefer_nav_expert)
    scene = sim.scene
    beam_angles = sim.lidar_beam_angles()
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    max_steps = max(1, int(round(float(seconds) / dt)))
    rng = np.random.default_rng((int(seed) + 1234) & 0xFFFFFFFF)
    start_grid = build_planning_grid(scene) if random_start else None
    base_offsets = [float(light.get("offset", 0.0)) for light in scene.lights]

    rows: Dict[str, List[np.ndarray]] = {k: [] for k in (
        "pose", "obs", "scans", "scans_clean", "cmds", "labels", "actions",
        "images", "episode", "time", "light", "speed", "dist_goal",
        "off_road")}
    episode_success: List[float] = []
    totals = {"collisions": 0, "red_light_violations": 0,
              "pedestrians_hit": 0, "off_road_time": 0.0}

    for ep in range(int(episodes)):
        if random_start and ep > 0 and start_grid is not None:
            sim.reset(_jitter_start(scene, start_grid, rng))
        else:
            sim.reset()
        # vary the traffic-light phase between episodes without moving the
        # pedestrians (a shifted initial *time* would make some episodes start
        # in contact with a pedestrian)
        if scene.lights:
            delta = float(rng.uniform(0.0, C.LIGHT_CYCLE))
            for li, light in enumerate(scene.lights):
                light["offset"] = base_offsets[li] + delta
        if hasattr(expert, "reset"):
            expert.reset(sim.pose)
        goal = sim.goal()
        last_scan: Optional[np.ndarray] = None
        success = False
        for k in range(max_steps):
            pose = sim.pose.copy()
            speed = sim.speed
            t = sim.time
            if k % sample_every == 0:
                scan = sim.lidar()
                clean = raycast_scan(pose, scene, t=t)
                image = sim.camera()
                if hasattr(expert, "set_speed"):
                    expert.set_speed(speed)
                try:
                    cmd = expert.update(pose, goal, scan, beam_angles, dt, t=t)
                except TypeError:
                    cmd = expert.update(pose, goal, scan, beam_angles, dt)
                body = reactive_action(pose, goal, scan, scene, speed, t)
                action = quantize_direction(body[1] * C.MAX_STEER)
                light = sim.traffic_light()
                rows["pose"].append(pose)
                rows["obs"].append(control_features(pose, goal, scan, scene,
                                                    speed, t))
                rows["scans"].append(scan)
                rows["scans_clean"].append(clean)
                rows["cmds"].append(np.asarray(cmd, dtype=np.float64))
                rows["labels"].append(np.asarray(body, dtype=np.float64))
                rows["actions"].append(np.asarray(action, dtype=np.int64))
                rows["images"].append(image)
                rows["episode"].append(np.asarray(ep, dtype=np.int64))
                rows["time"].append(np.asarray(t, dtype=np.float64))
                rows["light"].append(np.asarray(
                    light_features(pose, scene, t), dtype=np.float64))
                rows["speed"].append(np.asarray(speed, dtype=np.float64))
                rows["dist_goal"].append(np.asarray(
                    math.hypot(float(pose[0]) - goal[0],
                               float(pose[1]) - goal[1]), dtype=np.float64))
                rows["off_road"].append(np.asarray(
                    0.0 if scene.is_drivable(float(pose[0]),
                                             float(pose[1])) else 1.0,
                    dtype=np.float64))
                last_scan = scan
            else:
                if hasattr(expert, "set_speed"):
                    expert.set_speed(sim.speed)
                try:
                    cmd = expert.update(pose, goal, last_scan, beam_angles,
                                        dt * sample_every, t=sim.time)
                except TypeError:
                    cmd = expert.update(pose, goal, last_scan, beam_angles,
                                        dt * sample_every)
            sim.step(cmd, dt)
            p = sim.pose
            if math.hypot(float(p[0]) - goal[0],
                          float(p[1]) - goal[1]) <= 3.0:
                success = True
                break
        totals["collisions"] += int(sim.collisions)
        totals["red_light_violations"] += int(sim.red_light_violations)
        totals["pedestrians_hit"] += int(sim.pedestrians_hit)
        totals["off_road_time"] += float(sim.off_road_time)
        episode_success.append(1.0 if success else 0.0)
        print("  episode %d/%d scene=%s success=%s collisions=%d red=%d "
              "peds=%d offroad=%.1fs time=%.1fs"
              % (ep + 1, episodes, scene_name, success, sim.collisions,
                 sim.red_light_violations, sim.pedestrians_hit,
                 sim.off_road_time, sim.time))

    out: Dict[str, np.ndarray] = {}
    for li, light in enumerate(scene.lights):
        light["offset"] = base_offsets[li]
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
    out["collisions"] = np.asarray(totals["collisions"], dtype=np.int64)
    out["red_light_violations"] = np.asarray(
        totals["red_light_violations"], dtype=np.int64)
    out["pedestrians_hit"] = np.asarray(totals["pedestrians_hit"],
                                        dtype=np.int64)
    out["off_road_time"] = np.asarray(totals["off_road_time"], dtype=np.float64)
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
        "red_light_violations": int(data["red_light_violations"]),
        "pedestrians_hit": int(data["pedestrians_hit"]),
        "off_road_time": round(float(data["off_road_time"]), 2),
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
    uniq = np.unique(np.asarray(episode_ids, dtype=np.int64))
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    rng.shuffle(uniq)
    n_val = max(1, int(round(len(uniq) * float(val_fraction))))
    val_eps = set(int(e) for e in uniq[:n_val].tolist())
    mask = np.asarray([int(e) in val_eps for e in episode_ids], dtype=bool)
    return ~mask, mask, sorted(val_eps)


def load_training_data(data_dir: Optional[str] = None,
                       scenes: Sequence[str] = C.SCENES,
                       episodes: int = 2, seconds: float = 60.0,
                       seed: int = C.DEFAULT_SEED, force: bool = False,
                       keys: Optional[Sequence[str]] = None
                       ) -> Dict[str, np.ndarray]:
    datasets = ensure_datasets(data_dir=data_dir, scenes=scenes,
                               episodes=episodes, seconds=seconds,
                               seed=seed, force=force)
    if not datasets:
        raise RuntimeError("no dataset available")
    keys = keys or ["pose", "goal", "obs", "scans", "scans_clean", "images",
                    "cmds", "labels", "actions", "episode", "light", "speed",
                    "time", "dist_goal", "off_road"]
    return concat_datasets(datasets, keys)


def ensure_datasets(data_dir: Optional[str] = None,
                    scenes: Sequence[str] = C.SCENES,
                    episodes: int = 2, seconds: float = 60.0,
                    seed: int = C.DEFAULT_SEED,
                    force: bool = False,
                    random_start: bool = True
                    ) -> Dict[str, Dict[str, np.ndarray]]:
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
