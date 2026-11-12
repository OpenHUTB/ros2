"""Dataset collection and feature engineering for the HNN humanoid project.

The module has four jobs:

1. a fast, *exact* analytic replica of the humanoid lidar (72 beams, 360 deg,
   0.10 m ray marching against walls / crates / pillars / moving carts) used to
   build noise-free targets for the denoising network;
2. feature builders shared by every training tool (18 lidar sectors with 4
   beams each, relative goal polar, walking speed and lateral balance);
3. a geometric expert (A* + pure pursuit + speed control with a lateral
   balance constraint) plus a *reactive* walking policy whose labels are a pure
   function of the features, so behaviour cloning is well posed;
4. dataset plumbing (collect / save / load / split) and the discrete
   8-action indoor environment used by the RL comparison.

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

# Exactly the beam angles used by :class:`humanoid_walk_nn.sim.HumanoidSim`.
_SCAN_ANGLES = np.linspace(-_PI, _PI, C.LIDAR_BEAMS, endpoint=False)
# Body-frame directions of the 8 discrete planner / RL actions.
_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (_PI / 4.0)

N_SECTORS = 18                        # 72 beams / 4 beams per sector
SECTOR_BEAMS = C.LIDAR_BEAMS // N_SECTORS
MARCH_STEP = 0.10                     # m, must match HumanoidSim._ray_hit()


def _march_distances() -> np.ndarray:
    """March distances built by repeated addition, exactly like the simulator."""
    out = []
    d = float(C.LIDAR_RANGE_MIN)
    while d < C.LIDAR_RANGE_MAX:
        out.append(d)
        d += MARCH_STEP
    return np.asarray(out, dtype=np.float64)


MARCH_DISTS = _march_distances()
GRID_INFLATION = C.ROBOT_RADIUS + 0.33   # m, clearance kept by A* / sampling
CRUISE_SPEED = 0.86 * C.MAX_SPEED     # m/s, expert free-walk speed
VW_LIMIT = 0.75                       # (m/s)*(rad/s): steady balance offset
GOAL_RADIUS = 0.45                    # m, planner stop distance
CLEARANCE_SLOW = 1.6                  # m, lidar clearance where the expert slows


def scan_angles() -> np.ndarray:
    return _SCAN_ANGLES.copy()


def action_angles() -> np.ndarray:
    return _ACTION_ANGLES.copy()


def sector_angles(n_sectors: int = N_SECTORS) -> np.ndarray:
    per = C.LIDAR_BEAMS // int(n_sectors)
    return _SCAN_ANGLES.reshape(int(n_sectors), per).mean(axis=1)


_SECTOR_ANGLES = sector_angles(N_SECTORS)


def _wrap_vec(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a, dtype=np.float64) + _PI) % _TWO_PI - _PI


# ---------------------------------------------------------------- scene rays
def _free_points(scene: Scene, px: np.ndarray, py: np.ndarray,
                 radius: float) -> np.ndarray:
    """Vectorised ``scene.is_free(px, py, radius)``."""
    free = ((px >= radius) & (py >= radius)
            & (px <= scene.width - radius) & (py <= scene.height - radius))
    r2 = radius * radius
    for cx, cy, hw, hh in list(scene.walls) + list(scene.obstacles):
        dx = np.maximum(np.abs(px - cx) - hw, 0.0)
        dy = np.maximum(np.abs(py - cy) - hh, 0.0)
        free &= (dx * dx + dy * dy) > r2
    for cx, cy, r in scene.pillars:
        rr = radius + r
        free &= (px - cx) ** 2 + (py - cy) ** 2 > rr * rr
    return free


def _blocked_points(scene: Scene, px: np.ndarray, py: np.ndarray,
                    t: float = 0.0) -> np.ndarray:
    """Vectorised replica of ``HumanoidSim._ray_hit`` point tests."""
    blocked = ~_free_points(scene, px, py, 0.05)
    for cx, cy, hw, hh in list(scene.obstacles):
        blocked |= (np.abs(px - cx) <= hw) & (np.abs(py - cy) <= hh)
    for cx, cy, hw, hh in scene.cart_positions(t):
        blocked |= (np.abs(px - cx) <= hw) & (np.abs(py - cy) <= hh)
    return blocked


def raycast_scan(pose: Sequence[float], scene: Scene,
                 angles: Optional[np.ndarray] = None,
                 t: float = 0.0) -> np.ndarray:
    """Noise-free lidar scan identical to :meth:`HumanoidSim.lidar` (noise off).

    The simulator marches in 0.10 m steps from 0.15 m up to the maximum range
    and stops at the first blocked point; this function evaluates all
    72 x 79 samples at once with NumPy and reproduces it exactly.
    """
    angles = _SCAN_ANGLES if angles is None else np.asarray(angles, dtype=np.float64)
    yaw = float(pose[3])
    rel = yaw + angles
    dx, dy = np.cos(rel), np.sin(rel)
    px = float(pose[0]) + np.outer(dx, MARCH_DISTS)
    py = float(pose[1]) + np.outer(dy, MARCH_DISTS)
    blocked = _blocked_points(scene, px, py, t)
    any_hit = blocked.any(axis=1)
    idx = blocked.argmax(axis=1)
    ranges = np.where(any_hit, MARCH_DISTS[idx], C.LIDAR_RANGE_MAX)
    return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)


def lidar_noise_scan(clean: np.ndarray,
                     rng: np.random.Generator) -> np.ndarray:
    """Apply the exact :meth:`HumanoidSim.lidar` noise model to a clean scan."""
    ranges = np.asarray(clean, dtype=np.float64).copy()
    ranges = ranges * (1.0 + 0.01 * rng.normal(size=ranges.shape))
    ranges += 0.02 * rng.normal(size=ranges.shape)
    outlier = rng.random(ranges.shape) < 0.03
    ranges[outlier] = rng.uniform(1.0, C.LIDAR_RANGE_MAX,
                                  size=int(outlier.sum()))
    dropout = rng.random(ranges.shape) < 0.02
    ranges[dropout] = C.LIDAR_RANGE_MAX
    return np.clip(ranges, C.LIDAR_RANGE_MIN, C.LIDAR_RANGE_MAX)


# Kept for compatibility with the shared vehicle code.
sonar_noise_scan = lidar_noise_scan


# ------------------------------------------------------------- planning grid
def build_planning_grid(scene: Scene, resolution: float = C.PLAN_RESOLUTION,
                        inflate_radius: float = GRID_INFLATION) -> np.ndarray:
    """Occupancy grid (True = blocked) with walls/obstacles/pillars inflated.

    Moving carts are deliberately *not* part of the static grid: they are
    avoided reactively from the live lidar scan.
    """
    grid = scene.occupancy(resolution)
    h, w = grid.shape
    cells = int(round(float(inflate_radius) / float(resolution)))
    if cells > 0:
        padded = np.pad(grid, cells, mode="constant", constant_values=True)
        out = np.zeros_like(grid)
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                out |= padded[cells + dr:cells + dr + h,
                              cells + dc:cells + dc + w]
        grid = out
    return grid


class AStarPlanner:
    """8-connected A* on a boolean occupancy grid."""

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
                     max_radius: int = 40) -> Optional[Tuple[int, int]]:
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
             max_expansions: int = 400000
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
    feats = np.stack([np.clip(dist / 20.0, 0.0, 1.0),
                      np.sin(bearing), np.cos(bearing)], axis=1)
    return feats[0] if single else feats


CONTROL_DIM = 2 * N_SECTORS + 3 + 1 + 1      # 41 dims


def control_features(pose: np.ndarray, goal: np.ndarray, scan: np.ndarray,
                     speed: float = 0.0, balance: float = 0.0) -> np.ndarray:
    """Full control-MLP input vector (41 dims for the default layout)."""
    p = np.asarray(pose, dtype=np.float64)
    single = p.ndim == 1
    p2 = np.atleast_2d(p)
    s = np.asarray(scan, dtype=np.float64)
    if s.ndim == 1:
        s = s[None, :]
    sectors = sector_stats(s, N_SECTORS, ("min", "mean"))
    gf = goal_features(p2, goal)
    rows = p2.shape[0]
    sp = np.full((rows, 1), float(np.clip(speed / C.MAX_SPEED, -1.0, 1.0)))
    bal = np.full((rows, 1),
                  float(np.clip(balance / max(C.BALANCE_LIMIT, 1e-6),
                                -1.5, 1.5)))
    out = np.concatenate([sectors, gf, sp, bal], axis=1)
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


def depth_to_input(images: np.ndarray) -> np.ndarray:
    """uint8/float depth images (N,32,32) -> float32 NCHW (N,1,32,32)."""
    img = np.asarray(images)
    single = img.ndim == 2
    if single:
        img = img[None, ...]
    img = img.astype(np.float32)
    if img.max() > 1.5:
        img = img / 255.0
    img = np.clip(img, 0.0, 1.0)
    if single:
        return img[0][None, :, :]
    return img[:, None, :, :]


def image_to_input(images: np.ndarray) -> np.ndarray:
    """uint8 HWC RGB images -> float32 NCHW in [0, 1]."""
    img = np.asarray(images)
    single = img.ndim == 3
    if single:
        img = img[None, ...]
    img = img.astype(np.float32) / 255.0
    if single:
        return np.transpose(img[0], (2, 0, 1))
    return np.transpose(img, (0, 3, 1, 2))


def discretize_heading(err: float, n_actions: int = 8) -> int:
    step = _TWO_PI / n_actions
    return int(math.floor(float(err) / step + 0.5)) % n_actions


def quantize_direction(angle: float) -> int:
    return discretize_heading(float(angle), 8)


# ------------------------------------------------------ balance-constrained
def turn_limit(v: float) -> float:
    """Maximum turn rate that keeps the steady lateral CoM offset in bounds."""
    v_eff = max(abs(float(v)), 0.16)
    return min(C.MAX_TURN_RATE, VW_LIMIT / v_eff)


def balance_limited(v: float, omega: float) -> Tuple[float, float]:
    """Clamp (v, omega) so that BALANCE_GAIN*v*omega/RESTORE stays safe."""
    lim = turn_limit(v)
    return float(v), float(np.clip(omega, -lim, lim))


# ------------------------------------------------------ reactive expert
def reactive_action(pose: Sequence[float], goal: Sequence[float],
                    scan: np.ndarray, speed: float = 0.0,
                    balance: float = 0.0,
                    n_sectors: int = N_SECTORS) -> np.ndarray:
    """Reactive walking policy, a pure function of the features.

    Returns ``(v, omega)`` in m/s and rad/s.  The policy uses only quantities
    that are also part of :func:`control_features` (sector minima/means,
    relative goal bearing, current speed and lateral balance).
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
    # circular interpolation of the clearance along a bearing
    ang_ext = np.concatenate([ang, ang + _TWO_PI])
    r_ext = np.concatenate([ranges, ranges])
    clear_fwd = float(np.interp(bearing, ang_ext, r_ext))
    clear_left = float(np.interp(bearing + 0.70, ang_ext, r_ext))
    clear_right = float(np.interp(bearing - 0.70, ang_ext, r_ext))
    asym = float(np.clip((clear_left - clear_right) / 3.0, -1.0, 1.0))
    blocked = float(np.clip(1.0 - clear_fwd / CLEARANCE_SLOW, 0.0, 1.0))

    omega = 2.0 * bearing + 1.1 * asym * max(0.35, blocked)
    # near the goal, finish straight so the stop is clean
    if dist_goal < 1.2:
        omega = 1.8 * bearing
    v = CRUISE_SPEED * (1.0 - 0.55 * min(1.0, abs(omega) / C.MAX_TURN_RATE))
    v *= float(np.clip((clear_fwd - 0.20) / 1.8, 0.10, 1.0))
    if dist_goal < GOAL_RADIUS + 0.5:
        v *= float(np.clip(dist_goal / (GOAL_RADIUS + 0.5), 0.0, 1.0))
    # lateral balance: never command v * omega above the safe limit
    v, omega = balance_limited(v, omega)
    omega = float(np.clip(omega, -C.MAX_TURN_RATE, C.MAX_TURN_RATE))
    v = float(np.clip(v, 0.0, C.MAX_SPEED))
    return np.array([v, omega], dtype=np.float64)


def reactive_commands(pose: np.ndarray, goal: np.ndarray, scans: np.ndarray,
                      speeds: Optional[np.ndarray] = None,
                      balances: Optional[np.ndarray] = None) -> np.ndarray:
    pose = np.atleast_2d(np.asarray(pose, dtype=np.float64))
    scans = np.asarray(scans, dtype=np.float64)
    goal_arr = np.asarray(goal, dtype=np.float64)
    if goal_arr.ndim == 1:
        goal_arr = np.repeat(goal_arr.reshape(1, 3), len(pose), axis=0)
    speeds = (np.zeros(len(pose)) if speeds is None
              else np.asarray(speeds, dtype=np.float64).reshape(-1))
    balances = (np.zeros(len(pose)) if balances is None
                else np.asarray(balances, dtype=np.float64).reshape(-1))
    out = np.zeros((len(pose), 2), dtype=np.float64)
    for i in range(len(pose)):
        out[i] = reactive_action(pose[i], goal_arr[i], scans[i],
                                 speeds[i], balances[i])
    return out


def reactive_labels(pose: np.ndarray, goal: np.ndarray, scans: np.ndarray,
                    speeds: np.ndarray, balances: np.ndarray) -> np.ndarray:
    """Normalised control labels ``(v/MAX_SPEED, omega/MAX_TURN_RATE)``."""
    cmds = reactive_commands(pose, goal, scans, speeds, balances)
    out = np.zeros_like(cmds)
    out[:, 0] = np.clip(cmds[:, 0] / C.MAX_SPEED, -1.0, 1.0)
    out[:, 1] = np.clip(cmds[:, 1] / C.MAX_TURN_RATE, -1.0, 1.0)
    return out


def augment_control(scenes: Sequence[str], n_per_scene: int = 2000,
                    seed: int = C.DEFAULT_SEED,
                    resolution: float = C.PLAN_RESOLUTION):
    """Synthetic walking states labelled by the reactive expert.

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
            pose = np.array([x, y, 0.0, float(rng.uniform(-_PI, _PI))])
            speed = float(rng.uniform(0.0, C.MAX_SPEED))
            balance = float(rng.uniform(-0.5, 0.5) * C.BALANCE_LIMIT)
            t = float(rng.uniform(0.0, 20.0))
            clean = raycast_scan(pose, scene, t=t)
            scan = lidar_noise_scan(clean, rng)
            cmd = reactive_action(pose, goal, scan, speed, balance)
            xs.append(control_features(pose, goal, scan, speed, balance))
            ys.append([cmd[0] / C.MAX_SPEED, cmd[1] / C.MAX_TURN_RATE])
            acts.append(quantize_direction(cmd[1] * 0.6))
    return (np.asarray(xs, dtype=np.float64),
            np.asarray(ys, dtype=np.float64),
            np.asarray(acts, dtype=np.int64))


def augment_scans(scenes: Sequence[str], n_per_scene: int = 2000,
                  seed: int = C.DEFAULT_SEED,
                  resolution: float = C.PLAN_RESOLUTION):
    """Synthesise extra noisy / clean scan pairs from random free poses."""
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
            t = float(rng.uniform(0.0, 20.0))
            cl = raycast_scan(pose, scene, t=t)
            clean.append(cl)
            noisy.append(lidar_noise_scan(cl, rng))
    return (np.asarray(noisy, dtype=np.float64),
            np.asarray(clean, dtype=np.float64))


# ------------------------------------------------------------------- expert
class GeometricExpert:
    """Deterministic A* + pure-pursuit + balance-limited speed expert.

    Throttles the walking speed down in corners so that the lateral CoM offset
    ``BALANCE_GAIN * v * omega / BALANCE_RESTORE`` stays well inside the fall
    limit, and slows/steers away from obstacles seen by the lidar (including
    the moving carts, which are not in the static planning grid).
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
        self.goal = np.asarray(self.scene.goal, dtype=np.float64)
        self.last_info: Dict[str, float] = {}
        self.replans = 0
        self.reset()

    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y = self.scene.start[0], self.scene.start[1]
        else:
            x, y = float(pose[0]), float(pose[1])
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
        if float(d.min()) > 0.9:
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
               dt: float = 1.0 / C.CONTROL_HZ,
               speed: float = 0.0, t: float = 0.0) -> np.ndarray:
        x, y, _z, yaw = [float(v) for v in pose[:4]]
        goal = np.asarray(goal, dtype=np.float64)
        self.goal = goal
        self._replan_if_needed(x, y)
        dist_goal = math.hypot(float(goal[0]) - x, float(goal[1]) - y)
        lookahead = float(np.clip(0.8 + 0.8 * max(speed, 0.0), 1.0, 3.5))
        tx, ty = self._lookahead_target(x, y, lookahead)
        if dist_goal < lookahead:
            tx, ty = float(goal[0]), float(goal[1])
        desired = math.atan2(ty - y, tx - x)
        err = wrap_angle(desired - yaw)
        omega = float(np.clip(2.0 * err, -C.MAX_TURN_RATE, C.MAX_TURN_RATE))

        # ---- walk speed: slow for corners, the goal and nearby obstacles
        v = CRUISE_SPEED * (1.0 - 0.55 * min(1.0, abs(err) / 1.2))
        if dist_goal < 1.0:
            v *= float(np.clip(dist_goal / 1.0, 0.0, 1.0))
        if scan is not None:
            ang = (_SCAN_ANGLES if beam_angles is None
                   else np.asarray(beam_angles, dtype=np.float64))
            sc = np.asarray(scan, dtype=np.float64)
            cone = np.abs(_wrap_vec(ang - err)) <= 0.55
            if bool(cone.any()):
                front = float(sc[cone].min())
                if front < CLEARANCE_SLOW:
                    v = min(v, max(0.10, (front - 0.20) * 0.85))
                if front < 0.45:
                    v = 0.0
                    left = float(sc[_wrap_vec(ang) > 0.30].min()) \
                        if bool((_wrap_vec(ang) > 0.30).any()) else C.LIDAR_RANGE_MAX
                    right = float(sc[_wrap_vec(ang) < -0.30].min()) \
                        if bool((_wrap_vec(ang) < -0.30).any()) else C.LIDAR_RANGE_MAX
                    omega = C.MAX_TURN_RATE * 0.8 * (1.0 if left >= right else -1.0)
            # side clearance: keep a corridor around the body
            left_c = float(sc[(_wrap_vec(ang) > 0.45) & (_wrap_vec(ang) < 1.2)].min()) \
                if bool(((_wrap_vec(ang) > 0.45) & (_wrap_vec(ang) < 1.2)).any()) \
                else C.LIDAR_RANGE_MAX
            right_c = float(sc[(_wrap_vec(ang) < -0.45) & (_wrap_vec(ang) > -1.2)].min()) \
                if bool(((_wrap_vec(ang) < -0.45) & (_wrap_vec(ang) > -1.2)).any()) \
                else C.LIDAR_RANGE_MAX
            corridor = min(left_c, right_c)
            if corridor < 0.55:
                v = min(v, 0.35)
            if corridor < 0.55:
                bias = float(np.clip((left_c - right_c) / 1.2, -1.0, 1.0))
                omega = float(np.clip(omega + 0.6 * bias, -C.MAX_TURN_RATE,
                                      C.MAX_TURN_RATE))
        v, omega = balance_limited(v, omega)
        v = float(np.clip(v, 0.0, C.MAX_SPEED))
        omega = float(np.clip(omega, -C.MAX_TURN_RATE, C.MAX_TURN_RATE))
        self.last_info = {"heading_err": float(err), "dist_goal": float(dist_goal),
                          "v": float(v), "omega": float(omega),
                          "target_speed": float(v)}
        return np.array([v, omega], dtype=np.float64)


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
class DiscreteRoomEnv:
    """Discrete-action abstraction of the indoor walking task.

    The humanoid is driven with one of 8 (walk speed, turn rate) primitives
    for ``dt`` seconds using the same longitudinal / turning / lateral-balance
    model as the simulator, on the same scene geometry (walls, crates, pillars
    and moving carts at the current time).  Collisions and falls are penalised
    and counted, which keeps RL training fast while staying meaningful.
    """

    ACTION_TABLE = np.array([
        [1.00, 0.00],    # walk forward
        [0.90, 0.35],    # forward-left
        [0.90, -0.35],   # forward-right
        [0.60, 0.85],    # turn left
        [0.60, -0.85],   # turn right
        [0.30, 0.00],    # slow forward
        [0.00, 0.00],    # stand
        [1.00, 0.90],    # risky fast turn (can exceed the balance limit)
    ], dtype=np.float64)

    def __init__(self, scene_name: str = "hall",
                 max_steps: int = 90, goal_radius: float = 0.6,
                 resolution: float = 0.20, dt: float = 0.5,
                 random_start: bool = True, seed: int = 2026):
        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.resolution = float(resolution)
        self.dt = float(dt)
        self.blocked = build_planning_grid(self.scene, resolution,
                                           inflate_radius=C.ROBOT_RADIUS)
        self.goal_xy = np.array(self.scene.goal[:2], dtype=np.float64)
        self.start_pose = np.array(self.scene.start, dtype=np.float64)
        self.max_steps = int(max_steps)
        self.goal_radius = float(goal_radius)
        self.random_start = bool(random_start)
        self.rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        self._offsets = np.arange(-1.6, 1.61, 0.4)     # 9 values
        self.state_dim = len(self._offsets) ** 2 + 3 + 1 + 1
        self.n_actions = 8
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.yaw_rate = 0.0
        self.balance = 0.0
        self._fall_timer = 0.0
        self.time = 0.0
        self.steps = 0
        self.collisions = 0
        self.falls = 0
        self.prev_dist = 0.0
        self.done = False

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
        goal = np.array([np.clip(dist / 20.0, 0.0, 1.0),
                         math.sin(rel), math.cos(rel)])
        speed = np.array([np.clip(self.v / C.MAX_SPEED, -1.0, 1.0)])
        bal = np.array([np.clip(self.balance / C.BALANCE_LIMIT, -2.0, 2.0)])
        return np.concatenate([patch, goal, speed, bal], axis=0)

    def sample_free_start(self) -> np.ndarray:
        h, w = self.blocked.shape
        for _ in range(800):
            r = int(self.rng.integers(1, h - 1))
            c = int(self.rng.integers(1, w - 1))
            if self.blocked[r, c]:
                continue
            x, y = grid_to_world(c, r, self.resolution)
            if not self.scene.is_free(x, y, radius=C.ROBOT_RADIUS + 0.10):
                continue
            d = math.hypot(self.goal_xy[0] - x, self.goal_xy[1] - y)
            if 2.0 <= d <= 12.0:
                yaw = math.atan2(self.goal_xy[1] - y,
                                 self.goal_xy[0] - x) + float(self.rng.normal(0, 0.5))
                return np.array([x, y, 0.0, wrap_angle(yaw)])
        return self.start_pose.copy()

    def reset(self) -> np.ndarray:
        pose = self.sample_free_start() if self.random_start else self.start_pose
        self.x, self.y, self.yaw = float(pose[0]), float(pose[1]), float(pose[3])
        self.v = self.yaw_rate = self.balance = self._fall_timer = 0.0
        self.time = 0.0
        self.steps = 0
        self.collisions = 0
        self.falls = 0
        self.done = False
        self.prev_dist = float(math.hypot(self.goal_xy[0] - self.x,
                                          self.goal_xy[1] - self.y))
        return self.observation()

    def step(self, action: int):
        self.steps += 1
        v_target, w_target = self.ACTION_TABLE[int(action) % 8]
        v_target *= C.MAX_SPEED
        w_target *= C.MAX_TURN_RATE
        dv = v_target - self.v
        accel = C.MAX_ACCEL if dv > 0 else C.MAX_DECEL
        self.v += float(np.clip(dv, -accel * self.dt, accel * self.dt))
        dw = w_target - self.yaw_rate
        self.yaw_rate += float(np.clip(dw, -C.TURN_ACCEL * self.dt,
                                       C.TURN_ACCEL * self.dt))
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * self.dt)
        self.balance += (C.BALANCE_GAIN * self.v * self.yaw_rate
                         - C.BALANCE_RESTORE * self.balance) * self.dt
        reward = -0.02
        fell = False
        if abs(self.balance) > C.BALANCE_LIMIT:
            self._fall_timer += self.dt
            if self._fall_timer > C.FALL_HOLD:
                self.falls += 1
                self._fall_timer = 0.0
                self.balance = 0.0
                self.v = 0.0
                self.yaw_rate = 0.0
                fell = True
                reward -= 3.0
        else:
            self._fall_timer = max(0.0, self._fall_timer - 0.5 * self.dt)
        nx = self.x + self.v * math.cos(self.yaw) * self.dt
        ny = self.y + self.v * math.sin(self.yaw) * self.dt
        self.time += self.dt
        if not self.scene.is_free(nx, ny, radius=C.ROBOT_RADIUS):
            self.collisions += 1
            self.v = 0.0
            reward -= 1.5
        else:
            self.x, self.y = nx, ny
        dist = float(math.hypot(self.goal_xy[0] - self.x,
                                self.goal_xy[1] - self.y))
        reward += 2.5 * (self.prev_dist - dist)
        self.prev_dist = dist
        success = dist <= self.goal_radius
        if success:
            reward += 12.0
            self.done = True
        elif self.steps >= self.max_steps:
            self.done = True
        info = {"success": bool(success), "collisions": self.collisions,
                "falls": self.falls, "fall": bool(fell), "distance": dist}
        return self.observation(), float(reward), bool(self.done), info


# Alias kept so shared scripts keep working.
DiscretePoolEnv = DiscreteRoomEnv
DiscreteRoadEnv = DiscreteRoomEnv


def sample_free_pose(scene: Scene, grid: np.ndarray,
                     rng: np.random.Generator,
                     resolution: float = C.PLAN_RESOLUTION,
                     goal_xy: Optional[Sequence[float]] = None,
                     yaw_spread: float = 0.6
                     ) -> Tuple[float, float, float, float]:
    """Sample a collision-free pose (x, y, 0, yaw) inside the room."""
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
    return x, y, 0.0, yaw


# ----------------------------------------------------------------- collect
def collect(scene_name: str, seconds: float = 90.0, seed: int = 0,
            noise: bool = True, sample_hz: float = 10.0,
            episodes: int = 2, prefer_nav_expert: bool = False,
            random_start: bool = False) -> Dict[str, np.ndarray]:
    """Run expert episodes in :class:`HumanoidSim` and return supervised data.

    The expert's own ``(v, omega)`` commands and the pure-function reactive
    labels are both stored, together with the 32x32 depth camera images.
    """
    from .sim import HumanoidSim

    sim = HumanoidSim(scene_name, seed=seed, noise=noise)
    expert = make_expert(scene_name, prefer_nav=prefer_nav_expert)
    scene = sim.scene
    beam_angles = sim.lidar_beam_angles()
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    max_steps = max(1, int(round(float(seconds) / dt)))
    rng = np.random.default_rng((int(seed) + 1234) & 0xFFFFFFFF)
    start_grid = build_planning_grid(scene) if random_start else None
    base_cart_phases = [float(cart.get("phase", 0.0)) for cart in scene.carts]

    rows: Dict[str, List[np.ndarray]] = {k: [] for k in (
        "pose", "obs", "scans", "scans_clean", "depth", "cmds", "labels",
        "actions", "episode", "time", "speed", "balance", "dist_goal")}
    episode_success: List[float] = []
    totals = {"collisions": 0, "falls": 0}

    for ep in range(int(episodes)):
        if random_start and ep > 0 and start_grid is not None:
            sim.reset(sample_free_pose(scene, start_grid, rng,
                                       goal_xy=scene.goal[:2]))
        else:
            sim.reset()
        # vary the moving-cart phase between episodes (harmless when there are
        # no carts) so the demonstrations cover different cart timings
        for ci, cart in enumerate(scene.carts):
            cart["phase"] = base_cart_phases[ci] + float(rng.uniform(0.0, 0.5))
        if hasattr(expert, "reset"):
            expert.reset(sim.pose)
        goal = sim.goal()
        last_scan: Optional[np.ndarray] = None
        success = False
        for k in range(max_steps):
            pose = sim.pose.copy()
            speed = sim.speed
            balance = sim.balance
            t = sim.time
            if k % sample_every == 0:
                scan = sim.lidar()
                clean = raycast_scan(pose, scene, t=t)
                depth = np.clip(sim.camera_depth() * 255.0, 0, 255).astype(np.uint8)
                try:
                    cmd = expert.update(pose, goal, scan, beam_angles, dt,
                                        speed=speed, t=t)
                except TypeError:
                    try:
                        cmd = expert.update(pose, goal, scan, beam_angles, dt,
                                            t=t)
                    except TypeError:
                        cmd = expert.update(pose, goal, scan, beam_angles, dt)
                body = reactive_action(pose, goal, scan, speed, balance)
                action = quantize_direction(body[1] * 0.6)
                rows["pose"].append(pose)
                rows["obs"].append(control_features(pose, goal, scan, speed,
                                                    balance))
                rows["scans"].append(scan)
                rows["scans_clean"].append(clean)
                rows["depth"].append(depth)
                rows["cmds"].append(np.asarray(cmd, dtype=np.float64))
                rows["labels"].append(np.asarray(
                    reactive_labels(pose[None, :], goal[None, :],
                                    scan[None, :], np.asarray([speed]),
                                    np.asarray([balance]))[0],
                    dtype=np.float64))
                rows["actions"].append(np.asarray(action, dtype=np.int64))
                rows["episode"].append(np.asarray(ep, dtype=np.int64))
                rows["time"].append(np.asarray(t, dtype=np.float64))
                rows["speed"].append(np.asarray(speed, dtype=np.float64))
                rows["balance"].append(np.asarray(balance, dtype=np.float64))
                rows["dist_goal"].append(np.asarray(
                    math.hypot(float(pose[0]) - goal[0],
                               float(pose[1]) - goal[1]), dtype=np.float64))
                last_scan = scan
            else:
                try:
                    cmd = expert.update(pose, goal, last_scan, beam_angles,
                                        dt * sample_every, speed=speed, t=t)
                except TypeError:
                    cmd = expert.update(pose, goal, last_scan, beam_angles,
                                        dt * sample_every)
            sim.step(cmd, dt)
            p = sim.pose
            if math.hypot(float(p[0]) - goal[0],
                          float(p[1]) - goal[1]) <= 0.6:
                success = True
                break
        totals["collisions"] += int(sim.collisions)
        totals["falls"] += int(sim.falls)
        episode_success.append(1.0 if success else 0.0)
        print("  episode %d/%d scene=%s success=%s collisions=%d falls=%d "
              "time=%.1fs" % (ep + 1, episodes, scene_name, success,
                              sim.collisions, sim.falls, sim.time))

    out: Dict[str, np.ndarray] = {}
    for key, value in rows.items():
        if key == "depth":
            out[key] = np.asarray(value, dtype=np.uint8)
        elif key == "actions":
            out[key] = np.asarray(value, dtype=np.int64)
        else:
            out[key] = np.asarray(value, dtype=np.float64)
    for ci, cart in enumerate(scene.carts):
        cart["phase"] = base_cart_phases[ci]
    out["goal_xyz"] = np.asarray(sim.goal(), dtype=np.float64)
    out["goal"] = np.tile(np.asarray(sim.goal(), dtype=np.float64),
                          (len(out["pose"]), 1))
    out["scene"] = np.asarray(scene_name)
    out["collisions"] = np.asarray(totals["collisions"], dtype=np.int64)
    out["falls"] = np.asarray(totals["falls"], dtype=np.int64)
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
        "falls": int(data["falls"]),
        "episode_success_rate": float(success.mean()) if success.size else 0.0,
        "random_start": bool(data["random_start"]) if "random_start" in data else False,
        "beams": int(data["scans"].shape[1]),
        "scan_shape": list(data["scans"].shape),
        "clean_scan_shape": list(data["scans_clean"].shape),
        "depth_shape": list(data["depth"].shape),
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
                       episodes: int = 2, seconds: float = 90.0,
                       seed: int = C.DEFAULT_SEED, force: bool = False,
                       keys: Optional[Sequence[str]] = None
                       ) -> Dict[str, np.ndarray]:
    datasets = ensure_datasets(data_dir=data_dir, scenes=scenes,
                               episodes=episodes, seconds=seconds,
                               seed=seed, force=force)
    if not datasets:
        raise RuntimeError("no dataset available")
    keys = keys or ["pose", "goal", "obs", "scans", "scans_clean", "depth",
                    "cmds", "labels", "actions", "episode", "speed",
                    "balance", "time", "dist_goal"]
    return concat_datasets(datasets, keys)


def ensure_datasets(data_dir: Optional[str] = None,
                    scenes: Sequence[str] = C.SCENES,
                    episodes: int = 2, seconds: float = 90.0,
                    seed: int = C.DEFAULT_SEED,
                    force: bool = False,
                    random_start: bool = False
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
