"""Dataset collection and feature engineering for the RVNN ROV project.

The module has two jobs:

1. run the MuJoCo ROV simulator with a geometric expert controller and store
   supervised data (raw + clean sonar, camera images, expert commands);
2. build the normalised feature vectors shared by all training tools.

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
from .geometry import grid_to_world, polyline_length, raycast_obstacles, wrap_angle, world_to_grid
from .scenes import Scene, get_scene

# --------------------------------------------------------------------- helper
_PI = math.pi
_SCAN_ANGLES = np.linspace(-C.SONAR_FOV / 2.0, C.SONAR_FOV / 2.0,
                           C.SONAR_BEAMS)
_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (_PI / 4.0)


def scan_angles() -> np.ndarray:
    return _SCAN_ANGLES.copy()


def action_angles() -> np.ndarray:
    return _ACTION_ANGLES.copy()


# ---------------------------------------------------------------- scene rays
def wall_rects(scene: Scene) -> List[Tuple[float, float, float, float]]:
    """Axis aligned wall boxes exactly as they are placed in the MJCF model."""
    w, h = scene.width, scene.height
    return [
        (0.0, 0.0, 0.3, h),              # x = 0 wall
        (w, 0.0, 0.3, h),                # x = width wall
        (w / 2.0, 0.0, w / 2.0, 0.3),    # y = 0 wall
        (w / 2.0, h, w / 2.0, 0.3),      # y = height wall
    ]


def raycast_scan(pose: Sequence[float], scene: Scene,
                 angles: Optional[np.ndarray] = None) -> np.ndarray:
    """Noise-free sonar scan obtained from the analytic scene geometry.

    Matches :meth:`rov_numpy_nn_slam.sim.ROVSim.sonar` (same beam angles, same
    obstacles, same range limits) but without measurement noise.
    """
    angles = _SCAN_ANGLES if angles is None else angles
    world_angles = float(pose[3]) + angles
    dirs = np.stack([np.cos(world_angles), np.sin(world_angles)], axis=1)
    origin = np.array([float(pose[0]), float(pose[1])])
    ranges = raycast_obstacles(
        origin, dirs,
        list(scene.rects) + wall_rects(scene),
        scene.circles,
        C.SONAR_RANGE_MAX,
    )
    return np.clip(ranges, C.SONAR_RANGE_MIN, C.SONAR_RANGE_MAX)


def sonar_noise_scan(clean: np.ndarray,
                     rng: np.random.Generator) -> np.ndarray:
    """Apply the same noise model as :meth:`ROVSim.sonar` to a clean scan."""
    ranges = np.asarray(clean, dtype=np.float64).copy()
    ranges = ranges * (1.0 + 0.01 * rng.normal(size=ranges.shape))
    ranges += 0.02 * rng.normal(size=ranges.shape)
    outlier = rng.random(ranges.shape) < 0.04
    ranges[outlier] = rng.uniform(0.5, C.SONAR_RANGE_MAX, size=outlier.sum())
    dropout = rng.random(ranges.shape) < 0.03
    ranges[dropout] = C.SONAR_RANGE_MAX
    return np.clip(ranges, C.SONAR_RANGE_MIN, C.SONAR_RANGE_MAX)


def augment_scans(scenes: Sequence[str], n_per_scene: int = 2000,
                  seed: int = C.DEFAULT_SEED,
                  resolution: float = C.PLAN_RESOLUTION):
    """Synthesise extra noisy/clean sonar pairs from random free poses.

    The clean scan is computed analytically from the scene geometry and the
    noise follows the simulator model, so the synthetic pairs have exactly
    the same distribution as the collected data while covering the whole map.
    """
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    noisy, clean = [], []
    for scene_name in scenes:
        scene = get_scene(scene_name)
        grid = build_planning_grid(scene, resolution)
        free = np.argwhere(~grid)
        if len(free) == 0:
            continue
        hi = max(1.1, min(float(scene.depth) - 0.3, C.MAX_DEPTH))
        for _ in range(int(n_per_scene)):
            r, c = free[int(rng.integers(len(free)))]
            x, y = grid_to_world(int(c), int(r), resolution)
            pose = (x, y, float(rng.uniform(1.0, hi)),
                    float(rng.uniform(-_PI, _PI)))
            cl = raycast_scan(pose, scene)
            clean.append(cl)
            noisy.append(sonar_noise_scan(cl, rng))
    return (np.asarray(noisy, dtype=np.float64),
            np.asarray(clean, dtype=np.float64))


# ------------------------------------------------------- reactive expert
def sector_angles(n_sectors: int = 16) -> np.ndarray:
    """Relative bearing of each sonar sector centre."""
    per = C.SONAR_BEAMS // n_sectors
    return _SCAN_ANGLES.reshape(n_sectors, per).mean(axis=1)


_SECTOR_ANGLES = sector_angles(16)


def reactive_action(pose: Sequence[float], goal: Sequence[float],
                    scan: np.ndarray,
                    n_sectors: int = 16) -> np.ndarray:
    """Vector-field-histogram style reactive expert command.

    This local policy maps ``(goal bearing, 16 sonar sectors)`` directly to
    ``(surge, 0, 0, yaw_rate)`` so it is exactly learnable by the control MLP
    from the same features.  Candidate travel directions are the sector
    centres; each is scored by its angular distance to the goal plus a
    penalty for nearby obstacles.  The best direction drives the yaw command.
    """
    p = np.asarray(pose, dtype=np.float64)
    g = np.asarray(goal, dtype=np.float64).reshape(-1)[:3]
    scan = np.asarray(scan, dtype=np.float64)
    sectors = sector_features(scan[None, :], n_sectors)[0]
    ranges = sectors * (C.SONAR_RANGE_MAX - C.SONAR_RANGE_MIN) + C.SONAR_RANGE_MIN
    ang = _SECTOR_ANGLES if n_sectors == 16 else sector_angles(n_sectors)
    bearing = wrap_angle(math.atan2(g[1] - p[1], g[0] - p[0]) - float(p[3]))
    aligned = (ang - bearing + _PI) % (2.0 * _PI) - _PI
    obstacle = np.clip(1.0 - ranges / 3.5, 0.0, 1.0) * 2.0
    cost = np.abs(aligned) + obstacle
    # soft (continuous) direction selection keeps the policy smooth so the
    # control MLP can imitate it accurately
    weights = np.exp(-cost / 0.5)
    weights = weights / max(float(weights.sum()), 1e-9)
    steer = float(np.sum(weights * ang))
    yaw_cmd = float(np.clip(2.2 * steer, -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
    front = np.abs(ang) <= 0.5
    # mean (not min) clearance keeps the demonstrated surge smooth enough for
    # the control MLP to imitate accurately
    front_clear = (float(ranges[front].mean()) if bool(front.any())
                   else C.SONAR_RANGE_MAX)
    surge = (C.MAX_SURGE
             * (1.0 - 0.50 * min(1.0, abs(steer) / 1.2))
             * float(np.clip((front_clear - 0.6) / 1.4, 0.20, 1.0)))
    return np.array([surge, 0.0, 0.0, yaw_cmd], dtype=np.float64)


def reactive_commands(pose: np.ndarray, goal: np.ndarray,
                      scans: np.ndarray) -> np.ndarray:
    """Reactive expert commands for a batch of recorded states (T x 4)."""
    pose = np.atleast_2d(np.asarray(pose, dtype=np.float64))
    goal = np.asarray(goal, dtype=np.float64)
    scans = np.asarray(scans, dtype=np.float64)
    out = np.zeros((len(pose), 4), dtype=np.float64)
    for i in range(len(pose)):
        g = goal[i] if goal.ndim == 2 and goal.shape[0] == len(pose) else goal
        out[i] = reactive_action(pose[i], g, scans[i])
    return out


def augment_control(scenes: Sequence[str], n_per_scene: int = 2000,
                    seed: int = C.DEFAULT_SEED,
                    resolution: float = C.PLAN_RESOLUTION):
    """Synthetic states labelled by the reactive expert for the control MLP."""
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    xs, ys = [], []
    for scene_name in scenes:
        scene = get_scene(scene_name)
        grid = build_planning_grid(scene, resolution)
        free = np.argwhere(~grid)
        if len(free) == 0:
            continue
        hi = max(1.1, min(float(scene.depth) - 0.3, C.MAX_DEPTH))
        goal = np.asarray(scene.goal, dtype=np.float64)
        for _ in range(int(n_per_scene)):
            r, c = free[int(rng.integers(len(free)))]
            x, y = grid_to_world(int(c), int(r), resolution)
            pose = np.array([x, y, float(rng.uniform(1.0, hi)),
                             float(rng.uniform(-_PI, _PI))])
            clean = raycast_scan(pose, scene)
            scan = sonar_noise_scan(clean, rng)
            x_feat = np.concatenate([
                sector_features(scan[None, :], 16)[0],
                goal_features(pose, goal),
            ])
            cmd = reactive_action(pose, goal, scan)
            xs.append(x_feat)
            ys.append([cmd[0] / C.MAX_SURGE, cmd[3] / C.MAX_YAW_RATE])
    return np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64)


# ------------------------------------------------------------- planning grid
def build_planning_grid(scene: Scene, resolution: float = C.PLAN_RESOLUTION,
                        inflate_radius: float = 0.38) -> np.ndarray:
    """Occupancy grid (True = blocked) with walls and a safety inflation."""
    grid = scene.occupancy(resolution)
    w, h = scene.width, scene.height
    xs = (np.arange(grid.shape[1]) + 0.5) * resolution
    ys = (np.arange(grid.shape[0]) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    grid |= (gx < 0.30) | (gx > w - 0.30) | (gy < 0.30) | (gy > h - 0.30)
    cells = int(round(inflate_radius / resolution))
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

    def nearest_free(self, r: int, c: int, max_radius: int = 40) -> Optional[Tuple[int, int]]:
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
             goal_xy: Sequence[float]) -> Optional[List[Tuple[float, float]]]:
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
        heap = [(0.0, start[0], start[1])]
        found = False
        while heap:
            f, r, c = heapq.heappop(heap)
            if (r, c) == goal:
                found = True
                break
            base = g_score[r, c]
            if f > base + 1e9:
                continue
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
                    heapq.heappush(heap, (ng + hh, nr, nc))
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


# ------------------------------------------------------------------- expert
class GeometricExpert:
    """Deterministic A* + pure-pursuit + sonar-safety expert controller.

    This is the fallback expert used for behaviour cloning.  If the concurrent
    ``control.MissionController`` is importable it can be used instead (see
    :func:`make_expert`), but the geometric expert is always available.
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
        self.start_depth = float(self.scene.start[2])
        self.goal = np.asarray(self.scene.goal, dtype=np.float64)
        self.last_info: Dict[str, float] = {}
        self.reset()

    # ------------------------------------------------------------------ api
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        if pose is None:
            x, y, depth = self.scene.start[:3]
        else:
            x, y, depth = float(pose[0]), float(pose[1]), float(pose[2])
        self.start_depth = depth
        self._plan(x, y)

    def _plan(self, x: float, y: float) -> None:
        self.path = self.planner.plan((x, y), self.goal[:2])
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
        if float(d.min()) > 2.0:
            self._plan(x, y)

    def _lookahead_target(self, x: float, y: float,
                          lookahead: float = 0.9) -> Tuple[float, float]:
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
        x, y, depth, yaw = [float(v) for v in pose]
        goal = np.asarray(goal, dtype=np.float64)
        self.goal = goal
        self._replan_if_needed(x, y)
        tx, ty = self._lookahead_target(x, y)
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        if dist_goal < 0.9:
            tx, ty = float(goal[0]), float(goal[1])
        desired = math.atan2(ty - y, tx - x)
        err = wrap_angle(desired - yaw)

        yaw_cmd = float(np.clip(1.8 * err, -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        surge = C.MAX_SURGE * (1.0 - 0.55 * min(1.0, abs(err) / 1.2))
        if abs(err) > 0.9:
            surge *= 0.55
        if dist_goal < 1.2:
            surge *= max(0.0, min(1.0, dist_goal / 1.2))

        # ---- depth: ramp from the current depth to the goal depth
        if self.path_length > 1e-6:
            progress = 1.0 - min(1.0, dist_goal / self.path_length)
        else:
            progress = 0.0
        target_depth = (self.start_depth
                        + (float(goal[2]) - self.start_depth) * progress)
        heave = float(np.clip(1.2 * (depth - target_depth),
                              -C.MAX_HEAVE, C.MAX_HEAVE))

        # ---- sonar safety layer
        if scan is not None:
            ang = (_SCAN_ANGLES if beam_angles is None
                   else np.asarray(beam_angles, dtype=np.float64))
            scan = np.asarray(scan, dtype=np.float64)
            front = np.abs(ang) <= 0.50
            if bool(front.any()):
                front_min = float(scan[front].min())
                if front_min < 1.6:
                    surge *= max(0.0, min(1.0, (front_min - 0.55) / 1.05))
                if front_min < 0.85:
                    left = scan[ang > 0.35]
                    right = scan[ang < -0.35]
                    lm = float(left.min()) if left.size else C.SONAR_RANGE_MAX
                    rm = float(right.min()) if right.size else C.SONAR_RANGE_MAX
                    yaw_cmd = C.MAX_YAW_RATE * 0.9 * (1.0 if lm >= rm else -1.0)
                    surge = 0.0

        self.last_info = {
            "desired_heading": desired,
            "heading_err": float(err),
            "target_depth": float(target_depth),
            "dist_goal": float(dist_goal),
            "yaw_cmd": float(yaw_cmd),
            "surge": float(surge),
        }
        return np.array([surge, 0.0, heave, yaw_cmd], dtype=np.float64)


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


# ---------------------------------------------------------------- features
def scan_features(scans: np.ndarray) -> np.ndarray:
    """Normalise sonar ranges (metres) to [0, 1]."""
    s = np.asarray(scans, dtype=np.float64)
    span = C.SONAR_RANGE_MAX - C.SONAR_RANGE_MIN
    return np.clip((s - C.SONAR_RANGE_MIN) / span, 0.0, 1.0)


def sector_features(scans: np.ndarray, n_sectors: int = 16) -> np.ndarray:
    """Minimum range per angular sector, normalised (T x n_sectors)."""
    s = scan_features(scans)
    if s.ndim == 1:
        s = s[None, :]
    per = C.SONAR_BEAMS // n_sectors
    return s.reshape(s.shape[0], n_sectors, per).min(axis=2)


def goal_features(pose: np.ndarray, goal: np.ndarray) -> np.ndarray:
    """Relative polar goal features: [r, sin(bearing), cos(bearing), dz].

    ``goal`` may be a single ``(x, y, depth)`` triple or a per-sample
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
    bearing = (bearing + _PI) % (2.0 * _PI) - _PI
    feats = np.stack([
        np.clip(dist / 15.0, 0.0, 1.0),
        np.sin(bearing),
        np.cos(bearing),
        np.clip((g[:, 2] - p[:, 2]) / 2.0, -1.0, 1.0),
    ], axis=1)
    return feats[0] if single else feats


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
    return np.transpose(img, (0, 3, 1, 2)) if not single else np.transpose(img[0], (2, 0, 1))


def discretize_heading(err: float, n_actions: int = 8) -> int:
    """Map a signed heading error (rad, + = left) to one of 8 directions."""
    step = 2.0 * _PI / n_actions
    return int(math.floor(float(err) / step + 0.5)) % n_actions


# ------------------------------------------------------- discrete RL env
class DiscretePoolEnv:
    """Lightweight discrete-action abstraction of the pool navigation task.

    The ROV is a point mass moving 0.25 m per step in one of 8 world-frame
    directions on the pool occupancy grid.  This keeps RL training fast while
    using the exact same scene geometry as the MuJoCo benchmark.
    """

    def __init__(self, scene_name: str = "pool",
                 step_size: float = 0.25, max_steps: int = 120,
                 goal_radius: float = 0.45,
                 resolution: float = 0.10,
                 inflate_radius: float = 0.32,
                 random_start: bool = True,
                 seed: int = 2026):
        self.scene = get_scene(scene_name)
        self.resolution = float(resolution)
        self.grid = build_planning_grid(self.scene, resolution, inflate_radius)
        self.step_size = float(step_size)
        self.max_steps = int(max_steps)
        self.goal_radius = float(goal_radius)
        self.random_start = bool(random_start)
        self.start_xy = np.array(self.scene.start[:2], dtype=np.float64)
        self.goal_xy = np.array(self.scene.goal[:2], dtype=np.float64)
        self.angles = _ACTION_ANGLES
        self.rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        self._offsets = np.arange(-0.9, 0.91, 0.3)
        self.state_dim = len(self._offsets) ** 2 + 3
        self.n_actions = 8
        self.pos = self.start_xy.copy()
        self.steps = 0
        self.collisions = 0
        self.prev_dist = 0.0
        self.done = False

    # ------------------------------------------------------------------ api
    def _cell(self, xy: np.ndarray) -> Tuple[int, int]:
        c = int(math.floor(float(xy[0]) / self.resolution))
        r = int(math.floor(float(xy[1]) / self.resolution))
        c = max(0, min(self.grid.shape[1] - 1, c))
        r = max(0, min(self.grid.shape[0] - 1, r))
        return r, c

    def blocked(self, xy: np.ndarray) -> bool:
        r, c = self._cell(xy)
        return bool(self.grid[r, c])

    def observation(self) -> np.ndarray:
        ox = self.pos[0] + self._offsets
        oy = self.pos[1] + self._offsets
        cols = np.clip((ox / self.resolution).astype(np.int64), 0,
                       self.grid.shape[1] - 1)
        rows = np.clip((oy / self.resolution).astype(np.int64), 0,
                       self.grid.shape[0] - 1)
        patch = self.grid[np.ix_(rows, cols)].astype(np.float64).reshape(-1)
        dx = self.goal_xy[0] - self.pos[0]
        dy = self.goal_xy[1] - self.pos[1]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        goal = np.array([np.clip(dist / 20.0, 0.0, 1.0),
                         math.sin(bearing), math.cos(bearing)])
        return np.concatenate([patch, goal], axis=0)

    def sample_free_start(self, min_goal_dist: float = 4.0,
                          max_goal_dist: float = 14.0) -> np.ndarray:
        """Random free start position at a reasonable distance from the goal."""
        h, w = self.grid.shape
        for _ in range(500):
            r = int(self.rng.integers(1, h - 1))
            c = int(self.rng.integers(1, w - 1))
            if self.grid[r, c]:
                continue
            xy = np.array(grid_to_world(c, r, self.resolution))
            d = float(np.hypot(*(self.goal_xy - xy)))
            if min_goal_dist <= d <= max_goal_dist:
                return xy
        return self.start_xy.copy()

    def reset(self) -> np.ndarray:
        if self.random_start:
            self.pos = self.sample_free_start()
        else:
            self.pos = self.start_xy.copy()
        self.steps = 0
        self.collisions = 0
        self.done = False
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
                "distance": dist}
        return self.observation(), float(reward), bool(self.done), info


def sample_free_pose(scene: Scene, grid: np.ndarray,
                     rng: np.random.Generator,
                     resolution: float = C.PLAN_RESOLUTION,
                     goal_xy: Optional[Sequence[float]] = None,
                     yaw_spread: float = 0.5) -> Tuple[float, float, float, float]:
    """Sample a collision-free pose (x, y, depth, yaw) inside the scene.

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
    lo = 1.0
    hi = max(lo + 0.1, min(float(scene.depth) - 0.3, C.MAX_DEPTH))
    depth = float(rng.uniform(lo, hi))
    return x, y, depth, yaw


# ----------------------------------------------------------------- collect
def collect(scene_name: str, seconds: float = 60.0, seed: int = 0,
            noise: bool = True, sample_hz: float = 10.0,
            episodes: int = 1, prefer_nav_expert: bool = False,
            random_start: bool = False) -> Dict[str, np.ndarray]:
    """Run expert episodes in :class:`ROVSim` and return supervised arrays."""
    from .sim import ROVSim

    sim = ROVSim(scene_name, seed=seed, noise=noise)
    expert = make_expert(scene_name, prefer_nav=prefer_nav_expert)
    scene = sim.scene
    beam_angles = sim.sonar_beam_angles()
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    max_steps = max(1, int(round(float(seconds) / dt)))
    rng = np.random.default_rng((int(seed) + 1234) & 0xFFFFFFFF)
    start_grid = build_planning_grid(scene) if random_start else None

    rows = {k: [] for k in ("pose", "obs", "scans", "scans_clean", "cmds",
                            "actions", "images", "episode", "time")}
    episode_success: List[float] = []
    total_collisions = 0

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
                scan = sim.sonar()
                clean = raycast_scan(pose, scene)
                image = sim.camera()
                cmd = expert.update(pose, goal, scan, beam_angles, dt)
                info = getattr(expert, "last_info", {}) or {}
                err = float(info.get("heading_err", 0.0))
                rows["pose"].append(pose)
                rows["obs"].append(obs_features(pose, goal))
                rows["scans"].append(scan)
                rows["scans_clean"].append(clean)
                rows["cmds"].append(np.asarray(cmd, dtype=np.float64))
                rows["actions"].append(discretize_heading(err))
                rows["images"].append(image)
                rows["episode"].append(ep)
                rows["time"].append(sim.time)
                last_scan = scan
            else:
                cmd = expert.update(pose, goal, last_scan, beam_angles, dt)
            sim.step(cmd, dt)
            p = sim.pose
            dist_goal = math.hypot(float(p[0]) - goal[0], float(p[1]) - goal[1])
            if dist_goal < 0.7 and abs(float(p[2]) - float(goal[2])) < 0.5:
                success = True
                break
        total_collisions += int(sim.collisions)
        episode_success.append(1.0 if success else 0.0)
        print("  episode %d/%d scene=%s success=%s collisions=%d time=%.1fs"
              % (ep + 1, episodes, scene_name, success, sim.collisions,
                 sim.time))

    out: Dict[str, np.ndarray] = {}
    for key, value in rows.items():
        if key == "images":
            out[key] = np.asarray(value, dtype=np.uint8)
        elif key in ("episode", "actions"):
            out[key] = np.asarray(value, dtype=np.int64)
        else:
            out[key] = np.asarray(value, dtype=np.float64)
    out["goal_xyz"] = np.asarray(sim.goal(), dtype=np.float64)
    out["goal"] = np.tile(np.asarray(sim.goal(), dtype=np.float64),
                          (len(out["pose"]), 1))
    out["scene"] = np.asarray(scene_name)
    out["collisions"] = np.asarray(total_collisions, dtype=np.int64)
    out["success"] = np.asarray(episode_success, dtype=np.float64)
    out["episodes"] = np.asarray(int(episodes), dtype=np.int64)
    out["sample_hz"] = np.asarray(float(sample_hz))
    out["seed"] = np.asarray(int(seed), dtype=np.int64)
    out["random_start"] = np.asarray(bool(random_start))
    return out


# ----------------------------------------------------------- data plumbing
def save_dataset(data: Dict[str, np.ndarray], path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez(path, **data)


def load_collected(data_dir: Optional[str] = None,
                   scenes: Optional[Sequence[str]] = None) -> Dict[str, Dict[str, np.ndarray]]:
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
    return {
        "scene": str(data["scene"]),
        "samples": int(len(data["pose"])),
        "episodes": int(data["episodes"]),
        "sample_hz": float(data["sample_hz"]),
        "seed": int(data["seed"]),
        "collisions": int(data["collisions"]),
        "episode_success_rate": float(success.mean()) if success.size else 0.0,
        "random_start": bool(data["random_start"]) if "random_start" in data else False,
        "scan_shape": list(data["scans"].shape),
        "clean_scan_shape": list(data["scans_clean"].shape),
        "image_shape": list(data["images"].shape),
        "cmd_shape": list(data["cmds"].shape),
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
        episode.append(np.asarray(data["episode"], dtype=np.int64) + 1000 * scene_idx)
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
                       episodes: int = 2, seconds: float = 40.0,
                       seed: int = C.DEFAULT_SEED, force: bool = False,
                       keys: Optional[Sequence[str]] = None) -> Dict[str, np.ndarray]:
    """Ensure datasets exist and return one concatenated array dictionary."""
    datasets = ensure_datasets(data_dir=data_dir, scenes=scenes,
                               episodes=episodes, seconds=seconds,
                               seed=seed, force=force)
    if not datasets:
        raise RuntimeError("no dataset available")
    keys = keys or ["pose", "goal", "obs", "scans", "scans_clean", "images",
                    "cmds", "actions", "episode"]
    return concat_datasets(datasets, keys)


def ensure_datasets(data_dir: Optional[str] = None,
                    scenes: Sequence[str] = C.SCENES,
                    episodes: int = 2, seconds: float = 40.0,
                    seed: int = C.DEFAULT_SEED,
                    force: bool = False,
                    random_start: bool = True) -> Dict[str, Dict[str, np.ndarray]]:
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
        save_dataset(data, os.path.join(data_dir, "dataset_%s.npz" % scene_name))
        meta = dataset_metadata(data)
        meta["seconds_wall"] = round(time.time() - t0, 2)
        with open(os.path.join(data_dir, "dataset_%s.json" % scene_name), "w",
                  encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2, sort_keys=True)
        existing[scene_name] = data
        print("  -> %d samples in %.1fs" % (meta["samples"], meta["seconds_wall"]))
    return existing
