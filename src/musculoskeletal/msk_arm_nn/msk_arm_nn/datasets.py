"""Dataset collection and feature engineering for the MSK-Arm project.

The musculoskeletal arm is a planar two-link arm driven by four Hill-type
muscles (shoulder/elbow flexor-extensor pairs).  This module provides:

1. the shared feature vector (end-effector polar, target polar, joint angles,
   joint velocities, muscle activations and a downsampled depth image);
2. an analytic expert: ``IK -> Cartesian A* waypoints -> joint PD with gravity
   / Coriolis compensation -> muscle-activation mapping``;
3. synthetic state augmentation labelled by the same expert (a pure function
   of the state, so behaviour cloning is well posed);
4. the multi-target episode collector (with 32x32 depth images) and the
   discrete 8-action activation-primitive environment used by the RL
   comparison.
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
from .geometry import wrap_angle
from .scenes import Scene, get_scene

# --------------------------------------------------------------------- helper
_PI = math.pi
_TWO_PI = 2.0 * _PI

_ACTION_ANGLES = np.arange(8, dtype=np.float64) * (_PI / 4.0)

WORK_SPAN = float(C.WORK_X[1] - C.WORK_X[0])      # 0.69 m
REACH_MAX = float(C.L1 + C.L2)                    # 0.58 m
FEATURE_DIM = 2 + 2 + 2 + 2 + 4 + 16              # 28 dims
DEPTH_GRID = 4                                    # 4x4 downsample of 32x32
DEPTH_STEP = C.CAM_WIDTH // DEPTH_GRID

# expert gains / limits
KP = 28.0                     # Nm / rad
KD = 2.4                      # Nm s / rad
MAX_Q_DES_STEP = 1.6          # rad, waypoint joint step target smoothing
WAYPOINT_STEP = 0.06          # m, Cartesian lookahead along the A* path
PATH_INFLATION = C.EE_RADIUS + 0.012
GOAL_RADIUS = 0.045           # m, expert switches to the next waypoint


def action_angles() -> np.ndarray:
    return _ACTION_ANGLES.copy()


def _wrap_vec(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a, dtype=np.float64) + _PI) % _TWO_PI - _PI


# ------------------------------------------------------------- depth features
def depth_to_input(images: np.ndarray) -> np.ndarray:
    """uint8/float depth images (N,32,32) -> float32 NCHW (N,1,32,32)."""
    img = np.asarray(images)
    single = img.ndim == 2
    if single:
        img = img[None, ...]
    img = img.astype(np.float32)
    if float(img.max()) > 1.5:
        img = img / 255.0
    img = np.clip(img, 0.0, 1.0)
    if single:
        return img[0][None, :, :]
    return img[:, None, :, :]


def _scene_free_mask(scene, px, py, radius: float) -> np.ndarray:
    """Vectorised ``scene.is_free(px, py, radius)`` (obstacles only)."""
    ok = np.ones(px.shape, dtype=bool)
    for cx, cy, hw, hh in scene.obstacles:
        dx = np.maximum(np.abs(px - cx) - hw, 0.0)
        dy = np.maximum(np.abs(py - cy) - hh, 0.0)
        ok &= (dx * dx + dy * dy) > radius * radius
    return ok


def _march_dists(max_range: float = C.CAM_RANGE, step: float = 0.01,
                 start: float = 0.05) -> np.ndarray:
    out = []
    d = float(start)
    while d < max_range:
        out.append(d)
        d += step
    return np.asarray(out, dtype=np.float64)


def clean_depth(scene) -> np.ndarray:
    """Noise-free replica of :meth:`MskArmSim.depth_camera` (32x32)."""
    img = np.zeros((C.CAM_HEIGHT, C.CAM_WIDTH), dtype=np.float32)
    half = C.CAM_FOV / 2.0
    base_angle = 0.6
    dists = _march_dists()
    for col in range(C.CAM_WIDTH):
        ang = base_angle - half + C.CAM_FOV * (col + 0.5) / C.CAM_WIDTH
        dx, dy = math.cos(ang), math.sin(ang)
        free = _scene_free_mask(scene, dx * dists, dy * dists, 0.006)
        if bool(free.all()):
            dist = C.CAM_RANGE
        else:
            dist = float(dists[int(np.argmin(free))])
        prox = float(np.clip(1.0 - dist / C.CAM_RANGE, 0.0, 1.0))
        h = int(round(C.CAM_HEIGHT * prox))
        if h > 0:
            img[C.CAM_HEIGHT - h:, col] = 1.0
    for row in range(C.CAM_HEIGHT):
        img[row, :] = np.maximum(img[row, :],
                                 0.05 + 0.10 * row / C.CAM_HEIGHT)
    return img


def depth_downsample(depth: np.ndarray, grid: int = DEPTH_GRID) -> np.ndarray:
    """32x32 depth image -> grid x grid mean-proximity features."""
    img = np.asarray(depth, dtype=np.float64)
    if img.ndim == 3:
        img = img[..., 0]
    step = max(1, img.shape[0] // grid)
    h, w = img.shape
    out = np.zeros((grid, grid), dtype=np.float64)
    for i in range(grid):
        for j in range(grid):
            r0, c0 = i * step, j * step
            block = img[r0:min(h, r0 + step), c0:min(w, c0 + step)]
            out[i, j] = float(block.mean()) if block.size else 0.0
    return out.reshape(-1)


# ------------------------------------------------------------- feature vector
def arm_features(sim, target: Optional[Sequence[float]] = None,
                 depth: Optional[np.ndarray] = None) -> np.ndarray:
    """28-dim feature vector describing the arm / target / workspace state."""
    ee = np.asarray(sim.ee(), dtype=np.float64)
    tgt = np.asarray(sim.goal() if target is None else target,
                     dtype=np.float64)[:2]
    rel = tgt - ee
    q = np.asarray(sim.q, dtype=np.float64)
    dq = np.asarray(sim.dq, dtype=np.float64)
    a = np.asarray(sim.a, dtype=np.float64)
    if depth is None:
        depth = sim.depth_camera()
    feats = np.concatenate([
        [float(np.hypot(*ee) / WORK_SPAN), math.atan2(ee[1], ee[0]) / _PI],
        [float(np.hypot(*rel) / WORK_SPAN), math.atan2(rel[1], rel[0]) / _PI],
        [float(np.clip(q[0] / C.Q1_MAX, -1.5, 1.5)),
         float(np.clip(q[1] / C.Q2_MAX, 0.0, 1.5))],
        np.clip(dq / 6.0, -1.0, 1.0),
        np.clip(a, 0.0, 1.0),
        depth_downsample(depth),
    ])
    return np.asarray(feats, dtype=np.float64)


def feature_labels(feats: np.ndarray) -> Dict[str, int]:
    """Feature index bookkeeping (used by the training tools / docs)."""
    return {"ee_polar": 2, "target_polar": 2, "q": 2, "dq": 2, "activation": 4,
            "depth": 16, "total": int(feats.shape[-1])}


# ------------------------------------------------------------------- IK helpers
def ik_solve(x: float, y: float) -> Optional[Tuple[float, float]]:
    """Elbow-up analytic IK (identical to :meth:`MskArmSim.ik`)."""
    d2 = x * x + y * y
    cos_q2 = (d2 - C.L1 * C.L1 - C.L2 * C.L2) / (2.0 * C.L1 * C.L2)
    if cos_q2 < -1.0 or cos_q2 > 1.0:
        return None
    q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
    q1 = math.atan2(y, x) - math.atan2(C.L2 * math.sin(q2),
                                       C.L1 + C.L2 * math.cos(q2))
    q1 = float(np.clip(q1, C.Q1_MIN, C.Q1_MAX))
    q2 = float(np.clip(q2, C.Q2_MIN, C.Q2_MAX))
    return q1, q2


def fk_solve(q1: float, q2: float) -> Tuple[float, float]:
    return (C.L1 * math.cos(q1) + C.L2 * math.cos(q1 + q2),
            C.L1 * math.sin(q1) + C.L2 * math.sin(q1 + q2))


def q_des_for_target(x: float, y: float) -> np.ndarray:
    """Desired joints for a target; fully-extend towards unreachable targets.

    Two ``draw`` targets sit slightly outside the arm's reach (r = 0.628 and
    0.616 m vs L1+L2 = 0.58 m).  For those the best the arm can do is stretch
    straight towards them (residual < TARGET_TOLERANCE), so the elbow is
    commanded to q2 = 0 and the shoulder to the target bearing.
    """
    ik = ik_solve(x, y)
    if ik is not None:
        return np.asarray(ik, dtype=np.float64)
    q1 = float(np.clip(math.atan2(y, x), C.Q1_MIN, C.Q1_MAX))
    return np.array([q1, 0.0], dtype=np.float64)


# -------------------------------------------------- muscle activation mapping
def solve_activations(sim, q_des: Sequence[float],
                      dq_des: Optional[Sequence[float]] = None) -> np.ndarray:
    """Map a desired joint torque into the four muscle activation commands.

    ``tau = Kp (q_des - q) + Kd (dq_des - dq) + gravity + coriolis`` and each
    joint is driven by its antagonist pair (positive torque -> flexor column,
    negative torque -> extensor column of ``sim.activation_map()``).
    """
    q = np.asarray(sim.q, dtype=np.float64)
    dq = np.asarray(sim.dq, dtype=np.float64)
    q_des = np.asarray(q_des, dtype=np.float64)
    dq_des = (np.zeros(2) if dq_des is None
              else np.asarray(dq_des, dtype=np.float64))
    tau = (KP * (q_des - q) + KD * (dq_des - dq)
           + np.asarray(sim.gravity_torque(), dtype=np.float64)
           + np.asarray(sim.coriolis(), dtype=np.float64))
    b = np.asarray(sim.activation_map(), dtype=np.float64)
    u = np.zeros(4, dtype=np.float64)
    for j in range(2):
        pos, neg = 2 * j, 2 * j + 1
        if tau[j] >= 0.0:
            scale = max(abs(float(b[j, pos])), 1e-6)
            u[pos] = float(np.clip(tau[j] / scale, 0.0, 1.0))
        else:
            scale = max(abs(float(b[j, neg])), 1e-6)
            u[neg] = float(np.clip(-tau[j] / scale, 0.0, 1.0))
    return u


def activations_from_state(q: Sequence[float], dq: Sequence[float],
                           target: Sequence[float],
                           scene_name: str = "reach") -> np.ndarray:
    """Pure function used for the behaviour-cloning labels (needs no sim)."""
    q1, q2 = float(q[0]), float(q[1])
    q_des = q_des_for_target(float(target[0]), float(target[1]))
    # local two-link dynamics (same formulas as the simulator)
    q2f = q2
    c2, s2 = math.cos(q2f), math.sin(q2f)
    h = -C.M2 * C.L1 * (C.L2 / 2.0) * s2
    cor = np.array([h * (2.0 * dq[0] * dq[1] + dq[1] ** 2),
                    -h * dq[0] ** 2])
    g = np.array([
        (C.M1 * C.L1 / 2.0 + C.M2 * C.L1) * C.GRAVITY * math.cos(q1)
        + C.M2 * (C.L2 / 2.0) * C.GRAVITY * math.cos(q1 + q2f),
        C.M2 * (C.L2 / 2.0) * C.GRAVITY * math.cos(q1 + q2f)])
    tau = KP * (q_des - np.asarray(q)) + KD * (0.0 - np.asarray(dq)) + g + cor
    u = np.zeros(4, dtype=np.float64)
    # muscle rows: shoulder flex/ext, elbow flex/ext (sign * r * Fmax * fl)
    for j, (fmax, l0, r) in enumerate(((C.MUSCLES[0][1], C.MUSCLES[0][2],
                                        C.MUSCLES[0][3]),
                                       (C.MUSCLES[2][1], C.MUSCLES[2][2],
                                        C.MUSCLES[2][3]))):
        length = l0 + r * float(q[j])
        fl = math.exp(-((length - l0) / (0.5 * l0)) ** 2)
        scale = r * fmax * fl
        if tau[j] >= 0.0:
            u[2 * j] = float(np.clip(tau[j] / max(scale, 1e-6), 0.0, 1.0))
        else:
            u[2 * j + 1] = float(np.clip(-tau[j] / max(scale, 1e-6), 0.0, 1.0))
    return u


# ------------------------------------------------------------- planning grid
def build_planning_grid(scene: Scene, resolution: float = C.PLAN_RESOLUTION,
                        inflate_radius: float = PATH_INFLATION) -> np.ndarray:
    """Workspace occupancy grid (True = blocked), obstacles inflated."""
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
    """8-connected A* on a workspace occupancy grid (arm plane)."""

    def __init__(self, grid: np.ndarray, resolution: float = C.PLAN_RESOLUTION):
        self.grid = np.asarray(grid, dtype=bool)
        self.resolution = float(resolution)
        self.h, self.w = self.grid.shape
        self.x0, self.y0 = float(C.WORK_X[0]), float(C.WORK_Y[0])

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int(math.floor((float(x) - self.x0) / self.resolution))
        r = int(math.floor((float(y) - self.y0) / self.resolution))
        return max(0, min(self.h - 1, r)), max(0, min(self.w - 1, c))

    def cell_to_world(self, r: int, c: int) -> Tuple[float, float]:
        return (self.x0 + (c + 0.5) * self.resolution,
                self.y0 + (r + 0.5) * self.resolution)

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

    def plan(self, start_xy: Sequence[float], goal_xy: Sequence[float]
             ) -> Optional[List[Tuple[float, float]]]:
        sr, sc = self.world_to_cell(start_xy[0], start_xy[1])
        gr, gc = self.world_to_cell(goal_xy[0], goal_xy[1])
        start = self.nearest_free(sr, sc)
        goal = self.nearest_free(gr, gc)
        if start is None or goal is None:
            return None
        start, goal = tuple(start), tuple(goal)
        if start == goal:
            return [self.cell_to_world(goal[0], goal[1])]
        sq2 = math.sqrt(2.0)
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, sq2), (-1, 1, sq2), (1, -1, sq2), (1, 1, sq2)]
        g_score = np.full((self.h, self.w), np.inf, dtype=np.float64)
        parent = np.full((self.h, self.w, 2), -1, dtype=np.int32)
        g_score[start] = 0.0
        heap = [(0.0, 0.0, start[0], start[1])]
        found = False
        while heap:
            _f, base, r, c = heapq.heappop(heap)
            if (r, c) == goal:
                found = True
                break
            if base > g_score[r, c] + 1e-9:
                continue
            for dr, dc, step in moves:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.h and 0 <= nc < self.w):
                    continue
                if self.grid[nr, nc]:
                    continue
                if dr != 0 and dc != 0:
                    if self.grid[r + dr, c] or self.grid[r, c + dc]:
                        continue
                ng = base + step
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


# ------------------------------------------------------------------- expert
class GeometricExpert:
    """IK + Cartesian A* + joint PD + gravity compensation + activation map.

    The controller only needs the simulator state (``q``, ``dq``, ``a`` and the
    dynamics helpers) plus the current target; it plans a short Cartesian path
    around the workspace obstacles and tracks its first waypoint with the joint
    impedance loop.
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
        self.last_info: Dict[str, float] = {}
        self.replans = 0
        self.reset()

    def reset(self, pose=None) -> None:
        self.replans = 0
        self.path = None
        self.path_arr = np.zeros((0, 2))

    def _plan(self, ee: Sequence[float], target: Sequence[float]) -> None:
        self.path = self.planner.plan(ee, target)
        self.replans += 1
        if self.path is None:
            self.path_arr = np.zeros((0, 2))
        else:
            self.path_arr = np.asarray(self.path, dtype=np.float64)

    def _waypoint(self, ee: np.ndarray, target: np.ndarray) -> np.ndarray:
        dist = float(np.hypot(target[0] - ee[0], target[1] - ee[1]))
        if not self.scene.obstacles or dist < 0.10:
            return target
        if self.path_arr.shape[0] == 0:
            self._plan(ee, target)
        if self.path_arr.shape[0] == 0:
            return target
        d = np.hypot(self.path_arr[:, 0] - ee[0], self.path_arr[:, 1] - ee[1])
        i = int(np.argmin(d))
        travelled = 0.0
        j = i
        while j + 1 < len(self.path_arr) and travelled < WAYPOINT_STEP:
            travelled += math.hypot(self.path_arr[j + 1, 0] - self.path_arr[j, 0],
                                    self.path_arr[j + 1, 1] - self.path_arr[j, 1])
            j += 1
        return self.path_arr[j].copy()

    def update(self, sim, goal: Sequence[float],
               dt: float = 1.0 / C.CONTROL_HZ) -> np.ndarray:
        """Return the 4 muscle activation commands for the current state."""
        ee = np.asarray(sim.ee(), dtype=np.float64)
        target = np.asarray(goal, dtype=np.float64)[:2]
        # the scene advances to the next target automatically: re-plan whenever
        # the goal moves (otherwise the cached path keeps pointing backwards)
        prev = getattr(self, "_planned_target", None)
        if prev is None or float(np.hypot(*(target - prev))) > 0.01:
            self.path = None
            self.path_arr = np.zeros((0, 2))
            self._planned_target = target.copy()
        wp = self._waypoint(ee, target)
        q_des = q_des_for_target(float(wp[0]), float(wp[1]))
        u = solve_activations(sim, q_des)
        self.last_info = {
            "ee": (float(ee[0]), float(ee[1])),
            "target": (float(target[0]), float(target[1])),
            "dist": float(np.hypot(target[0] - ee[0], target[1] - ee[1])),
        }
        return u


def make_expert(scene_name: str, prefer_nav: bool = False):
    """Return a nav controller when available (and requested) else the expert."""
    if prefer_nav:
        try:
            from . import control as nav_control  # type: ignore
            for attr in ("MissionController", "ReachingController"):
                factory = getattr(nav_control, attr, None)
                if factory is not None:
                    try:
                        return factory(get_scene(scene_name)), attr
                    except Exception:
                        continue
        except Exception:
            pass
    return GeometricExpert(scene_name), "geometric_expert"


# ------------------------------------------------------- discrete RL env
class DiscreteArmEnv:
    """Discrete activation-primitive reaching task on the real arm dynamics.

    Each action is one of 8 muscle-activation primitives held for ``dt``
    seconds; the agent must reach the current target (and, in the ``draw``
    scene, the next ones) while avoiding the workspace obstacles.  Collisions
    and effort are penalised, which keeps the comparison meaningful.
    """

    ACTION_TABLE = np.array([
        [0.00, 0.00, 0.00, 0.00],   # relax
        [0.80, 0.00, 0.00, 0.00],   # shoulder flex
        [0.00, 0.80, 0.00, 0.00],   # shoulder extend
        [0.00, 0.00, 0.80, 0.00],   # elbow flex
        [0.00, 0.00, 0.00, 0.80],   # elbow extend
        [0.60, 0.00, 0.60, 0.00],   # shoulder+elbow flex
        [0.60, 0.00, 0.00, 0.00],   # gentle shoulder flex
        [0.45, 0.45, 0.45, 0.45],   # co-contraction stiffen
    ], dtype=np.float64)

    def __init__(self, scene_name: str = "reach", dt: float = 0.1,
                 max_steps: int = 80, goal_radius: float = 0.06,
                 random_start: bool = True, seed: int = 2026):
        from .sim import MskArmSim

        self.scene_name = scene_name
        self.scene = get_scene(scene_name)
        self.dt = float(dt)
        self.max_steps = int(max_steps)
        self.goal_radius = float(goal_radius)
        self.random_start = bool(random_start)
        self.rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        self.sim = MskArmSim(scene_name, seed=seed, noise=True)
        self.n_actions = 8
        self.state_dim = FEATURE_DIM
        self.steps = 0
        self.done = False
        self.prev_dist = 0.0
        self.prev_collisions = 0
        self.prev_effort = 0.0

    def _dist(self) -> float:
        ee = self.sim.ee()
        g = self.sim.goal()
        return float(np.hypot(g[0] - ee[0], g[1] - ee[1]))

    def reset(self) -> np.ndarray:
        self.sim.reset()
        if self.random_start:
            for _ in range(40):
                q1 = float(self.rng.uniform(C.Q1_MIN + 0.05, C.Q1_MAX - 0.05))
                q2 = float(self.rng.uniform(C.Q2_MIN + 0.05, C.Q2_MAX - 0.05))
                self.sim.q[:] = (q1, q2)
                self.sim.dq[:] = self.rng.normal(0.0, 0.05, size=2)
                self.sim.a[:] = 0.0
                self.sim._update_ee()
                ee = self.sim.ee()
                if self.scene.is_free(float(ee[0]), float(ee[1]),
                                      radius=C.EE_RADIUS + 0.01):
                    break
        self.steps = 0
        self.done = False
        self.prev_dist = self._dist()
        self.prev_collisions = self.sim.collisions
        self.prev_effort = self.sim.effort
        return arm_features(self.sim)

    def step(self, action: int):
        self.steps += 1
        u = self.ACTION_TABLE[int(action) % 8]
        self.sim.step(u, self.dt)
        reward = -0.02
        dist = self._dist()
        reward += 4.0 * (self.prev_dist - dist)
        self.prev_dist = dist
        if self.sim.collisions > self.prev_collisions:
            reward -= 1.5
        self.prev_collisions = self.sim.collisions
        effort = self.sim.effort - self.prev_effort
        self.prev_effort = self.sim.effort
        reward -= 0.05 * float(effort)
        success = bool(self.sim.success) if self.scene.targets else False
        success = success or (dist <= self.goal_radius)
        if success:
            reward += 12.0
            self.done = True
        elif self.steps >= self.max_steps:
            self.done = True
        info = {"success": bool(success),
                "collisions": int(self.sim.collisions),
                "reached": int(max(self.sim.reached, 1 if success else 0)),
                "distance": dist}
        return arm_features(self.sim), float(reward), bool(self.done), info


# Alias kept so shared scripts keep importing a familiar name.
DiscreteRoomEnv = DiscreteArmEnv


# ---------------------------------------------------------- synthetic augment
def augment_control(scenes: Sequence[str], n_per_scene: int = 2000,
                    seed: int = C.DEFAULT_SEED):
    """Synthetic arm states labelled by the analytic activation policy.

    Returns ``(features, activation_labels, planner_labels)``.
    """
    from .sim import MskArmSim

    rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
    xs, ys, acts = [], [], []
    for scene_name in scenes:
        scene = get_scene(scene_name)
        sim = MskArmSim(scene_name, seed=int(seed) + 17, noise=True)
        targets = list(scene.targets) or [(0.4, 0.25)]
        for _ in range(int(n_per_scene)):
            q1 = float(rng.uniform(C.Q1_MIN, C.Q1_MAX))
            q2 = float(rng.uniform(C.Q2_MIN, C.Q2_MAX))
            sim.q[:] = (q1, q2)
            sim.dq[:] = rng.normal(0.0, 0.8, size=2)
            sim.a[:] = rng.uniform(0.0, 1.0, size=4)
            sim._update_ee()
            target = targets[int(rng.integers(len(targets)))]
            depth = sim.depth_camera()
            feats = arm_features(sim, target, depth)
            u = activations_from_state(sim.q, sim.dq, target)
            ee = sim.ee()
            bearing = math.atan2(target[1] - ee[1], target[0] - ee[0])
            xs.append(feats)
            ys.append(u)
            acts.append(quantize_direction(bearing))
    return (np.asarray(xs, dtype=np.float64),
            np.asarray(ys, dtype=np.float64),
            np.asarray(acts, dtype=np.int64))


def discretize_heading(err: float, n_actions: int = 8) -> int:
    step = _TWO_PI / n_actions
    return int(math.floor(float(err) / step + 0.5)) % n_actions


def quantize_direction(angle: float) -> int:
    return discretize_heading(float(angle), 8)


# ----------------------------------------------------------------- collect
def collect(scene_name: str, seconds: float = 25.0, seed: int = 0,
            noise: bool = True, sample_hz: float = 20.0,
            episodes: int = 3, prefer_nav_expert: bool = False
            ) -> Dict[str, np.ndarray]:
    """Run expert episodes in :class:`MskArmSim` and return supervised data."""
    from .sim import MskArmSim

    sim = MskArmSim(scene_name, seed=seed, noise=noise)
    expert, expert_source = make_expert(scene_name, prefer_nav=prefer_nav_expert)
    scene = sim.scene
    dt = 1.0 / C.CONTROL_HZ
    sample_every = max(1, int(round(C.CONTROL_HZ / float(sample_hz))))
    max_steps = max(1, int(round(float(seconds) / dt)))
    rng = np.random.default_rng((int(seed) + 1234) & 0xFFFFFFFF)

    rows: Dict[str, List[np.ndarray]] = {k: [] for k in (
        "pose", "obs", "depth", "depth_clean", "cmds", "labels", "actions", "episode",
        "time", "q", "dq", "act", "target", "dist_ee", "reached")}
    episode_success: List[float] = []
    totals = {"collisions": 0}

    for ep in range(int(episodes)):
        sim.reset()
        if ep > 0:
            # start away from the target so the demonstration contains real
            # reaching motion (the scene start pose is already next to it)
            targets0 = scene.targets[0] if scene.targets else (0.4, 0.25)
            for _ in range(80):
                r = float(rng.uniform(0.18, C.L1 + C.L2 - 0.03))
                ang = float(rng.uniform(-0.9, 1.35))
                px, py = r * math.cos(ang), r * math.sin(ang)
                if not (C.WORK_X[0] + 0.02 <= px <= C.WORK_X[1] - 0.02):
                    continue
                if not (C.WORK_Y[0] + 0.02 <= py <= C.WORK_Y[1] - 0.02):
                    continue
                if not scene.is_free(px, py, radius=C.EE_RADIUS + 0.01):
                    continue
                if math.hypot(px - targets0[0], py - targets0[1]) < 0.15:
                    continue
                sim.reset((px, py, 0.0, 0.0))
                break
        if hasattr(expert, "reset"):
            try:
                expert.reset(sim.pose())
            except Exception:
                pass
        success = False
        for k in range(max_steps):
            pose = sim.pose().copy()
            if k % sample_every == 0:
                depth = sim.depth_camera()
                depth_clean = clean_depth(scene)
                target = sim.goal()
                feats = arm_features(sim, target, depth)
                cmd = expert.update(sim, target, dt * sample_every)
                lab = activations_from_state(sim.q, sim.dq, target)
                bearing = math.atan2(target[1] - pose[1],
                                     target[0] - pose[0])
                rows["pose"].append(pose)
                rows["obs"].append(feats)
                rows["depth"].append(np.clip(depth * 255.0, 0, 255)
                                     .astype(np.uint8))
                rows["depth_clean"].append(np.clip(depth_clean * 255.0, 0, 255)
                                           .astype(np.uint8))
                rows["cmds"].append(np.asarray(cmd, dtype=np.float64))
                rows["labels"].append(np.asarray(lab, dtype=np.float64))
                rows["actions"].append(np.asarray(quantize_direction(bearing),
                                                  dtype=np.int64))
                rows["episode"].append(np.asarray(ep, dtype=np.int64))
                rows["time"].append(np.asarray(sim.time, dtype=np.float64))
                rows["q"].append(np.asarray(sim.q, dtype=np.float64))
                rows["dq"].append(np.asarray(sim.dq, dtype=np.float64))
                rows["act"].append(np.asarray(sim.a, dtype=np.float64))
                rows["target"].append(np.asarray(target, dtype=np.float64))
                rows["dist_ee"].append(
                    np.asarray(math.hypot(pose[0] - target[0],
                                          pose[1] - target[1]),
                               dtype=np.float64))
                rows["reached"].append(
                    np.asarray(sim.reached, dtype=np.float64))
            else:
                cmd = expert.update(sim, sim.goal(), dt)
            sim.step(cmd, dt)
            if sim.success:
                success = True
                break
        totals["collisions"] += int(sim.collisions)
        episode_success.append(1.0 if success else 0.0)
        print("  episode %d/%d scene=%s success=%s reached=%d collisions=%d "
              "time=%.2fs effort=%.3f max_torque=%.2f"
              % (ep + 1, episodes, scene_name, success, sim.reached,
                 sim.collisions, sim.time, sim.effort, sim.max_torque))

    out: Dict[str, np.ndarray] = {}
    for key, value in rows.items():
        if key in ("depth", "depth_clean"):
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
    out["success"] = np.asarray(episode_success, dtype=np.float64)
    out["episodes"] = np.asarray(int(episodes), dtype=np.int64)
    out["sample_hz"] = np.asarray(float(sample_hz))
    out["seed"] = np.asarray(int(seed), dtype=np.int64)
    out["expert_source"] = np.asarray(expert_source)
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
        "episode_success_rate": float(success.mean()) if success.size else 0.0,
        "expert_source": str(data["expert_source"]) if "expert_source" in data
        else "unknown",
        "feature_shape": list(data["obs"].shape),
        "depth_shape": list(data["depth"].shape),
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
                       episodes: int = 3, seconds: float = 25.0,
                       seed: int = C.DEFAULT_SEED, force: bool = False,
                       keys: Optional[Sequence[str]] = None
                       ) -> Dict[str, np.ndarray]:
    datasets = ensure_datasets(data_dir=data_dir, scenes=scenes,
                               episodes=episodes, seconds=seconds,
                               seed=seed, force=force)
    if not datasets:
        raise RuntimeError("no dataset available")
    keys = keys or ["pose", "goal", "obs", "depth", "depth_clean", "cmds", "labels",
                    "actions", "episode", "q", "dq", "act", "target",
                    "dist_ee", "reached"]
    return concat_datasets(datasets, keys)


def ensure_datasets(data_dir: Optional[str] = None,
                    scenes: Sequence[str] = C.SCENES,
                    episodes: int = 3, seconds: float = 25.0,
                    seed: int = C.DEFAULT_SEED,
                    force: bool = False) -> Dict[str, Dict[str, np.ndarray]]:
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
                       noise=True, episodes=episodes)
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
