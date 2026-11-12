"""Global and local path planning for the road vehicle (Ackermann car).

* :class:`GridPlanner` -- 8-connected A* on the road occupancy grid (off-road,
  parked cars, cones and barriers are all blocked), circular inflation by at
  least the car safety radius, octile heuristic and line-of-sight smoothing.
* :class:`DWAPlanner` -- dynamic-window local planner: samples
  ``(v, delta)``, rolls the kinematic bicycle model out for a short horizon and
  rejects rollouts that hit a lidar return or leave the drivable road.
* :class:`GeometricPlanner` -- simple goal-attraction / obstacle-repulsion
  fallback controller.

The vehicle is strictly 2-D: poses are ``(x, y, 0, yaw)`` and the command is
``(throttle, steer)`` with ``steer > 0`` meaning "turn left".
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import bresenham, wrap_angle

__all__ = ["GridPlanner", "DWAPlanner", "GeometricPlanner",
           "scene_planning_grid"]

_SQRT2 = math.sqrt(2.0)
_MOVES = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2),
)

_RANGE_MAX = float(C.LIDAR_RANGE_MAX)
_RANGE_MIN = float(C.LIDAR_RANGE_MIN)
_FRONT_HALF_FOV = math.radians(60.0)


# ------------------------------------------------------------------- helpers
def scene_planning_grid(scene, resolution: float = C.PLAN_RESOLUTION
                        ) -> np.ndarray:
    """Blocked planning grid for a :class:`scenes.Scene`.

    Uses :meth:`scenes.Scene.coarse_map` when it really is sampled at
    ``resolution``; otherwise rebuilds exactly at that resolution from
    :meth:`scenes.Scene.occupancy` (the shipped ``coarse_map`` used an integer
    downsample factor which silently changed 0.25 m to 0.20 m cells).
    """
    expected = (int(round(float(scene.height) / resolution)),
                int(round(float(scene.width) / resolution)))
    cm = getattr(scene, "coarse_map", None)
    if cm is not None:
        grid = np.asarray(scene.coarse_map(resolution), dtype=bool)
        if grid.shape == expected:
            return grid
    return np.asarray(scene.occupancy(resolution), dtype=bool)


def _clean_scan(scan: Sequence[float], beam_angles: Sequence[float],
                window: int = 2) -> Tuple[np.ndarray, np.ndarray]:
    """Drop no-return beams and isolated outliers from a lidar scan."""
    ranges = np.asarray(scan, dtype=np.float64).ravel()
    beams = np.asarray(beam_angles, dtype=np.float64).ravel()
    n = min(ranges.size, beams.size)
    ranges, beams = ranges[:n], beams[:n]
    if n == 0:
        return ranges, beams
    if n >= 5:
        padded = np.pad(ranges, window, mode="edge")
        stack = np.stack([padded[i: i + n] for i in range(2 * window + 1)],
                         axis=0)
        med = np.median(stack, axis=0)
        outlier = (ranges > med + 0.8) | (ranges < med - 0.8)
        ranges = np.where(outlier, med, ranges)
    valid = np.isfinite(ranges) & (ranges > _RANGE_MIN + 0.02) \
        & (ranges < _RANGE_MAX * 0.995)
    return ranges[valid], beams[valid]


def _scan_points(pose: Sequence[float], scan: Sequence[float],
                 beam_angles: Sequence[float]) -> np.ndarray:
    """World-frame obstacle points from a lidar scan, shape ``(N, 2)``."""
    pose = np.asarray(pose, dtype=np.float64).ravel()
    if pose.size < 2:
        return np.zeros((0, 2), dtype=np.float64)
    if pose.size >= 4:
        yaw = float(pose[3])
    elif pose.size >= 3:
        yaw = float(pose[2])
    else:
        yaw = 0.0
    ranges, beams = _clean_scan(scan, beam_angles)
    if ranges.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    ang = yaw + beams
    return np.column_stack([pose[0] + ranges * np.cos(ang),
                            pose[1] + ranges * np.sin(ang)])


def _inflate_circular(grid: np.ndarray, radius_cells: float) -> np.ndarray:
    """Dilate a boolean grid with a circular structuring element."""
    if radius_cells <= 0.0:
        return grid.copy()
    k = int(math.ceil(radius_cells))
    h, w = grid.shape
    out = grid.copy()
    padded = np.pad(grid, k, mode="constant", constant_values=False)
    r2 = radius_cells * radius_cells + 1e-9
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dx * dx + dy * dy <= r2:
                out |= padded[k + dy: k + dy + h, k + dx: k + dx + w]
    return out


class GridPlanner:
    """8-connected A* global planner (no corner cutting, car-radius inflation).

    Parameters
    ----------
    occupancy_bool:
        2-D boolean grid, ``True`` = blocked (build it with
        :func:`scene_planning_grid` for a :class:`scenes.Scene`).
    resolution:
        Grid cell size in metres, default :data:`config.PLAN_RESOLUTION`.
    inflate_cells:
        Obstacle inflation radius in cells.  Defaults to
        :data:`config.INFLATION_RADIUS` (1.55 m) which is larger than the car
        radius (:data:`config.CAR_RADIUS`, 1.05 m).
    """

    def __init__(self, occupancy_bool: np.ndarray,
                 resolution: float = C.PLAN_RESOLUTION,
                 inflate_cells: Optional[float] = None) -> None:
        self.resolution = float(resolution)
        base = np.asarray(occupancy_bool, dtype=bool)
        if base.ndim != 2:
            raise ValueError("occupancy_bool must be a 2-D array")
        self.base = base
        self.height, self.width = base.shape
        self.car_radius = float(C.CAR_RADIUS)
        if inflate_cells is None:
            self.inflate_radius_m = max(float(C.CAR_RADIUS),
                                        float(C.INFLATION_RADIUS))
            self.inflate_radius_cells = self.inflate_radius_m / self.resolution
        else:
            self.inflate_radius_cells = max(0.0, float(inflate_cells))
            self.inflate_radius_m = self.inflate_radius_cells * self.resolution
        self.inflate_cells = int(math.ceil(self.inflate_radius_cells))
        self.grid = _inflate_circular(base, self.inflate_radius_cells)

    # ------------------------------------------------------------ conversion
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        c = int(math.floor(float(x) / self.resolution))
        r = int(math.floor(float(y) / self.resolution))
        return min(max(c, 0), self.width - 1), min(max(r, 0), self.height - 1)

    def cell_to_world(self, c: int, r: int) -> Tuple[float, float]:
        return (float(c) + 0.5) * self.resolution, (float(r) + 0.5) * self.resolution

    def is_free(self, x: float, y: float) -> bool:
        """True when the inflated planning grid is free at world ``(x, y)``."""
        c, r = self.world_to_cell(x, y)
        return not bool(self.grid[r, c])

    def clearance(self, x: float, y: float, max_cells: int = 12) -> float:
        """Distance (m) from ``(x, y)`` to the nearest raw blocked cell."""
        c, r = self.world_to_cell(x, y)
        if self.base[r, c]:
            return 0.0
        h, w = self.height, self.width
        for rad in range(1, int(max_cells) + 1):
            r0, r1 = max(0, r - rad), min(h, r + rad + 1)
            c0, c1 = max(0, c - rad), min(w, c + rad + 1)
            window = self.base[r0:r1, c0:c1]
            if window.any():
                rr, cc = np.nonzero(window)
                d = np.hypot((cc + c0 - c) * self.resolution,
                             (rr + r0 - r) * self.resolution)
                return float(d.min())
        return float(max_cells) * self.resolution

    # ---------------------------------------------------------------- search
    def _snap(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Nearest free cell to world ``(x, y)`` (``(col, row)``) or ``None``."""
        c0, r0 = self.world_to_cell(x, y)
        if not self.grid[r0, c0]:
            return c0, r0
        limit = max(self.width, self.height)
        window = 4
        while window <= limit:
            r_lo, r_hi = max(0, r0 - window), min(self.height - 1, r0 + window)
            c_lo, c_hi = max(0, c0 - window), min(self.width - 1, c0 + window)
            free = ~self.grid[r_lo: r_hi + 1, c_lo: c_hi + 1]
            if np.any(free):
                rr, cc = np.nonzero(free)
                rr = rr + r_lo
                cc = cc + c_lo
                d = (cc - c0) ** 2 + (rr - r0) ** 2
                k = int(np.argmin(d))
                return int(cc[k]), int(rr[k])
            window *= 2
        return None

    def nearest_free(self, x: float, y: float) -> Tuple[float, float]:
        """World coordinates of the nearest free cell to ``(x, y)``."""
        cell = self._snap(float(x), float(y))
        if cell is None:
            return float(x), float(y)
        return self.cell_to_world(*cell)

    @staticmethod
    def _heuristic(c: int, r: int, gc: int, gr: int) -> float:
        dc, dr = abs(gc - c), abs(gr - r)
        return (dc + dr) + (_SQRT2 - 2.0) * min(dc, dr)

    def plan(self, start_xy: Sequence[float],
             goal_xy: Sequence[float]) -> Optional[List[Tuple[float, float]]]:
        """A* path ``[(x, y), ...]`` from start to goal (world coords) or ``None``."""
        start = self._snap(float(start_xy[0]), float(start_xy[1]))
        goal = self._snap(float(goal_xy[0]), float(goal_xy[1]))
        if start is None or goal is None:
            return None
        sc, sr = start
        gc, gr = goal
        if (sc, sr) == (gc, gr):
            return [self.cell_to_world(sc, sr)]
        w, h = self.width, self.height
        start_idx = sr * w + sc
        goal_idx = gr * w + gc

        g = np.full(h * w, np.inf, dtype=np.float64)
        parent = np.full(h * w, -1, dtype=np.int64)
        closed = np.zeros(h * w, dtype=bool)
        g[start_idx] = 0.0
        heap: List[Tuple[float, int, int]] = [
            (self._heuristic(sc, sr, gc, gr), 0, start_idx)]
        counter = 1
        grid = self.grid
        found = False
        while heap:
            _, _, cur = heapq.heappop(heap)
            if closed[cur]:
                continue
            closed[cur] = True
            if cur == goal_idx:
                found = True
                break
            r, c = divmod(cur, w)
            base_cost = g[cur]
            for dc, dr, cost in _MOVES:
                nc, nr = c + dc, r + dr
                if nc < 0 or nc >= w or nr < 0 or nr >= h:
                    continue
                if grid[nr, nc]:
                    continue
                if dc != 0 and dr != 0 and (grid[r, nc] or grid[nr, c]):
                    continue  # no corner cutting
                nxt = nr * w + nc
                if closed[nxt]:
                    continue
                ng = base_cost + cost
                if ng < g[nxt] - 1e-9:
                    g[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(heap, (ng + self._heuristic(nc, nr, gc, gr),
                                          counter, nxt))
                    counter += 1
        if not found:
            return None
        cells: List[Tuple[int, int]] = []
        cur = goal_idx
        while cur != -1:
            r, c = divmod(cur, w)
            cells.append((c, r))
            cur = int(parent[cur])
        cells.reverse()
        pts = [self.cell_to_world(c, r) for c, r in cells]
        return self._simplify(pts)

    # ------------------------------------------------------------ smoothing
    def _line_free(self, a: Sequence[float], b: Sequence[float]) -> bool:
        """Line-of-sight check on the inflated grid (supercover, no corner cut)."""
        c0, r0 = self.world_to_cell(a[0], a[1])
        c1, r1 = self.world_to_cell(b[0], b[1])
        prev = None
        for c, r in bresenham(c0, r0, c1, r1):
            if self.grid[r, c]:
                return False
            if prev is not None:
                dc = c - prev[0]
                dr = r - prev[1]
                if dc != 0 and dr != 0:
                    if self.grid[prev[1], c] or self.grid[r, prev[0]]:
                        return False
            prev = (c, r)
        return True

    def _simplify(self, pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Greedy line-of-sight shortcutting of a dense grid path."""
        if len(pts) <= 2:
            return list(pts)
        out = [pts[0]]
        i = 0
        n = len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not self._line_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    # --------------------------------------------------------------- metrics
    @staticmethod
    def path_length(path: Sequence[Sequence[float]]) -> float:
        total = 0.0
        for a, b in zip(path[:-1], path[1:]):
            total += math.hypot(b[0] - a[0], b[1] - a[1])
        return float(total)


class DWAPlanner:
    """Dynamic-window local planner for the Ackermann vehicle.

    Samples a grid of ``(v, delta)`` pairs, rolls the kinematic bicycle model
    (same understeer term as :class:`sim.RoadSim`) for ``horizon`` seconds,
    rejects rollouts that come closer than ``safe_radius`` to a lidar hit or
    that leave the road, then returns ``(throttle, steer)``.
    """

    def __init__(self, max_speed: float = C.MAX_SPEED,
                 max_steer: float = C.MAX_STEER,
                 horizon: float = 1.4, dt: float = 0.1,
                 safe_radius: float = C.CAR_RADIUS + 0.15,
                 scene=None) -> None:
        self.max_speed = float(max_speed)
        self.max_steer = float(max_steer)
        self.horizon = float(horizon)
        self.dt = float(dt)
        self.safe_radius = float(safe_radius)
        self.scene = scene
        self.speeds = np.linspace(0.0, self.max_speed, 6)
        self.steers = np.linspace(-self.max_steer, self.max_steer, 9)
        self._steps = max(2, int(round(self.horizon / self.dt)))
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._prev_dt = 0.05
        self.last_speed = 0.0
        self.last_steer = 0.0

    # --------------------------------------------------------------- rollout
    def _rollout(self, v: float, steer_angle: float,
                 x: float, y: float, yaw: float) -> np.ndarray:
        steps = self._steps
        dt = self.dt
        traj = np.empty((steps, 2), dtype=np.float64)
        wheelbase = float(C.WHEELBASE)
        under = float(C.UNDERSTEER)
        px, py, pyaw = x, y, yaw
        for k in range(steps):
            yaw_rate = (v / wheelbase) * math.tan(steer_angle) / (
                1.0 + under * v * v)
            pyaw = wrap_angle(pyaw + yaw_rate * dt)
            px += v * math.cos(pyaw) * dt
            py += v * math.sin(pyaw) * dt
            traj[k, 0] = px
            traj[k, 1] = py
        return traj

    def _estimate_speed(self, x: float, y: float, dt: float) -> float:
        if self._prev_xy is None:
            self._prev_xy = (x, y)
            return 0.0
        px, py = self._prev_xy
        v = math.hypot(x - px, y - py) / max(dt, 1e-6)
        self._prev_xy = (x, y)
        return v

    # ---------------------------------------------------------------- public
    def compute(self, pose: Sequence[float], goal_xy: Sequence[float],
                scan: Sequence[float], beam_angles: Sequence[float],
                speed: Optional[float] = None,
                dt: float = 0.05) -> Tuple[float, float]:
        """Return ``(throttle, steer)`` in the sim's [-1, 1] units."""
        pose = np.asarray(pose, dtype=np.float64).ravel()
        x, y = float(pose[0]), float(pose[1])
        if pose.size >= 4:
            yaw = float(pose[3])
        elif pose.size >= 3:
            yaw = float(pose[2])
        else:
            yaw = 0.0
        gx, gy = float(goal_xy[0]), float(goal_xy[1])
        if speed is None:
            speed = self._estimate_speed(x, y, dt)
        speed = float(speed)

        pts = _scan_points(pose, scan, beam_angles)
        d_now = math.hypot(gx - x, gy - y)
        best_score = -1e9
        best = (0.0, 0.0)
        for v in self.speeds:
            for steer_angle in self.steers:
                traj = self._rollout(float(v), float(steer_angle), x, y, yaw)
                if pts.shape[0]:
                    d = np.hypot(traj[:, 0][:, None] - pts[None, :, 0],
                                 traj[:, 1][:, None] - pts[None, :, 1])
                    min_d = float(d.min())
                else:
                    min_d = _RANGE_MAX
                if min_d < self.safe_radius:
                    continue
                if self.scene is not None:
                    curbed = False
                    for tx, ty in traj:
                        if not self.scene.is_drivable(tx, ty):
                            curbed = True
                            break
                    if curbed:
                        continue
                ex, ey = traj[-1]
                ang_goal = math.atan2(gy - ey, gx - ex)
                head = math.cos(wrap_angle(ang_goal - yaw))
                remain = math.hypot(gx - ex, gy - ey)
                progress = (d_now - remain) / max(self.max_speed * self.horizon,
                                                  1e-6)
                score = (1.6 * head
                         + 1.0 * math.tanh(min_d / max(self.safe_radius, 0.5))
                         + 0.8 * progress
                         + 0.4 * (v / max(self.max_speed, 1e-6)))
                score -= 0.20 * abs(steer_angle) / max(self.max_steer, 1e-6)
                if score > best_score:
                    best_score = score
                    best = (float(v), float(steer_angle))
        if best_score <= -1e8:
            return self._escape(pose, scan, beam_angles, speed)
        v_best, steer_best = best
        self.last_speed = v_best
        self.last_steer = steer_best
        throttle = float(np.clip((v_best - speed) / 2.0, -1.0, 1.0))
        steer = float(np.clip(steer_best / self.max_steer, -1.0, 1.0))
        return throttle, steer

    def _escape(self, pose: np.ndarray, scan: Sequence[float],
                beam_angles: Sequence[float],
                speed: float) -> Tuple[float, float]:
        ranges, beams = _clean_scan(scan, beam_angles)
        if ranges.size == 0:
            return -1.0, 0.0
        front = np.abs(beams) <= _FRONT_HALF_FOV
        if not np.any(front):
            return -1.0, 0.0
        k = int(np.argmax(np.where(front, ranges, -1.0)))
        steer_angle = float(np.clip(0.9 * float(beams[k]),
                                    -self.max_steer, self.max_steer))
        throttle = -1.0 if float(np.min(ranges[front])) < 3.0 else 0.0
        self.last_speed = 0.0
        self.last_steer = steer_angle
        return float(throttle), float(steer_angle / self.max_steer)


class GeometricPlanner:
    """Goal-attraction / obstacle-repulsion fallback controller."""

    def __init__(self, max_speed: float = C.MAX_SPEED,
                 max_steer: float = C.MAX_STEER, scene=None) -> None:
        self.max_speed = float(max_speed)
        self.max_steer = float(max_steer)
        self.scene = scene
        self.safe_radius = float(C.CAR_RADIUS) + 0.6
        self._prev_xy: Optional[Tuple[float, float]] = None

    def compute(self, pose: Sequence[float], goal_xy: Sequence[float],
                scan: Sequence[float], beam_angles: Sequence[float],
                speed: Optional[float] = None,
                dt: float = 0.05) -> Tuple[float, float]:
        pose = np.asarray(pose, dtype=np.float64).ravel()
        x, y = float(pose[0]), float(pose[1])
        if pose.size >= 4:
            yaw = float(pose[3])
        elif pose.size >= 3:
            yaw = float(pose[2])
        else:
            yaw = 0.0
        gx, gy = float(goal_xy[0]), float(goal_xy[1])
        if speed is None:
            if self._prev_xy is None:
                speed = 0.0
            else:
                speed = math.hypot(x - self._prev_xy[0],
                                   y - self._prev_xy[1]) / max(dt, 1e-6)
        self._prev_xy = (x, y)

        alpha = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
        steer_angle = float(np.clip(1.5 * alpha, -self.max_steer,
                                    self.max_steer))
        ranges, beams = _clean_scan(scan, beam_angles)
        target = self.max_speed * max(0.0, 1.0 - abs(alpha) / math.pi)
        if ranges.size:
            front = np.abs(beams) <= _FRONT_HALF_FOV
            if np.any(front):
                clearance = float(np.min(ranges[front]))
                if clearance < self.safe_radius + 4.0:
                    target *= max(0.0, (clearance - self.safe_radius) / 4.0)
                repulse = 0.0
                for rng, beam in zip(ranges[front], beams[front]):
                    if rng < self.safe_radius + 1.5:
                        repulse -= math.copysign(1.0, beam) * \
                            (self.safe_radius + 1.5 - rng)
                steer_angle = float(np.clip(
                    steer_angle + 0.4 * repulse, -self.max_steer,
                    self.max_steer))
        throttle = float(np.clip((target - float(speed)) / 2.0, -1.0, 1.0))
        return throttle, float(steer_angle / self.max_steer)
