"""全局与局部路径规划：A* 栅格规划器、DWA 局部规划器、几何回退规划器。"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import bresenham, wrap_angle

__all__ = ["GridPlanner", "DWAPlanner", "GeometricPlanner"]

_SQRT2 = math.sqrt(2.0)
_MOVES = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2),
)

# beams considered "frontal" for local obstacle checks (rad)
_FRONT_HALF_FOV = math.radians(55.0)


def _clean_scan(scan: Sequence[float], beam_angles: Sequence[float],
                window: int = 2) -> Tuple[np.ndarray, np.ndarray]:
    """Drop no-return beams and isolated outliers from a sonar scan.

    Returns ``(ranges, world_angles_relative_to_heading)`` i.e. the beam
    angles themselves, with invalid samples removed.
    """
    ranges = np.asarray(scan, dtype=np.float64).ravel()
    beams = np.asarray(beam_angles, dtype=np.float64).ravel()
    n = min(ranges.size, beams.size)
    ranges, beams = ranges[:n], beams[:n]
    if n == 0:
        return ranges, beams
    # median filter over neighbouring beams rejects isolated spikes
    if n >= 5:
        padded = np.pad(ranges, window, mode="edge")
        stack = np.stack([padded[i: i + n] for i in range(2 * window + 1)], axis=0)
        med = np.median(stack, axis=0)
        outlier = (ranges > med + 0.8) | (ranges < med - 0.8)
        ranges = np.where(outlier, med, ranges)
    valid = np.isfinite(ranges) & (ranges > C.SONAR_RANGE_MIN + 0.02) \
        & (ranges < C.SONAR_RANGE_MAX * 0.995)
    return ranges[valid], beams[valid]


def _scan_points(pose: Sequence[float], scan: Sequence[float],
                 beam_angles: Sequence[float]) -> np.ndarray:
    """World-frame obstacle points from a scan, shape ``(N, 2)``."""
    pose = np.asarray(pose, dtype=np.float64).ravel()
    if pose.size < 2:
        return np.zeros((0, 2), dtype=np.float64)
    yaw = float(pose[3]) if pose.size >= 4 else (float(pose[2]) if pose.size >= 3 else 0.0)
    ranges, beams = _clean_scan(scan, beam_angles)
    if ranges.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    ang = yaw + beams
    pts = np.column_stack([pose[0] + ranges * np.cos(ang),
                           pose[1] + ranges * np.sin(ang)])
    return pts


def _inflate_circular(grid: np.ndarray, radius_cells: float) -> np.ndarray:
    """Dilate a boolean grid with a circular structuring element.

    A circular element keeps narrow diagonal gaps passable, unlike a square
    element which over-inflates the corners.  ``radius_cells`` is the radius in
    cells, so the resulting clearance is roughly ``radius_cells * resolution``.
    """
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
    """8 连通 A* 全局规划器（禁止切角，障碍物按 ROV 半径膨胀）。"""

    def __init__(self, occupancy_bool: np.ndarray,
                 resolution: float = C.PLAN_RESOLUTION,
                 inflate_cells: Optional[float] = None) -> None:
        self.resolution = float(resolution)
        base = np.asarray(occupancy_bool, dtype=bool)
        if base.ndim != 2:
            raise ValueError("occupancy_bool must be a 2-D array")
        self.base = base
        self.height, self.width = base.shape
        if inflate_cells is None:
            # 膨胀半径取 ROV 半径 + 3 cm 余量，圆形结构元避免过度堵塞窄通道
            self.inflate_radius_cells = (C.ROV_RADIUS + 0.03) / self.resolution
        else:
            self.inflate_radius_cells = max(0.0, float(inflate_cells))
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

    # ---------------------------------------------------------------- search
    def _snap(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Nearest free cell to world ``(x, y)`` (``(col, row)``) or ``None``."""
        c0, r0 = self.world_to_cell(x, y)
        if not self.grid[r0, c0]:
            return c0, r0
        for rad in range(1, 60):
            best: Optional[Tuple[int, int]] = None
            best_d = math.inf
            r_lo, r_hi = max(0, r0 - rad), min(self.height - 1, r0 + rad)
            c_lo, c_hi = max(0, c0 - rad), min(self.width - 1, c0 + rad)
            for r in range(r_lo, r_hi + 1):
                for c in range(c_lo, c_hi + 1):
                    if max(abs(c - c0), abs(r - r0)) != rad:
                        continue
                    if self.grid[r, c]:
                        continue
                    d = (c - c0) ** 2 + (r - r0) ** 2
                    if d < best_d or (d == best_d and (c, r) < best):
                        best_d = d
                        best = (c, r)
            if best is not None:
                return best
        return None

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
        w, h = self.width, self.height
        start_idx = sr * w + sc
        goal_idx = gr * w + gc
        if start_idx == goal_idx:
            return [self.cell_to_world(sc, sr)]

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
                ng = g[cur] + cost
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
        c0, r0 = self.world_to_cell(a[0], a[1])
        c1, r1 = self.world_to_cell(b[0], b[1])
        for c, r in bresenham(c0, r0, c1, r1):
            if self.grid[r, c]:
                return False
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
    """动态窗口法（DWA）局部避障规划器。

    ``compute`` 采样一组 ``(vx, wz)``，前向仿真 ``horizon`` 秒，按“朝向目标 +
    离障碍物余量 + 前进速度”打分，并拒绝预测轨迹会碰撞的候选。
    """

    def __init__(self, max_surge: float = C.MAX_SURGE,
                 max_yaw: float = C.MAX_YAW_RATE,
                 horizon: float = 1.6, dt: float = 0.1) -> None:
        self.max_surge = float(max_surge)
        self.max_yaw = float(max_yaw)
        self.horizon = float(horizon)
        self.dt = float(dt)
        self.speeds = np.linspace(0.0, self.max_surge, 7)
        self.yaw_rates = np.linspace(-self.max_yaw, self.max_yaw, 9)
        self._steps = max(2, int(round(self.horizon / self.dt)))

    def compute(self, pose: Sequence[float], goal_xy: Sequence[float],
                scan: Sequence[float],
                beam_angles: Sequence[float]) -> Tuple[float, float]:
        pose = np.asarray(pose, dtype=np.float64).ravel()
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[3]) if pose.size >= 4 else (
            float(pose[2]) if pose.size >= 3 else 0.0)
        gx, gy = float(goal_xy[0]), float(goal_xy[1])
        pts = _scan_points(pose, scan, beam_angles)
        steps = self._steps
        d_now = math.hypot(gx - x, gy - y)

        best_score = -1e9
        best = (0.0, 0.0)
        safe_cost = 0.55  # ROV radius + margin used for predictive rejection
        for vx in self.speeds:
            for wz in self.yaw_rates:
                pyaw = yaw
                px, py = x, y
                traj = np.empty((steps, 2), dtype=np.float64)
                for k in range(steps):
                    pyaw = yaw + wz * self.dt * (k + 1)
                    px += vx * self.dt * math.cos(pyaw)
                    py += vx * self.dt * math.sin(pyaw)
                    traj[k, 0], traj[k, 1] = px, py
                if pts.shape[0]:
                    d = np.hypot(traj[:, 0][:, None] - pts[None, :, 0],
                                 traj[:, 1][:, None] - pts[None, :, 1])
                    min_d = float(d.min())
                else:
                    min_d = C.SONAR_RANGE_MAX
                if min_d < safe_cost:
                    continue
                ang_goal = math.atan2(gy - py, gx - px)
                head = math.cos(wrap_angle(ang_goal - pyaw))
                remain = math.hypot(gx - px, gy - py)
                progress = (d_now - remain) / max(self.max_surge * self.horizon, 1e-6)
                score = (1.6 * head + 1.0 * math.tanh(min_d / 1.2)
                         + 0.8 * progress + 0.4 * (vx / self.max_surge))
                # prefer smoother controls a little
                score -= 0.05 * abs(wz) / self.max_yaw
                if score > best_score:
                    best_score = score
                    best = (float(vx), float(wz))
        if best_score > -1e8:
            return best
        # 所有候选都会碰撞：原地转向最开阔的方向
        return self._escape(pose, scan, beam_angles)

    def _escape(self, pose: np.ndarray, scan: Sequence[float],
                beam_angles: Sequence[float]) -> Tuple[float, float]:
        ranges, beams = _clean_scan(scan, beam_angles)
        yaw = float(pose[3]) if pose.size >= 4 else 0.0
        if ranges.size == 0:
            return 0.0, 0.0
        idx = int(np.argmax(ranges))
        target = wrap_angle(yaw + float(beams[idx]))
        return 0.0, float(np.clip(1.5 * target, -self.max_yaw, self.max_yaw))


class GeometricPlanner:
    """简单几何（目标吸引 + 障碍排斥）规划器，作为回退与数据集生成策略。"""

    def __init__(self, max_surge: float = C.MAX_SURGE,
                 max_yaw: float = C.MAX_YAW_RATE) -> None:
        self.max_surge = float(max_surge)
        self.max_yaw = float(max_yaw)

    def compute(self, pose: Sequence[float], goal_xy: Sequence[float],
                scan: Sequence[float],
                beam_angles: Sequence[float]) -> Tuple[float, float]:
        pose = np.asarray(pose, dtype=np.float64).ravel()
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[3]) if pose.size >= 4 else (
            float(pose[2]) if pose.size >= 3 else 0.0)
        gx, gy = float(goal_xy[0]), float(goal_xy[1])
        alpha = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
        wz = float(np.clip(2.0 * alpha, -self.max_yaw, self.max_yaw))

        ranges, beams = _clean_scan(scan, beam_angles)
        surge = self.max_surge * (0.20 + 0.80 * max(0.0, math.cos(alpha)))
        if ranges.size:
            front = np.abs(beams) <= math.radians(75.0)
            clearance = float(np.min(ranges[front])) if np.any(front) else C.SONAR_RANGE_MAX
            if clearance < 1.2:
                surge *= max(0.0, (clearance - 0.55) / 0.65)
            # 侧向排斥：把障碍物推向远离航向的方向
            repulse = 0.0
            for rng, beam in zip(ranges[front], beams[front]):
                if rng < 1.0:
                    repulse -= math.copysign(1.0, beam) * (1.0 - rng) * 0.8
            wz = float(np.clip(wz + 0.6 * repulse, -self.max_yaw, self.max_yaw))
        if abs(alpha) > 1.4:
            surge *= 0.4
        return float(np.clip(surge, 0.0, self.max_surge)), wz
