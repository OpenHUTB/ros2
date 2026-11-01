"""占据栅格建图与相关扫描匹配 SLAM。

地图用对数几率（log-odds）表示，命中 +0.85、穿过 -0.40，概率
``p = 1 / (1 + exp(-l))``，比直接累加概率数值更稳定。
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

L_HIT = 0.85
L_MISS = -0.40
L_MIN = -4.0
L_MAX = 4.0


class OccupancyGrid:
    """二维对数几率占据栅格。"""

    def __init__(self, size_x: float = 12.0, size_y: float = 8.0, res: float = 0.05,
                 origin: Optional[Tuple[float, float]] = None):
        self.res = float(res)
        self.nx = int(round(size_x / res))
        self.ny = int(round(size_y / res))
        self.origin = np.array(origin if origin is not None else
                               (-size_x / 2.0, -size_y / 2.0), dtype=float)
        self.logodds = np.zeros((self.ny, self.nx), dtype=np.float32)

    # ------------------------------------------------------------------
    def world_to_cell(self, x, y):
        cx = ((np.asarray(x) - self.origin[0]) / self.res).astype(np.int32)
        cy = ((np.asarray(y) - self.origin[1]) / self.res).astype(np.int32)
        return cx, cy

    def cell_to_world(self, cx, cy):
        return (self.origin[0] + (np.asarray(cx) + 0.5) * self.res,
                self.origin[1] + (np.asarray(cy) + 0.5) * self.res)

    def prob(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.logodds))

    def occupied(self, thresh: float = 0.65) -> np.ndarray:
        return self.prob() > thresh

    # ------------------------------------------------------------------
    def integrate(self, pose, ranges, angles, max_range: float = 8.0,
                  hit_weight: float = 1.0, miss_weight: float = 1.0) -> None:
        """把一帧激光雷达写入地图（Bresenham 走线）。"""
        x0, y0, th = pose
        r = np.asarray(ranges, dtype=np.float64)
        a = th + np.asarray(angles, dtype=np.float64)
        valid = r < (max_range - 1e-3)
        ex = x0 + r * np.cos(a)
        ey = y0 + r * np.sin(a)

        # 每条射线按 res/2 采样，标记自由空间
        step = self.res * 0.5
        n_steps = int(max_range / step) + 1
        t = np.arange(1, n_steps, dtype=np.float64) * step
        px = x0 + np.outer(np.cos(a), t)
        py = y0 + np.outer(np.sin(a), t)
        dist_from_start = np.outer(np.ones_like(a), t)
        free_mask = dist_from_start < (r[:, None] - self.res)
        cx, cy = self.world_to_cell(px[free_mask], py[free_mask])
        self._bump(cx, cy, L_MISS * miss_weight)

        # 命中点标记占据
        hx, hy = self.world_to_cell(ex[valid], ey[valid])
        self._bump(hx, hy, L_HIT * hit_weight)

    def _bump(self, cx, cy, delta):
        ok = (cx >= 0) & (cx < self.nx) & (cy >= 0) & (cy < self.ny)
        if not np.any(ok):
            return
        flat = cy[ok] * self.nx + cx[ok]
        np.add.at(self.logodds.reshape(-1), flat, delta)
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)

    # ------------------------------------------------------------------
    def coverage(self) -> float:
        """已知（非未知）栅格占可通行区域的比例。"""
        known = np.abs(self.logodds) > 0.2
        return float(known.mean())

    def as_image(self) -> np.ndarray:
        """0=自由 1=占据 0.5=未知 的灰度图，供可视化。"""
        p = self.prob()
        img = np.where(np.abs(self.logodds) < 0.2, 0.5, 1.0 - p)
        return np.clip(img, 0, 1)


class ScanMatcher:
    """相关扫描匹配（correlative scan matching）。

    在里程计给出的预测位姿附近，搜索 (dx, dy, dθ) 使"命中点落在已知占据栅格"
    的得分最大。搜索是粗到细两级，单帧约 3~8 ms（180 线，CPU）。
    """

    def __init__(self, grid: OccupancyGrid, xy_window: float = 0.12,
                 th_window: float = 0.12, coarse: int = 5, fine: int = 5):
        self.grid = grid
        self.xy_window = xy_window
        self.th_window = th_window
        self.coarse = coarse
        self.fine = fine

    def _score(self, pts, pose) -> np.ndarray:
        """pts: (N,2) 局部坐标；pose: (C,3) 候选位姿 -> (C,) 得分。"""
        c, s = np.cos(pose[:, 2])[:, None], np.sin(pose[:, 2])[:, None]
        wx = pose[:, 0:1] + c * pts[None, :, 0] - s * pts[None, :, 1]
        wy = pose[:, 1:2] + s * pts[None, :, 0] + c * pts[None, :, 1]
        cx, cy = self.grid.world_to_cell(wx, wy)
        ok = (cx >= 0) & (cx < self.grid.nx) & (cy >= 0) & (cy < self.grid.ny)
        grid = self.grid.prob()
        vals = np.where(ok, grid[np.clip(cy, 0, self.grid.ny - 1), np.clip(cx, 0, self.grid.nx - 1)],
                        0.0)
        return (vals * ok).sum(axis=1)

    def match(self, pts_local: np.ndarray, predicted) -> Tuple[np.ndarray, float]:
        px, py, pth = predicted
        best = np.array([px, py, pth], dtype=float)
        best_score = -1.0
        for stage, (win, nth, nxy) in enumerate((
                (1.0, self.coarse, self.coarse),
                (1.0 / max(self.fine - 1, 1), self.fine, self.fine))):
            ths = best[2] + np.linspace(-self.th_window * win, self.th_window * win, nth)
            xs = best[0] + np.linspace(-self.xy_window * win, self.xy_window * win, nxy)
            ys = best[1] + np.linspace(-self.xy_window * win, self.xy_window * win, nxy)
            grid_pts = np.array(np.meshgrid(xs, ys, ths, indexing="ij")).reshape(3, -1).T
            scores = self._score(pts_local, grid_pts)
            i = int(np.argmax(scores))
            if scores[i] > best_score:
                best_score = float(scores[i])
                best = grid_pts[i]
            self.xy_window *= 0.5
            self.th_window *= 0.5
        # 恢复窗口，便于下一帧使用
        self.xy_window = 0.12
        self.th_window = 0.12
        return best, best_score


class Slam:
    """里程计预测 + 扫描匹配校正 + 地图增量更新。"""

    def __init__(self, grid: Optional[OccupancyGrid] = None, max_range: float = 8.0):
        self.grid = grid or OccupancyGrid()
        self.matcher = ScanMatcher(self.grid)
        self.max_range = max_range
        self.pose = np.zeros(3)
        self.trajectory = []
        self.last_score = 0.0
        self.matched = True

    def reset(self, pose=(0.0, 0.0, 0.0)):
        self.grid.logodds[:] = 0.0
        self.pose = np.array(pose, dtype=float)
        self.trajectory = [tuple(self.pose)]

    def update(self, odom_delta, ranges, angles, guess=None) -> np.ndarray:
        """``odom_delta`` = (dx, dy, dyaw) 自上一帧以来的里程计增量。"""
        pred = self.pose + np.asarray(odom_delta, dtype=float)
        r = np.asarray(ranges, dtype=np.float64)
        a = np.asarray(angles, dtype=np.float64) + pred[2]
        valid = r < self.max_range - 1e-3
        pts = np.stack([r[valid] * np.cos(a[valid] - pred[2]),
                        r[valid] * np.sin(a[valid] - pred[2])], axis=1)
        if len(pts) < 20 or not np.any(np.abs(self.grid.logodds) > 0.2):
            new_pose, score = pred, 0.0
            self.matched = False
        else:
            new_pose, score = self.matcher.match(pts, pred)
            self.matched = True
        # 限幅，防止匹配发散
        new_pose[0] = float(np.clip(new_pose[0], pred[0] - 0.25, pred[0] + 0.25))
        new_pose[1] = float(np.clip(new_pose[1], pred[1] - 0.25, pred[1] + 0.25))
        new_pose[2] = pred[2] + math.atan2(math.sin(new_pose[2] - pred[2]),
                                           math.cos(new_pose[2] - pred[2]))
        self.pose = np.asarray(new_pose, dtype=float)
        self.last_score = float(score)
        self.grid.integrate(self.pose, ranges, angles, self.max_range)
        self.trajectory.append(tuple(self.pose))
        return self.pose.copy()


def local_patch(grid: "OccupancyGrid", pose, size: int = 21, stride: int = 2):
    """取以机器人位姿为中心、下采样后的局部占据栅格（供规划网络使用）。"""
    cx, cy = grid.world_to_cell(pose[0], pose[1])
    half = size // 2
    ys, xs = np.mgrid[-half:half + 1, -half:half + 1]
    gx = np.clip(cx + xs, 0, grid.nx - 1)
    gy = np.clip(cy + ys, 0, grid.ny - 1)
    occ = grid.occupied(0.6).astype(np.float32)
    return occ[gy, gx][::stride, ::stride]
