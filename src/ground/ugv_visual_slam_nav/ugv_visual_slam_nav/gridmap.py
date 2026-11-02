"""占据栅格建图：对数几率逆传感器模型。

传感器模型：视觉深度点 → 射线沿途标"空"，端点标"占据"。
真值地图用于计算探索区内的 IoU。
"""

import math

import numpy as np

from . import params as P

N_STEP = 140          # 每条射线的采样点数


class GridMap:
    def __init__(self):
        self.lo = np.zeros((P.GH, P.GW), dtype=np.float32)

    # ------------------------------------------------------------ 更新
    def update(self, pose, wall_pts):
        """pose: (x,y,theta) 用 SLAM 估计位姿；wall_pts: (K,2) (bearing, range)。"""
        if len(wall_pts) == 0:
            return
        x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
        b = wall_pts[:, 0]
        r = wall_pts[:, 1]
        a = th + b
        px = x + r * np.cos(a)
        py = y + r * np.sin(a)

        t = np.linspace(0.02, 1.0, N_STEP).astype(np.float32)[None, :]
        xs = x + (px - x)[:, None] * t
        ys = y + (py - y)[:, None] * t

        cols = np.floor(xs / P.GRID_RES).astype(np.int32)
        rows = np.floor(ys / P.GRID_RES).astype(np.int32)
        ok = (rows >= 0) & (rows < P.GH) & (cols >= 0) & (cols < P.GW)

        # 射线中部：标记为空
        rf, cf = rows[:, :-1], cols[:, :-1]
        okf = (rf >= 0) & (rf < P.GH) & (cf >= 0) & (cf < P.GW)
        np.add.at(self.lo, (rf[okf], cf[okf]), P.LOGODDS_MISS)
        # 端点：标记为占据（在 miss 之上再补一次净增量）
        re, ce = rows[:, -1], cols[:, -1]
        oke = (re >= 0) & (re < P.GH) & (ce >= 0) & (ce < P.GW)
        np.add.at(self.lo, (re[oke], ce[oke]), P.LOGODDS_HIT - P.LOGODDS_MISS)

        np.clip(self.lo, P.LOGODDS_MIN, P.LOGODDS_MAX, out=self.lo)

    # ------------------------------------------------------------ 查询
    def mark_obstacle(self, x, y, radius=0.12):
        """把 SLAM 估计出的路标位置写进栅格（路标是真实障碍物）。"""
        c = int(x / P.GRID_RES)
        r = int(y / P.GRID_RES)
        k = max(1, int(math.ceil(radius / P.GRID_RES)))
        r0, r1 = max(0, r - k), min(P.GH, r + k + 1)
        c0, c1 = max(0, c - k), min(P.GW, c + k + 1)
        if r0 >= r1 or c0 >= c1:
            return
        blk = self.lo[r0:r1, c0:c1]
        np.maximum(blk, P.LOGODDS_HIT, out=blk)

    def prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.lo))

    def occupied(self, th=0.62):
        return self.prob() > th

    def explored(self):
        return np.abs(self.lo) > 1e-6

    def free_mask(self):
        return self.prob() < 0.38

    def counts(self):
        return int(self.occupied().sum()), int(self.explored().sum())

    # ------------------------------------------------------------ 评价
    def iou(self, truth):
        """只在已探索区域内算 IoU，避免未探索区把所有分数拉平。"""
        est = self.occupied()
        exp = self.explored()
        if not exp.any():
            return 0.0
        tp = float((est & truth & exp).sum())
        fp = float((est & ~truth & exp).sum())
        fn = float((~est & truth & exp).sum())
        den = tp + fp + fn
        return 1.0 if den == 0 else tp / den

    def coverage(self):
        return float(self.explored().sum()) / float(P.GW * P.GH)


def truth_grid(world):
    """真值占据栅格：墙体/障碍覆盖到的格子。"""
    m = np.zeros((P.GH, P.GW), dtype=bool)
    for x0, y0, x1, y1 in world.rects:
        c0 = max(0, int(math.floor(x0 / P.GRID_RES)))
        c1 = min(P.GW - 1, int(math.ceil(x1 / P.GRID_RES)))
        r0 = max(0, int(math.floor(y0 / P.GRID_RES)))
        r1 = min(P.GH - 1, int(math.ceil(y1 / P.GRID_RES)))
        m[r0:r1 + 1, c0:c1 + 1] = True
    for cx, cy, r in world.circles():
        c0 = max(0, int(math.floor((cx - r) / P.GRID_RES)))
        c1 = min(P.GW - 1, int(math.ceil((cx + r) / P.GRID_RES)))
        r0 = max(0, int(math.floor((cy - r) / P.GRID_RES)))
        r1 = min(P.GH - 1, int(math.ceil((cy + r) / P.GRID_RES)))
        for rr in range(r0, r1 + 1):
            for cc in range(c0, c1 + 1):
                wx = (cc + 0.5) * P.GRID_RES
                wy = (rr + 0.5) * P.GRID_RES
                if (wx - cx) ** 2 + (wy - cy) ** 2 <= r * r:
                    m[rr, cc] = True
    return m
