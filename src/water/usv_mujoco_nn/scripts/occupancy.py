"""占据栅格建图（对数几率 log-odds 更新）。

每个栅格维护 l = log(p/(1-p))：
    命中障碍    l <- l + l_occ
    穿过空闲    l <- l - l_free
概率还原 p = 1 - 1/(1+exp(l))。
"""

import numpy as np


class OccupancyMap(object):
    def __init__(self, half_extent=25.0, resolution=0.5):
        self.half = float(half_extent)
        self.res = float(resolution)
        self.n = int(round(2.0 * self.half / self.res))
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)
        self.l_occ = 0.85
        self.l_free = -0.40
        self.l_min = -6.0
        self.l_max = 6.0

    # ---------------------------------------------------------------- 坐标变换
    def world_to_cell(self, pts):
        idx = np.floor((pts + self.half) / self.res).astype(np.int32)
        return np.clip(idx, 0, self.n - 1)

    def cell_to_world(self, i, j):
        return (np.array([i, j], dtype=np.float64) + 0.5) * self.res - self.half

    # ---------------------------------------------------------------- 更新
    def update(self, pose, scan, angle_min, angle_max, max_range):
        x, y, yaw = pose
        n_beams = scan.shape[0]
        angles = yaw + np.linspace(angle_min, angle_max, n_beams)

        # 每条射线上等间隔采样点（步长取半个栅格）
        n_samples = int(max_range / (self.res * 0.5)) + 1
        frac = np.linspace(0.0, 1.0, n_samples)[None, :]           # (1, K)
        rng = np.clip(scan, 0.0, max_range)[:, None]                # (B, 1)
        dist = rng * frac                                           # (B, K)

        px = x + dist * np.cos(angles)[:, None]
        py = y + dist * np.sin(angles)[:, None]
        pts = np.stack([px, py], axis=-1).reshape(-1, 2)
        idx = self.world_to_cell(pts)

        # 命中点单独标记
        hit = np.stack([x + scan * np.cos(angles), y + scan * np.sin(angles)], axis=-1)
        hidx = self.world_to_cell(hit)

        np.add.at(self.logodds, (idx[:, 1], idx[:, 0]), self.l_free)
        np.add.at(self.logodds, (hidx[:, 1], hidx[:, 0]), self.l_occ - self.l_free)
        np.clip(self.logodds, self.l_min, self.l_max, out=self.logodds)

    # ---------------------------------------------------------------- 查询
    def probability(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds.astype(np.float64)))

    def crop(self, pose, size=21, body_frame=True):
        """取以机器人中心为原点、size×size 的局部占据概率图。

        body_frame=True 时按船体坐标系旋转采样（自车中心表示），
        使同一张栅格对任意艏向都对应相同的动作语义，便于策略学习；
        否则按世界系直接裁剪。
        """
        x, y, yaw = pose
        half = size // 2
        offs = (np.arange(size) - half) * self.res
        dx, dy = np.meshgrid(offs, offs)          # dx 为列方向，dy 为行方向

        if body_frame:
            c, s = np.cos(yaw), np.sin(yaw)
            wx = x + c * dx - s * dy
            wy = y + s * dx + c * dy
        else:
            wx = x + dx
            wy = y + dy

        idx = self.world_to_cell(np.stack([wx, wy], axis=-1).reshape(-1, 2))
        prob = self.probability()
        out = prob[idx[:, 1], idx[:, 0]].reshape(size, size)
        return out.astype(np.float32)

    def covered_ratio(self):
        """已观测栅格占比，用于评价建图覆盖。"""
        return float(np.mean(np.abs(self.logodds) > 1e-6))
