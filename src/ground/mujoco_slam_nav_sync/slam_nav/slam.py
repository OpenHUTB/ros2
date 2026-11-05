r"""EKF-SLAM：位姿 + 路标联合估计，外加一张对数几率占据栅格。

状态 $\boldsymbol{\mu}=[x, y, \theta, m_{1x}, m_{1y}, \dots]^\top$，
协方差 $\Sigma$ 随路标数增长；预测用轮式里程计，更新用雷达测到的
(距离, 方位) 观测：

$$
\mathbf{z} = \left[\sqrt{(m_x-x)^2+(m_y-y)^2},\ \operatorname{atan2}(m_y-y, m_x-x)-\theta\right]^\top
$$

路标位置对算法**未知**，第一次观测时按观测反投影初始化（$3\sigma$ 不确定性）。
数据关联用"预测观测 vs 实测观测"的门限匹配。
"""

from __future__ import annotations

import numpy as np

from .env import LIDAR_BEAMS, LIDAR_RANGE, PILLAR_RADIUS, wrap

GRID_RES = 0.1
GRID_X0, GRID_Y0 = -1.0, -7.0    # 栅格原点：覆盖 x∈[-1,13]、y∈[-7,7]，场地是 12x12
GRID_W, GRID_H = 140, 140
L_OCC = 0.85
L_FREE = -0.35
L_CLAMP = 3.5


class OccupancyGrid:
    """对数几率占据栅格（0.1 m/格）。"""

    def __init__(self, res: float = GRID_RES, w: int = GRID_W, h: int = GRID_H):
        self.res, self.w, self.h = res, w, h
        self.logodds = np.zeros((h, w), dtype=np.float32)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int((x - GRID_X0) / self.res), int((y - GRID_Y0) / self.res)

    @property
    def x0(self) -> float:
        """栅格左下角的世界坐标 x（ROS OccupancyGrid.info.origin 要用）。"""
        return GRID_X0

    @property
    def y0(self) -> float:
        return GRID_Y0

    def cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        return (cx * self.res + GRID_X0 + self.res / 2,
                cy * self.res + GRID_Y0 + self.res / 2)

    def update(self, pose, ranges: np.ndarray) -> None:
        """一次扫描的栅格更新（向量化：先算射线上的空闲格，再标命中格）。"""
        x, y, yaw = pose
        ang = yaw + 2.0 * np.pi * np.arange(len(ranges)) / len(ranges)
        dirs = np.stack([np.cos(ang), np.sin(ang)], axis=1)          # (B, 2)

        # 沿射线的空闲格：距离 res, 2res, ... < r
        step_d = self.res
        ks = np.arange(1, int(LIDAR_RANGE / step_d))
        dists = ks[None, :] * step_d                                  # (1, K)
        valid = dists < ranges[:, None]                               # (B, K)
        if valid.any():
            pts = np.stack([x + dirs[:, 0:1] * dists,
                            y + dirs[:, 1:2] * dists], axis=2)        # (B, K, 2)
            cx = ((pts[..., 0] - GRID_X0) / self.res).astype(np.int32)[valid]
            cy = ((pts[..., 1] - GRID_Y0) / self.res).astype(np.int32)[valid]
            ok = (cx >= 0) & (cx < self.w) & (cy >= 0) & (cy < self.h)
            np.add.at(self.logodds, (cy[ok], cx[ok]), L_FREE)
            np.clip(self.logodds, -L_CLAMP, L_CLAMP, out=self.logodds)

        hit = ranges < LIDAR_RANGE - 1e-3
        if hit.any():
            hp = np.stack([x + dirs[hit, 0] * ranges[hit],
                           y + dirs[hit, 1] * ranges[hit]], axis=1)
            cx = ((hp[:, 0] - GRID_X0) / self.res).astype(np.int32)
            cy = ((hp[:, 1] - GRID_Y0) / self.res).astype(np.int32)
            ok = (cx >= 0) & (cx < self.w) & (cy >= 0) & (cy < self.h)
            np.add.at(self.logodds, (cy[ok], cx[ok]), L_OCC)
            np.clip(self.logodds, -L_CLAMP, L_CLAMP, out=self.logodds)

    def occupied(self, thresh: float = 0.5) -> np.ndarray:
        return self.logodds > thresh

    def occupancy_for_planning(self, inflate_cells: int = 2) -> np.ndarray:
        """膨胀后的占据图：机器人半径 0.18 m ≈ 2 格。"""
        occ = self.occupied()
        out = occ.copy()
        if inflate_cells > 0:
            src = occ.copy()
            for dy in range(-inflate_cells, inflate_cells + 1):
                for dx in range(-inflate_cells, inflate_cells + 1):
                    if dx * dx + dy * dy > inflate_cells ** 2:
                        continue
                    out |= np.roll(np.roll(src, dy, axis=0), dx, axis=1)
        return out

    def known_ratio(self) -> float:
        return float((np.abs(self.logodds) > 0.3).mean())


class EKFSLAM:
    """3 维位姿 + 2N 维路标的 EKF。"""

    def __init__(self, n_landmarks: int = 6, init_pose=(0.0, 0.0, 0.0),
                 sigma_v: float = 0.25, sigma_w: float = 0.25,
                 sigma_r: float = 0.10, sigma_b: float = 0.04):
        self.n = n_landmarks
        n = 3 + 2 * self.n
        self.mu = np.zeros(n)
        self.mu[:3] = init_pose
        self.Sigma = np.eye(n) * 1e-4
        self.seen = np.zeros(self.n, dtype=bool)
        self.rng = np.random.default_rng(0)
        self.sigma_v, self.sigma_w = sigma_v, sigma_w
        self.sigma_r, self.sigma_b = sigma_r, sigma_b

    # ---------------------------------------------------------------- 预测
    def predict(self, v: float, w: float, dt: float) -> None:
        th = self.mu[2]
        F = np.eye(self.mu.size)
        self.mu[0] += v * np.cos(th) * dt
        self.mu[1] += v * np.sin(th) * dt
        self.mu[2] = wrap(th + w * dt)
        G = np.eye(self.mu.size)
        G[0, 2] = -v * np.sin(th) * dt
        G[1, 2] = v * np.cos(th) * dt
        Q = np.zeros((self.mu.size, self.mu.size))
        Q[0, 0] = (self.sigma_v * dt) ** 2
        Q[1, 1] = (self.sigma_v * dt) ** 2
        Q[2, 2] = (self.sigma_w * dt) ** 2
        self.Sigma = G @ self.Sigma @ G.T + Q

    # ---------------------------------------------------------------- 观测模型
    def _expected(self, i: int):
        mx, my = self.mu[3 + 2 * i], self.mu[4 + 2 * i]
        dx, dy = mx - self.mu[0], my - self.mu[1]
        r = float(np.hypot(dx, dy))
        b = wrap(float(np.arctan2(dy, dx)) - self.mu[2])
        return r, b

    def observe(self, detections) -> int:
        """detections: [(range, bearing), ...]，返回成功更新的条数。"""
        used = 0
        for r_meas, b_meas in detections:
            best, best_d = None, 1e9
            for i in range(self.n):
                if not self.seen[i]:
                    continue
                r_pred, b_pred = self._expected(i)
                d = (r_pred - r_meas) ** 2 + (2.0 * wrap(b_pred - b_meas)) ** 2
                if d < best_d:
                    best, best_d = i, d
            gate = 0.6 ** 2 + (2.0 * 0.35) ** 2
            if best is not None and best_d < gate:
                self._update(best, r_meas, b_meas)
                used += 1
            else:
                free = np.nonzero(~self.seen)[0]
                if free.size:
                    self._init_landmark(int(free[0]), r_meas, b_meas)
        return used

    def _init_landmark(self, i: int, r: float, b: float) -> None:
        th = self.mu[2]
        x = self.mu[0] + r * np.cos(th + b)
        y = self.mu[1] + r * np.sin(th + b)
        kx, ky = 3 + 2 * i, 4 + 2 * i
        self.mu[kx], self.mu[ky] = x, y
        self.seen[i] = True
        # 反投影的不确定性：距离/方位噪声传过去（2x2 雅可比）
        c, s = np.cos(th + b), np.sin(th + b)
        J = np.array([[c, -r * s],
                      [s, r * c]])
        R = np.diag([self.sigma_r ** 2, (r * self.sigma_b) ** 2])
        cov = J @ R @ J.T
        self.Sigma[kx, kx] += cov[0, 0]
        self.Sigma[ky, ky] += cov[1, 1]

    def _update(self, i: int, r_meas: float, b_meas: float) -> None:
        r_pred, b_pred = self._expected(i)
        dx, dy = self.mu[3 + 2 * i] - self.mu[0], self.mu[4 + 2 * i] - self.mu[1]
        q = dx * dx + dy * dy
        if q < 1e-9:
            return
        r = np.sqrt(q)
        H = np.zeros((2, self.mu.size))
        H[0, 0] = -dx / r
        H[0, 1] = -dy / r
        H[0, 3 + 2 * i] = dx / r
        H[0, 4 + 2 * i] = dy / r
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        H[1, 3 + 2 * i] = -dy / q
        H[1, 4 + 2 * i] = dx / q
        R = np.diag([self.sigma_r ** 2, self.sigma_b ** 2])
        S = H @ self.Sigma @ H.T + R
        innov = np.array([r_meas - r_pred, wrap(b_meas - b_pred)])
        # 卡方门限（2 自由度，99.9% -> 13.8）：拒绝错误关联，防止路标把位姿拉飞
        if float(innov @ np.linalg.solve(S, innov)) > 13.8:
            return
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ innov
        self.mu[2] = wrap(self.mu[2])
        I_KH = np.eye(self.mu.size) - K @ H
        self.Sigma = I_KH @ self.Sigma @ I_KH.T + K @ R @ K.T

    # ---------------------------------------------------------------- 输出
    @property
    def pose(self) -> np.ndarray:
        return np.array([self.mu[0], self.mu[1], wrap(self.mu[2])])

    def landmarks_estimate(self) -> np.ndarray:
        return np.array([[self.mu[3 + 2 * i], self.mu[4 + 2 * i]] for i in range(self.n)])


def detect_landmarks(ranges: np.ndarray) -> list[tuple[float, float]]:
    """把雷达量程按笛卡尔距离聚类，挑出"紧凑"簇作为路标观测 (range, bearing)。

    柱子直径 0.44 m，在雷达上只有几个点、空间展布 < 0.6 m；墙和隔断则会连成
    很长的链（展布几米）。因此用「簇的空间展布」区分路标与墙面，比按射线条数
    更稳（墙面斜着看也会出现只有 1-2 条射线的短段）。
    """
    n = len(ranges)
    pts = []
    for k, r in enumerate(ranges):
        if r >= LIDAR_RANGE - 1e-3:
            continue
        ang = 2.0 * np.pi * k / n
        pts.append((r * np.cos(ang), r * np.sin(ang), r, ang))
    if not pts:
        return []

    clusters, cur = [], [pts[0]]
    for prev, item in zip(pts, pts[1:]):
        gap = np.hypot(item[0] - prev[0], item[1] - prev[1])
        if gap < 0.25:
            cur.append(item)
        else:
            clusters.append(cur)
            cur = [item]
    clusters.append(cur)

    dets = []
    for c in clusters:
        if len(c) < 1 or len(c) > 8:
            continue
        xs = np.array([p[0] for p in c])
        ys = np.array([p[1] for p in c])
        extent = np.hypot(xs.max() - xs.min(), ys.max() - ys.min())
        r_mean = float(np.mean([p[2] for p in c]))
        if 0.05 < extent < 0.7 and r_mean < 5.0:
            ang = float(np.arctan2(ys.mean(), xs.mean()))
            # 雷达打到的是柱面，加回半径得到路标中心距离
            dets.append((float(np.hypot(xs.mean(), ys.mean())) + PILLAR_RADIUS, wrap(ang)))
    return dets
