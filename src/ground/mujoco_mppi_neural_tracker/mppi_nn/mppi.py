"""MPPI（Model Predictive Path Integral）采样式 MPC。

每个控制周期采样 K 条带时间相关的控制噪声序列，用**运动学自行车模型**前向推演 H 步，
按代价做指数加权平均得到当前控制量：

$$
w_i = \\frac{\\exp(-S_i/\\lambda)}{\\sum_j \\exp(-S_j/\\lambda)},\\qquad
u_t = \\sum_i w_i\\, u_t^{(i)}
$$

代价由横向误差、航向误差、速度误差与控制平滑组成。推演用的是简化模型（不含轮胎侧偏），
所以每步只要几毫秒——这也正是后面要用神经网络"蒸馏"它的原因。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .env import WHEELBASE, DT
from . import track


@dataclass
class MPPIConfig:
    horizon: int = 25
    samples: int = 512
    lam: float = 0.8
    noise_delta: float = 0.35
    noise_v: float = 0.45
    w_e: float = 8.0
    w_psi: float = 2.0
    w_v: float = 1.5
    w_du: float = 0.6
    v_ref: float = 0.9
    v_min: float = 0.35
    v_max: float = 1.6
    delta_max: float = 0.6


class MPPIController:
    def __init__(self, path: np.ndarray, cfg: MPPIConfig | None = None, seed: int = 0):
        self.cfg = cfg or MPPIConfig()
        self.path = path
        self.rng = np.random.default_rng(seed)
        H = self.cfg.horizon
        self.u = np.zeros((H, 2))
        self.u[:, 0] = self.cfg.v_ref

    # ------------------------------------------------------------------
    def _cost(self, idx0: int, x, y, th, v, delta):
        """x/y/th/v/delta 都是 (K, H) 的滚动轨迹。"""
        cfg = self.cfg
        n = len(self.path)
        idx = np.full(x.shape[0], idx0, dtype=np.int64)
        cost = np.zeros(x.shape[0])
        for h in range(x.shape[1]):
            # 只在 ±6 个参考点上找最近点，避免 8 字自交处跳段
            cand = (idx[:, None] + np.arange(-6, 7)[None, :]) % n
            d = (self.path[cand][:, :, 0] - x[:, h][:, None]) ** 2 + \
                (self.path[cand][:, :, 1] - y[:, h][:, None]) ** 2
            best = d.argmin(axis=1)
            idx = cand[np.arange(x.shape[0]), best]
            p = self.path[idx]
            nxt = self.path[(idx + 1) % n]
            t = nxt - p
            t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-12
            nrm = np.stack([-t[:, 1], t[:, 0]], axis=1)
            e = nrm[:, 0] * (x[:, h] - p[:, 0]) + nrm[:, 1] * (y[:, h] - p[:, 1])
            head = np.stack([np.cos(th[:, h]), np.sin(th[:, h])], axis=1)
            psi = np.arctan2(t[:, 0] * head[:, 1] - t[:, 1] * head[:, 0],
                             (t * head).sum(axis=1))
            cost += cfg.w_e * e ** 2 + cfg.w_psi * (1 - np.cos(psi)) + \
                cfg.w_v * (v[:, h] - cfg.v_ref) ** 2
        cost += cfg.w_du * ((np.diff(delta, axis=1)) ** 2).sum(axis=1)
        return cost

    def act(self, pose, v_now: float = 0.9) -> tuple[float, float]:
        cfg = self.cfg
        K, H = cfg.samples, cfg.horizon
        x0, y0, th0 = pose
        idx0 = track.nearest_index(self.path, (x0, y0))

        # 带时间相关的噪声：噪声本身是一阶平滑的，轨迹才不像锯齿
        eps_d = np.cumsum(self.rng.normal(0, 1, (K, H)), axis=1) * cfg.noise_delta / np.sqrt(H)
        eps_v = np.cumsum(self.rng.normal(0, 1, (K, H)), axis=1) * cfg.noise_v / np.sqrt(H)
        delta = np.clip(self.u[None, :, 1] + eps_d, -cfg.delta_max, cfg.delta_max)
        vel = np.clip(self.u[None, :, 0] + eps_v, cfg.v_min, cfg.v_max)

        x = np.empty((K, H))
        y = np.empty((K, H))
        th = np.empty((K, H))
        vv = np.empty((K, H))
        xi, yi, ti = np.full(K, x0), np.full(K, y0), np.full(K, th0)
        for h in range(H):
            ti = ti + vel[:, h] * np.tan(delta[:, h]) / WHEELBASE * DT
            xi = xi + vel[:, h] * np.cos(ti) * DT
            yi = yi + vel[:, h] * np.sin(ti) * DT
            x[:, h], y[:, h], th[:, h], vv[:, h] = xi, yi, ti, vel[:, h]

        cost = self._cost(idx0, x, y, th, vv, delta)
        cost -= cost.min()
        w = np.exp(-cost / cfg.lam)
        w /= w.sum() + 1e-12

        u0 = np.array([float((w * vel[:, 0]).sum()), float((w * delta[:, 0]).sum())])
        # 滚动：把加权后的整段序列往前挪一格
        seq = np.stack([(w[:, None] * vel).sum(axis=0),
                        (w[:, None] * delta).sum(axis=0)], axis=1)
        self.u = np.vstack([seq[1:], seq[-1:]])
        self.last_cost = float(cost.min())
        return float(u0[0]), float(u0[1])


def pure_pursuit(pose, path, idx: int, ld: float = 1.1, v_ref: float = 0.9,
                 wheelbase: float = WHEELBASE) -> tuple[float, float]:
    """几何基线：追前视点。"""
    x, y, yaw = pose
    n = len(path)
    # 按弧长找前视点（点距不均匀，不能假设 5 cm）
    step = float(np.mean(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    tgt = path[(idx + max(1, int(round(ld / max(step, 1e-6))))) % n]
    dx, dy = tgt[0] - x, tgt[1] - y
    alpha = np.arctan2(dy, dx) - yaw
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
    dist = np.hypot(dx, dy)
    delta = np.arctan2(2 * wheelbase * np.sin(alpha), max(dist, 1e-6))
    return v_ref, float(np.clip(delta, -0.6, 0.6))
