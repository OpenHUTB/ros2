#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EKF-SLAM：把机器人位姿与全部路标放进同一个状态向量做联合估计。

    状态  μ = [x, y, θ, m₁ₓ, m₁ᵧ, …, m_Mₓ, m_My]

* **预测**只推进位姿；位姿与路标的互协方差一并传递
* **观测更新**用联合协方差算卡尔曼增益，路标随位姿一起修正
* **新路标初始化**传播位姿不确定度，补上互协方差

地图规模 30 个路标 → 62×62 的协方差，numpy 下每帧不足 1 ms。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

__all__ = ["EkfConfig", "EkfSlam"]


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


@dataclass
class EkfConfig:
    #: 里程计过程噪声：每帧（0.1 s）的位移与偏航
    sigma_dp: float = 0.05          # m
    sigma_dyaw: float = 0.015       # rad
    #: 观测噪声基准（学习式观测模型会覆盖它）
    sigma_r: float = 0.10           # m
    sigma_b: float = 0.030          # rad
    #: 数据关联门限：2 自由度卡方 99.5% 分位
    gate: float = 10.6
    #: 关联用的固定噪声（0 表示门限直接用观测噪声 R）。非 0 时门限与 R 解耦，
    #: 这样"换观测噪声模型"不会顺带改变关联行为，对比才是受控的。
    gate_sigma_r: float = 0.15
    gate_sigma_b: float = 0.045
    #: 新路标的额外膨胀（防止同一个柱子被反复初始化）
    new_landmark_inflate: float = 2.0
    gps_sigma: float = 0.5


class EkfSlam:
    """单个 EKF 承载全图。可选传入 ``obs_model`` 用学习到的观测噪声。"""

    def __init__(self, cfg: Optional[EkfConfig] = None, obs_model=None):
        self.cfg = cfg or EkfConfig()
        self.obs_model = obs_model
        self.reset()

    def reset(self, x: float = 0.0, y: float = 0.0, th: float = 0.0) -> None:
        self.mu = np.array([x, y, th])
        self.Sigma = np.eye(3) * 1e-6
        self.n_lm = 0
        # 只由 record() 追加，与帧数一一对应。构造函数里塞一个初始位姿会让
        # 估计轨迹相对真值整体错开一帧（1.4 m/s 巡航下 0.14 m 的系统滞后）。
        self.traj: List[np.ndarray] = []
        self.landmark_seen = np.zeros(0, dtype=int)
        self.last_innov = np.zeros(0)
        self.n_updates = 0
        self.n_new = 0

    # ------------------------------------------------------------ 预测
    def predict(self, dp_b: np.ndarray, dyaw: float,
                sigma: Optional[np.ndarray] = None) -> None:
        """``dp_b`` 为机体系位移增量；``sigma`` 可给出逐帧不确定度（1σ）。"""
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        c, s = math.cos(th), math.sin(th)
        dx, dy = float(dp_b[0]), float(dp_b[1])
        # 均值走**非线性**转移，雅可比只用于协方差 —— 把 F 乘到 μ 上是一阶近似，
        # 单帧 0.17 m 的位移看不出来，但整条轨迹累积下来会引入系统性偏差
        self.mu[0] = x + c * dx - s * dy
        self.mu[1] = y + s * dx + c * dy
        self.mu[2] = float(wrap(th + dyaw))
        F = np.eye(len(self.mu))
        F[0, 2] = -s * dx - c * dy
        F[1, 2] = c * dx - s * dy

        q = np.zeros((len(self.mu), len(self.mu)))
        sr, sa = self.cfg.sigma_dp, self.cfg.sigma_dyaw
        if sigma is not None:
            sr = float(max(sigma[0], 1e-4))
            sa = float(max(sigma[2], 1e-5))
        q[0, 0] = q[1, 1] = sr ** 2
        q[2, 2] = sa ** 2
        self.Sigma = F @ self.Sigma @ F.T + q

    def record(self) -> None:
        """一个时间步的全部更新做完后调用，把当前位姿记进轨迹。

        不能在 predict 里记：那时观测更新还没做，记下来的是"只积分、没修正"
        的位姿，误差会比实际大一大截。
        """
        self.traj.append(self.mu[:3].copy())

    # ------------------------------------------------------------ 观测更新
    def _predict_obs(self, j: int) -> Tuple[float, float, np.ndarray, np.ndarray]:
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        mx, my = self.mu[3 + 2 * j], self.mu[3 + 2 * j + 1]
        dx, dy = mx - x, my - y
        q = dx * dx + dy * dy
        r = math.sqrt(max(q, 1e-9))
        b = float(wrap(math.atan2(dy, dx) - th))
        H = np.zeros((2, len(self.mu)))
        H[0, 0] = -dx / r
        H[0, 1] = -dy / r
        H[0, 3 + 2 * j] = dx / r
        H[0, 3 + 2 * j + 1] = dy / r
        H[1, 0] = dy / max(q, 1e-9)
        H[1, 1] = -dx / max(q, 1e-9)
        H[1, 2] = -1.0
        H[1, 3 + 2 * j] = -dy / max(q, 1e-9)
        H[1, 3 + 2 * j + 1] = dx / max(q, 1e-9)
        return r, b, H, np.array([r, b])

    def measurement_noise(self, r: float, b: float) -> np.ndarray:
        """观测噪声 ``R``。学习式模型在这里接管，否则用常数。"""
        if self.obs_model is not None:
            sr, sb = self.obs_model.predict(r, b)
        else:
            sr, sb = self.cfg.sigma_r, self.cfg.sigma_b
        return np.diag([sr ** 2, sb ** 2])

    def update_landmarks(self, obs_r, obs_b, n_obs: int) -> int:
        """逐条观测做数据关联 + 更新；未关联上的初始化为新路标。返回新增路标数。

        ``R`` 按**观测到的** (r, b) 算一次，而不是对每个候选路标都按预测值算 ——
        学习式观测噪声每次调用要跑一遍网络，按候选逐个算会让整个滤波器慢 30 倍
        （实测 3 s → 96 s）。噪声随距离变化本就是观测几何的性质，
        用观测值代入在物理上同样成立。
        """
        added = 0
        for k in range(int(n_obs)):
            r, b = float(obs_r[k]), float(obs_b[k])
            R = self.measurement_noise(r, b)
            j, d2 = self._associate(r, b, R)
            if j is None:
                self._init_landmark(r, b, R)
                added += 1
            else:
                self._ekf_update(j, r, b, R)
        self.n_new += added
        return added

    def _associate(self, r: float, b: float, R: np.ndarray
                   ) -> Tuple[Optional[int], float]:
        """马氏距离最近邻。返回 (路标序号 或 None, 卡方距离)。

        门限用的协方差由 ``gate_sigma_*`` 决定，与卡尔曼更新用的 ``R`` 解耦。
        """
        c = self.cfg
        Rg = R if c.gate_sigma_r <= 0 else np.diag(
            [c.gate_sigma_r ** 2, c.gate_sigma_b ** 2])
        best, best_d2 = None, c.gate
        for j in range(self.n_lm):
            _, _, H, zp = self._predict_obs(j)
            S = H @ self.Sigma @ H.T + Rg
            innov = np.array([r - zp[0], float(wrap(b - zp[1]))])
            try:
                d2 = float(innov @ np.linalg.solve(S, innov))
            except np.linalg.LinAlgError:
                continue
            if d2 < best_d2:
                best, best_d2 = j, d2
        return best, best_d2

    def _ekf_update(self, j: int, r: float, b: float, R: np.ndarray) -> None:
        _, _, H, zp = self._predict_obs(j)
        innov = np.array([r - zp[0], float(wrap(b - zp[1]))])
        S = H @ self.Sigma @ H.T + R
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ innov
        self.mu[2] = float(wrap(self.mu[2]))
        I = np.eye(len(self.mu))
        self.Sigma = (I - K @ H) @ self.Sigma
        self.Sigma = 0.5 * (self.Sigma + self.Sigma.T)
        self.landmark_seen[j] += 1
        self.n_updates += 1

    def _init_landmark(self, r: float, b: float, R: np.ndarray = None) -> None:
        """把观测反投影成世界系路标，并把位姿不确定度传播进去。

        新路标由当前位姿推出，与已有状态不独立：补上与位姿（及经位姿与其它
        路标）的互协方差。只写对角块时，回环处路标被当作真值，位姿无法修正。
        """
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        ang = th + b
        mx = x + r * math.cos(ang)
        my = y + r * math.sin(ang)
        Gx = np.array([[1.0, 0.0, -r * math.sin(ang)],
                       [0.0, 1.0, r * math.cos(ang)]])
        d = self.cfg.new_landmark_inflate
        Gz = np.array([[math.cos(ang), -r * math.sin(ang) * d],
                       [math.sin(ang), r * math.cos(ang) * d]])
        if R is None:
            R = self.measurement_noise(r, b)
        n = len(self.mu)
        J = np.zeros((2, n))
        J[:, :3] = Gx
        Sigma_new = np.zeros((n + 2, n + 2))
        Sigma_new[:n, :n] = self.Sigma
        Sigma_new[:n, n:] = self.Sigma @ J.T
        Sigma_new[n:, :n] = J @ self.Sigma
        Sigma_new[n:, n:] = J @ self.Sigma @ J.T + Gz @ R @ Gz.T
        self.Sigma = 0.5 * (Sigma_new + Sigma_new.T)
        self.mu = np.concatenate([self.mu, [mx, my]])
        self.n_lm += 1
        self.landmark_seen = np.append(self.landmark_seen, 0)

    # ------------------------------------------------------------ 绝对观测
    def update_gps(self, xy: np.ndarray) -> None:
        H = np.zeros((2, len(self.mu)))
        H[0, 0] = H[1, 1] = 1.0
        R = np.eye(2) * self.cfg.gps_sigma ** 2
        innov = np.asarray(xy, float) - self.mu[:2]
        S = H @ self.Sigma @ H.T + R
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ innov
        self.Sigma = (np.eye(len(self.mu)) - K @ H) @ self.Sigma
        self.Sigma = 0.5 * (self.Sigma + self.Sigma.T)

    # ------------------------------------------------------------ 读出
    @property
    def pose(self) -> np.ndarray:
        return self.mu[:3].copy()

    @property
    def landmarks(self) -> np.ndarray:
        if self.n_lm == 0:
            return np.zeros((0, 2))
        return self.mu[3:].reshape(-1, 2).copy()

    def pose_sigma(self) -> np.ndarray:
        return np.sqrt(np.diag(self.Sigma)[:3]).copy()

    def trajectory(self) -> np.ndarray:
        return np.asarray(self.traj)
