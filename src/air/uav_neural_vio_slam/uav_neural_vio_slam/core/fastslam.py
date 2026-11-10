#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FastSLAM 1.0：粒子滤波表示**轨迹**，每个粒子带自己的一套路标 EKF。

与 EKF-SLAM 的根本差别是**位姿的表示**：EKF 用单个高斯，只能近似单峰分布，
回环时被线性化误差卡住；FastSLAM 用一组粒子，天然能表示多峰，
而且每条粒子独立维护地图，更新是 O(P·M) 的小矩阵运算而不是 O(M²) 的大矩阵。

两个工程细节：

* **数据关联跨粒子共享**（用加权平均地图做最近邻），不是逐粒子独立采样。
  标准 FastSLAM 1.0 是逐粒子采样的，但那样不同粒子会对同一根柱子给出不同的
  身份，路标数组没法共享。共享关联把实现从"每个粒子一份地图索引"简化成
  一份，代价是丢掉了关联上的多峰性 —— 这一点如实写进文档。
* **路标更新用采样出来的位姿当精确值**，这是 FastSLAM 1.0 的固有近似；
  位姿的不确定度由粒子云的散布来表达，不再进路标的协方差。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .ekf_slam import wrap

__all__ = ["FastSlamConfig", "FastSlam"]


@dataclass
class FastSlamConfig:
    n_particles: int = 160
    sigma_dp: float = 0.05
    sigma_dyaw: float = 0.015
    sigma_r: float = 0.10
    sigma_b: float = 0.030
    gate: float = 10.6
    resample_ratio: float = 0.5
    max_landmarks: int = 60
    gps_sigma: float = 0.5
    gps_weight: float = 1.0
    seed: int = 0


class FastSlam:
    def __init__(self, cfg: Optional[FastSlamConfig] = None, obs_model=None):
        self.cfg = cfg or FastSlamConfig()
        self.obs_model = obs_model
        self.reset()

    # ------------------------------------------------------------ 生命周期
    def reset(self, x: float = 0.0, y: float = 0.0, th: float = 0.0) -> None:
        c = self.cfg
        self.rng = np.random.default_rng(c.seed)
        P = c.n_particles
        self.poses = np.tile(np.array([x, y, th], float), (P, 1))
        # 权重存**对数**形式：一次观测的似然可能小到 1e-40，直接连乘会整片下溢成 0，
        # 之后 np.average(weights=w) 直接 ZeroDivisionError（实测踩过）。
        # 每一帧重采样前都要重新归一化，让 logw 保持在 0 附近。
        self.logw = np.zeros(P)
        self.w = np.full(P, 1.0 / P)
        self.lm_mu = np.zeros((P, 0, 2))
        self.lm_P = np.tile(np.eye(2) * 1.0, (P, 0, 1, 1))
        self.lm_seen = np.zeros(0, dtype=int)
        self.traj: List[np.ndarray] = []          # 只由 record() 追加，与帧数一一对应
        self.n_resample = 0
        self.n_updates = 0
        self._last_new = 0

    # ------------------------------------------------------------ 预测
    def _noise(self, sigma):
        c = self.cfg
        sr = c.sigma_dp if sigma is None else float(max(sigma[0], 1e-4))
        sa = c.sigma_dyaw if sigma is None else float(max(sigma[2], 1e-5))
        return sr, sa

    def predict(self, dp_b: np.ndarray, dyaw: float,
                sigma: Optional[np.ndarray] = None) -> None:
        sr, sa = self._noise(sigma)
        P = len(self.poses)
        # 过程噪声加在**机体系位移增量**上，与前端的不确定度语义一致
        dp = np.asarray(dp_b, float)[:2][None, :] \
            + self.rng.normal(0.0, sr, (P, 2))
        dy = float(dyaw) + self.rng.normal(0.0, sa, P)
        th = self.poses[:, 2]
        c, s = np.cos(th), np.sin(th)
        self.poses[:, 0] += c * dp[:, 0] - s * dp[:, 1]
        self.poses[:, 1] += s * dp[:, 0] + c * dp[:, 1]
        self.poses[:, 2] = wrap(th + dy)

    # ------------------------------------------------------------ 观测更新
    def _meas_cov(self, r: float, b: float) -> np.ndarray:
        if self.obs_model is not None:
            sr, sb = self.obs_model.predict(r, b)
        else:
            sr, sb = self.cfg.sigma_r, self.cfg.sigma_b
        return np.diag([sr ** 2, sb ** 2])

    def _predict_obs(self, j: int) -> Tuple[np.ndarray, np.ndarray]:
        """全部粒子对路标 j 的预测观测。返回 (P,2) 与雅可比 (P,2,2)。"""
        mx = self.lm_mu[:, j, 0] - self.poses[:, 0]
        my = self.lm_mu[:, j, 1] - self.poses[:, 1]
        q = mx * mx + my * my
        r = np.sqrt(np.maximum(q, 1e-9))
        b = wrap(np.arctan2(my, mx) - self.poses[:, 2])
        H = np.empty((len(self.poses), 2, 2))
        H[:, 0, 0] = mx / r
        H[:, 0, 1] = my / r
        H[:, 1, 0] = -my / np.maximum(q, 1e-9)
        H[:, 1, 1] = mx / np.maximum(q, 1e-9)
        return np.stack([r, b], axis=1), H

    def _mean_pose(self) -> np.ndarray:
        w = self.w / self.w.sum()
        th = float(np.arctan2(np.sum(w * np.sin(self.poses[:, 2])),
                              np.sum(w * np.cos(self.poses[:, 2]))))
        return np.array([float(np.sum(w * self.poses[:, 0])),
                         float(np.sum(w * self.poses[:, 1])), th])

    def _mean_map(self) -> np.ndarray:
        if self.lm_mu.shape[1] == 0:
            return np.zeros((0, 2))
        w = (self.w / self.w.sum())[:, None, None]
        return (self.lm_mu * w).sum(axis=0)

    def update_landmarks(self, obs_r, obs_b, n_obs: int) -> int:
        added = 0
        for k in range(int(n_obs)):
            r, b = float(obs_r[k]), float(obs_b[k])
            j = self._associate_shared(r, b)
            if j is None:
                self._add_landmark(r, b)
                added += 1
            else:
                self._update(j, r, b)
            self._normalize()          # 每处理完一条观测就归一化，防下溢
        self._last_new = added
        self._maybe_resample()
        return added

    def _associate_shared(self, r: float, b: float) -> Optional[int]:
        """用加权平均地图做马氏最近邻（跨粒子共享关联）。"""
        mp = self._mean_map()
        if len(mp) == 0:
            return None
        pose = self._mean_pose()
        dx = mp[:, 0] - pose[0]
        dy = mp[:, 1] - pose[1]
        rr = np.hypot(dx, dy)
        bb = wrap(np.arctan2(dy, dx) - pose[2])
        R = self._meas_cov(r, b)
        # 位姿不确定度用粒子云散布近似
        var = self._spread()
        d2 = (r - rr) ** 2 / (R[0, 0] + var) \
            + wrap(b - bb) ** 2 / (R[1, 1] + var)
        j = int(np.argmin(d2))
        return j if d2[j] < self.cfg.gate else None

    def _update(self, j: int, r: float, b: float) -> None:
        zp, H = self._predict_obs(j)
        R = self._meas_cov(r, b)
        P = self.lm_P[:, j]
        S = H @ P @ H.transpose(0, 2, 1) + R[None, :, :]
        Sinv = np.linalg.inv(S)
        innov = np.stack([r - zp[:, 0], wrap(b - zp[:, 1])], axis=1)
        # 权重 = 该观测的似然（对全部观测连乘）
        det = np.maximum(np.linalg.det(S), 1e-12)
        maha = np.einsum("pi,pij,pj->p", innov, Sinv, innov)
        self.logw += -0.5 * maha - 0.5 * np.log(det)
        K = P @ H.transpose(0, 2, 1) @ Sinv
        self.lm_mu[:, j] += np.einsum("pij,pj->pi", K, innov)
        I = np.eye(2)[None, :, :]
        self.lm_P[:, j] = (I - K @ H) @ P
        self.lm_P[:, j] = 0.5 * (self.lm_P[:, j]
                                 + self.lm_P[:, j].transpose(0, 2, 1))
        self.lm_seen[j] += 1
        self.n_updates += 1

    def _add_landmark(self, r: float, b: float) -> None:
        if self.lm_mu.shape[1] >= self.cfg.max_landmarks:
            return
        ang = self.poses[:, 2] + b
        mx = self.poses[:, 0] + r * np.cos(ang)
        my = self.poses[:, 1] + r * np.sin(ang)
        self.lm_mu = np.concatenate(
            [self.lm_mu, np.stack([mx, my], axis=1)[:, None, :]], axis=1)
        R = self._meas_cov(r, b)
        # 初值协方差：观测噪声 + 粒子云散布（位姿不确定度）
        mean = self._mean_pose()
        dev = self.poses[:, :2] - mean[None, :2]
        spread = float(np.average((dev ** 2).sum(axis=1), weights=self.w))
        P0 = np.eye(2) * (spread + R[0, 0] + (r * R[1, 1]))
        self.lm_P = np.concatenate(
            [self.lm_P, np.tile(P0, (len(self.poses), 1, 1))[:, None, :, :]],
            axis=1)
        self.lm_seen = np.append(self.lm_seen, 0)

    # ------------------------------------------------------------ 重采样
    def _normalize(self) -> None:
        """对数权重的稳定归一化（log-sum-exp 减去最大值）。"""
        P = len(self.poses)
        m = float(self.logw.max())
        if not np.isfinite(m):
            self.logw[:] = 0.0
            self.w = np.full(P, 1.0 / P)
            return
        lse = m + math.log(float(np.sum(np.exp(self.logw - m))))
        self.logw -= lse
        self.w = np.exp(self.logw)

    def _maybe_resample(self) -> None:
        neff = 1.0 / float(np.sum(self.w ** 2))
        if neff >= self.cfg.resample_ratio * len(self.poses):
            return
        self._resample()

    def _resample(self) -> None:
        """系统重采样：低方差，且不会把好粒子丢光。"""
        P = len(self.poses)
        pos = (self.rng.random() + np.arange(P)) / P
        cum = np.cumsum(self.w)
        idx = np.searchsorted(cum, pos)
        idx = np.clip(idx, 0, P - 1)
        self.poses = self.poses[idx].copy()
        self.lm_mu = self.lm_mu[idx].copy()
        self.lm_P = self.lm_P[idx].copy()
        self.logw[:] = 0.0
        self.w = np.full(P, 1.0 / P)
        self.n_resample += 1

    # ------------------------------------------------------------ 绝对观测
    def update_gps(self, xy: np.ndarray) -> None:
        sg = self.cfg.gps_sigma
        d2 = ((self.poses[:, 0] - xy[0]) ** 2 + (self.poses[:, 1] - xy[1]) ** 2)
        self.logw += -0.5 * d2 / (sg ** 2) * self.cfg.gps_weight
        self._normalize()
        self._maybe_resample()

    # ------------------------------------------------------------ 读出
    @property
    def pose(self) -> np.ndarray:
        return self._mean_pose()

    @property
    def landmarks(self) -> np.ndarray:
        mp = self._mean_map()
        if len(mp) == 0:
            return mp
        return mp[self.lm_seen > 0]

    def pose_sigma(self) -> np.ndarray:
        m = self._mean_pose()
        v = np.average((self.poses - m[None, :]) ** 2, weights=self.w, axis=0)
        return np.sqrt(v)

    def _spread(self) -> float:
        mean = self._mean_pose()
        dev = self.poses[:, :2] - mean[None, :2]
        return float(np.average((dev ** 2).sum(axis=1), weights=self.w))

    def record(self) -> None:
        self.traj.append(self._mean_pose())

    def trajectory(self) -> np.ndarray:
        return np.asarray(self.traj)

    def diagnostics(self) -> Dict[str, float]:
        return {"n_particles": float(len(self.poses)),
                "n_resample": float(self.n_resample),
                "n_landmarks": float(self.lm_mu.shape[1]),
                "neff": float(1.0 / np.sum(self.w ** 2))}
