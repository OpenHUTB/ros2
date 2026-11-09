#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全过滤器：在候选速度集上近似求解离散时间 CBF 约束。

约束 ``q(x, u) >= (1 - alpha) * h(x)``，其中 ``q`` 是一步后的间距预测；
可行候选里取离标称指令最近的一个。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .env import FRAME_DT
from .geometry import rollout_clearance_many
from .network import SafetyNet
from .ranges import direction_range, grid_min_range, lidar_lower_bound

__all__ = ["FilterConfig", "candidates", "NoFilter", "BrakeFilter",
           "GeometricFilter", "NeuralFilter", "LearnedFilter"]


@dataclass
class FilterConfig:
    margin: float = 0.80
    horizon: float = 0.6
    max_horiz: float = 4.0
    max_vert: float = 2.0
    tau: float = 0.12
    progress: float = 0.2
    smooth: float = 0.0
    q_bias: float = 0.0
    lidar_guard: bool = True
    guard_m: float = 0.0         # 学习策略的最后一道近距刹车
    dir_guard: float = 0.0       # 指令方向上的最近回波低于该值时减速


def _clip(u: np.ndarray, cfg: FilterConfig) -> np.ndarray:
    return np.array([np.clip(u[0], -1.0, cfg.max_horiz),
                     np.clip(u[1], -3.0, 3.0),
                     np.clip(u[2], -cfg.max_vert, cfg.max_vert)])


def candidates(u_nom: np.ndarray, cfg: Optional[FilterConfig] = None
               ) -> np.ndarray:
    cfg = cfg or FilterConfig()
    u = np.asarray(u_nom, float).reshape(3)
    speed = max(float(np.hypot(u[0], u[1])), 1.0)
    out: List[np.ndarray] = [np.zeros(3)]
    for scale in (0.0, 0.25, 0.5, 0.75, 1.0):
        for deg in (0, -30, 30, -60, 60, -90, 90):
            a = math.radians(deg)
            for dv in (0.0, -0.8, 0.8):
                out.append(np.array([scale * speed * math.cos(a),
                                     scale * speed * math.sin(a),
                                     u[2] + dv]))
    out.append(np.array([0.0, 0.0, 1.5]))
    out.append(np.array([0.0, 0.0, -1.5]))
    cands = np.asarray([_clip(c, cfg) for c in out], float)
    _, keep = np.unique(np.round(cands, 3), axis=0, return_index=True)
    return cands[np.sort(keep)]


def _select(u_nom: np.ndarray, cands: np.ndarray, h: float, q: np.ndarray,
            cfg: FilterConfig, prev: Optional[np.ndarray] = None,
            goal_dir: Optional[np.ndarray] = None
            ) -> Tuple[np.ndarray, Dict[str, float]]:
    thr = cfg.margin
    feasible = q >= thr
    dev = np.sum((cands - u_nom[None, :]) ** 2, axis=1)
    i_nom = int(np.argmin(dev))
    if np.any(feasible):
        cost = dev
        if prev is not None and cfg.smooth > 0:
            cost = cost + cfg.smooth * np.sum(
                (cands - prev[None, :]) ** 2, axis=1)
        if goal_dir is not None:
            cost = cost - cfg.progress * (cands @ np.asarray(goal_dir, float))
        k = int(np.argmin(np.where(feasible, cost, np.inf)))
    else:
        k = int(np.argmin(np.sum(cands ** 2, axis=1)))
    return cands[k], {"h": float(h), "threshold": float(thr),
                      "n_feasible": float(np.sum(feasible)),
                      "q_sel": float(q[k]), "q_nom": float(q[i_nom])}


class NoFilter:
    name = "none"

    def reset(self, obs) -> None:
        pass

    def act(self, obs, u_nom):
        u = np.asarray(u_nom, float)
        return u, {"h": float(obs["h"]), "threshold": 0.0,
                   "n_feasible": 1.0, "q_sel": float("nan"),
                   "q_nom": float("nan")}


class BrakeFilter:
    """最近距离低于阈值时减速：不转向的经典反应式基线。"""
    name = "brake"

    def __init__(self, stop_m: float = 1.2):
        self.stop_m = float(stop_m)

    def reset(self, obs) -> None:
        pass

    def act(self, obs, u_nom):
        r = grid_min_range(obs["ranges"])
        u = np.asarray(u_nom, float)
        if r < self.stop_m:
            u = u * max(0.0, (r - 0.4) / max(self.stop_m - 0.4, 1e-6))
        return u, {"h": float(obs["h"]), "threshold": self.stop_m,
                   "n_feasible": float(r >= self.stop_m), "q_sel": float(r),
                   "q_nom": float(r)}


class GeometricFilter:
    """用真实几何计算 h 与一步后间距，作为候选集过滤器的上界。"""
    name = "geometric"

    def __init__(self, env, cfg: Optional[FilterConfig] = None):
        self.env = env
        self.cfg = cfg or FilterConfig()
        self._prev = np.zeros(3)

    def reset(self, obs) -> None:
        self._prev = np.zeros(3)

    def act(self, obs, u_nom):
        p = np.asarray(obs["pos"], float)
        yaw = float(obs["yaw"])
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        cands = candidates(u_nom, self.cfg)
        v_w = Rz @ np.asarray(obs["vel"], float)
        h = rollout_clearance_many(
            self.env.scene, p, v_w[None], yaw, np.asarray(obs["vel"])[None],
            self.cfg.horizon)[0]
        q = rollout_clearance_many(
            self.env.scene, p, np.repeat(v_w[None, :], len(cands), axis=0),
            yaw, cands, self.cfg.horizon)
        u, diag = _select(np.asarray(u_nom, float), cands, h, q, self.cfg,
                          self._prev, obs.get("goal_dir"))
        self._prev = u
        return u, diag


class NeuralFilter:
    name = "neural"

    def __init__(self, net: SafetyNet, cfg: Optional[FilterConfig] = None):
        self.net = net
        self.cfg = cfg or FilterConfig()
        self._prev = np.zeros(3)

    def reset(self, obs) -> None:
        self._prev = np.zeros(3)

    def act(self, obs, u_nom):
        feat = self.net.features(np.asarray(obs["feat"], float)[None, :])
        h = float(self.net.h_from_feat(feat)[0])
        cands = candidates(u_nom, self.cfg)
        qr = np.asarray([direction_range(obs["ranges"], u) for u in cands],
                        np.float32)
        q = self.net.q_from_feat(feat, np.concatenate([cands, qr], axis=1))
        q_lb = np.asarray([lidar_lower_bound(obs["ranges"], u, self.cfg.horizon)
                           for u in cands])
        q = q + q_lb - self.cfg.q_bias
        if self.cfg.lidar_guard:
            q = np.minimum(q, q_lb)
        u, diag = _select(np.asarray(u_nom, float), cands, h, q, self.cfg,
                          self._prev, obs.get("goal_dir"))
        self._prev = u
        return u, diag


class LearnedFilter:
    """一次前向直接输出修正指令，网络由几何安全层的行为克隆得到。"""
    name = "learned"

    def __init__(self, net, cfg: Optional[FilterConfig] = None):
        self.net = net
        self.cfg = cfg or FilterConfig()

    def reset(self, obs) -> None:
        pass

    def act(self, obs, u_nom):
        x = np.concatenate([np.asarray(obs["feat"], float),
                            np.asarray(u_nom, float)])[None, :]
        u = _clip(self.net.forward(x)[0], self.cfg)
        if self.cfg.guard_m > 0 and grid_min_range(obs["ranges"]) < self.cfg.guard_m:
            u = np.zeros(3)
        if self.cfg.dir_guard > 0:
            r = direction_range(obs["ranges"], u)[0] * 8.0
            if r < self.cfg.dir_guard:
                u = u * max(0.0, (r - 0.3)
                            / max(self.cfg.dir_guard - 0.3, 1e-6))
        return u, {"h": float(obs["h"]), "threshold": self.cfg.guard_m,
                   "n_feasible": 1.0, "q_sel": float("nan"),
                   "q_nom": float("nan")}
