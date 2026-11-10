#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""候选速度集上的离散安全约束求解。

约束 ``q(x, u) >= margin``：``q`` 是按环境动力学（一阶速度低通 + 航向跟随）
外推 ``horizon`` 秒后，路径上到障碍表面的最小间距。几何层用时变几何做精确
外推（教师/上界），神经层用一次前向直接给出修正指令（部署）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .geometry import rollout_clearance
from .ranges import direction_range, grid_min_range

__all__ = ["FilterConfig", "candidates", "select", "NoFilter", "BrakeFilter",
           "GeometricGuard", "NeuralGuard"]


@dataclass
class FilterConfig:
    margin: float = 0.80
    horizon: float = 0.6
    max_horiz: float = 4.0
    max_vert: float = 2.0
    tau: float = 0.12
    max_yaw_rate: float = 1.5
    progress: float = 0.2
    acc_max: float = 4.0
    guard_m: float = 0.0
    dir_guard: float = 0.0
    n_candidates: int = 0


def _clip(u: np.ndarray, cfg: FilterConfig) -> np.ndarray:
    return np.array([np.clip(u[0], -1.5, cfg.max_horiz),
                     np.clip(u[1], -3.0, 3.0),
                     np.clip(u[2], -cfg.max_vert, cfg.max_vert)])


def candidates(u_nom: np.ndarray, cfg: Optional[FilterConfig] = None
               ) -> np.ndarray:
    """速度缩放 × 方位偏转 × 高度偏置 的候选集（去重后约 110 个）。"""
    cfg = cfg or FilterConfig()
    u = np.asarray(u_nom, float).reshape(3)
    speed = max(float(np.hypot(u[0], u[1])), 1.0)
    out = [np.zeros(3)]
    for scale in (0.0, 0.3, 0.6, 1.0):
        for deg in (0, -30, 30, -60, 60, -90, 90, -120, 120):
            a = math.radians(deg)
            for dv in (0.0, -0.7, 0.7):
                out.append(np.array([scale * speed * math.cos(a),
                                     scale * speed * math.sin(a),
                                     u[2] + dv]))
    out.append(np.array([0.0, 0.0, 1.5]))
    out.append(np.array([0.0, 0.0, -1.5]))
    cands = np.asarray([_clip(c, cfg) for c in out], float)
    _, keep = np.unique(np.round(cands, 3), axis=0, return_index=True)
    return cands[np.sort(keep)]


def select(u_nom: np.ndarray, cands: np.ndarray, h: float, q: np.ndarray,
           cfg: FilterConfig, prev: Optional[np.ndarray] = None,
           goal_dir: Optional[np.ndarray] = None
           ) -> Tuple[np.ndarray, Dict[str, float]]:
    """可行候选里取代价最小者；无可行候选时刹车（而不是退回到最接近标称的）。"""
    dev = np.sum((cands - np.asarray(u_nom, float)[None, :]) ** 2, axis=1)
    feasible = q >= cfg.margin
    i_nom = int(np.argmin(dev))
    if np.any(feasible):
        cost = dev
        if goal_dir is not None:
            cost = cost - cfg.progress * (cands @ np.asarray(goal_dir, float))
        k = int(np.argmin(np.where(feasible, cost, np.inf)))
    else:
        # 无可行候选时取路径间距最大的动作：面对运动障碍，停下来只会被撞；
        # 往空隙里躲至少要留出一点余量。间距相同再比谁离标称更近。
        k = int(np.lexsort((dev, -q))[0])
    return cands[k], {"h": float(h), "threshold": float(cfg.margin),
                      "n_feasible": float(np.sum(feasible)),
                      "q_sel": float(q[k]), "q_nom": float(q[i_nom])}


class NoFilter:
    """不修正：直接用标称指令。"""

    name = "none"

    def reset(self, obs) -> None:
        pass

    def act(self, obs, u_nom):
        u = np.asarray(u_nom, float)
        return u, {"h": float(obs.get("h", 0.0)), "threshold": 0.0,
                   "n_feasible": 1.0, "q_sel": float("nan"),
                   "q_nom": float("nan")}


class BrakeFilter:
    """最近回波低于阈值时按比例减速：不转向的反应式基线。"""

    name = "brake"

    def __init__(self, stop_m: float = 1.2, floor_m: float = 0.4):
        self.stop_m = float(stop_m)
        self.floor_m = float(floor_m)

    def reset(self, obs) -> None:
        pass

    def act(self, obs, u_nom):
        r = grid_min_range(obs["ranges"])
        u = np.asarray(u_nom, float).copy()
        if r < self.stop_m:
            u = u * max(0.0, (r - self.floor_m)
                        / max(self.stop_m - self.floor_m, 1e-6))
        return u, {"h": float(obs.get("h", 0.0)), "threshold": self.stop_m,
                   "n_feasible": float(r >= self.stop_m), "q_sel": float(r),
                   "q_nom": float(r)}


class GeometricGuard:
    """几何教师：用场景真值（含运动障碍的轨迹）做 0.6 s 外推。"""

    name = "geometric"

    def __init__(self, env, cfg: Optional[FilterConfig] = None):
        self.env = env
        self.cfg = cfg or FilterConfig()

    def reset(self, obs) -> None:
        pass

    def act(self, obs, u_nom):
        cfg = self.cfg
        p = np.asarray(obs["pos"], float)
        yaw = float(obs["yaw"])
        t = float(obs.get("t", 0.0))
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        v_w = rot @ np.asarray(obs["vel"], float)
        cands = candidates(u_nom, cfg)
        q = rollout_clearance(self.env.field.batch_at, p,
                              np.repeat(v_w[None, :], len(cands), axis=0),
                              yaw, cands, horizon=cfg.horizon, tau=cfg.tau,
                              max_yaw_rate=cfg.max_yaw_rate,
                              acc_max=cfg.acc_max, t0=t)
        h = self.env.field.clearance_at(p, t)
        u, diag = select(np.asarray(u_nom, float), cands, h, q, cfg,
                         goal_dir=obs.get("goal_dir"))
        return u, diag


class NeuralGuard:
    """神经安全层：一次前向输出修正指令；``mode`` 选择单帧或时序特征。"""

    name = "neural_seq"

    def __init__(self, net, cfg: Optional[FilterConfig] = None,
                 mode: str = "seq"):
        if mode not in ("seq", "single"):
            raise ValueError("mode 只能是 seq 或 single")
        self.net = net
        self.cfg = cfg or FilterConfig()
        self.mode = mode
        self.name = "neural_seq" if mode == "seq" else "neural_single"

    def reset(self, obs) -> None:
        pass

    def features(self, obs) -> np.ndarray:
        key = "feat" if self.mode == "seq" else "feat_single"
        return np.asarray(obs[key], float)

    def act(self, obs, u_nom):
        cfg = self.cfg
        x = np.concatenate([self.features(obs),
                            np.asarray(u_nom, float)]).astype(np.float64)
        u = _clip(np.asarray(self.net.forward(x[None, :])[0], float), cfg)
        if cfg.guard_m > 0 and grid_min_range(obs["ranges"]) < cfg.guard_m:
            u = np.zeros(3)
        if cfg.dir_guard > 0:
            r = direction_range(obs["ranges"], u)[0] * 8.0
            if r < cfg.dir_guard:
                u = u * max(0.0, (r - 0.3)
                            / max(cfg.dir_guard - 0.3, 1e-6))
        return u, {"h": float(obs.get("h", 0.0)), "threshold": cfg.guard_m,
                   "n_feasible": 1.0, "q_sel": float("nan"),
                   "q_nom": float("nan")}
