#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 导航共享逻辑：观测构造 / 动作映射 / 激光雷达直方图 / 模型推理.

训练环境（纯 numpy 质点模型）与 CarlaAir 部署节点**共用本模块**，
保证两侧的观测与动作接口完全一致，训练出的策略可直接迁移部署。
"""
from __future__ import annotations

import math
from typing import Dict

import numpy as np

# ---- 常量 ----
N_SECTORS = 16          # 激光雷达水平扇形数量
MAX_RANGE = 12.0        # 激光雷达有效距离 (m)，超出视为"无遮挡"
MAX_HORIZ_SPEED = 3.0   # 水平速度上限 (m/s)，对应动作 [-1, 1]
MAX_VERT_SPEED = 1.5    # 垂直速度上限 (m/s)
MAX_YAW_RATE = 1.0      # 偏航角速度上限 (rad/s)
GOAL_SCALE = 6.0        # 目标相对位置归一化尺度 (m)
OBS_DIM = N_SECTORS + 3 + 3   # 16 扇形 + 3 目标 + 3 速度 = 22
ACT_DIM = 4

_Z_BAND = 1.5           # 直方图只统计无人机高度 ± 该范围的障碍点，避免地面误报


def lidar_points_to_histogram(points_body, n_sectors=N_SECTORS,
                              max_range=MAX_RANGE) -> np.ndarray:
    """机体系点云 (N, 3) -> 水平扇形直方图 (n_sectors,).

    points_body: 每行 (x_forward, y_left, z_up)，原点为无人机。
    每个扇形返回"该方向最近障碍距离"，归一化到 [0, 1]（1=无遮挡，0=贴脸）。
    垂直方向只统计 |z| < _Z_BAND 的点，避免地面/天花板误报成障碍。
    """
    pts = np.asarray(points_body, dtype=np.float64).reshape(-1, 3)
    hist = np.ones(n_sectors, dtype=np.float32)
    if pts.shape[0] == 0:
        return hist
    pts = pts[np.abs(pts[:, 2]) < _Z_BAND]
    if pts.shape[0] == 0:
        return hist
    r = np.hypot(pts[:, 0], pts[:, 1])
    ang = np.arctan2(pts[:, 1], pts[:, 0])            # -pi..pi，0 = 正前方
    sector = 2.0 * math.pi / n_sectors
    # 扇形 0 覆盖 [-sector/2, sector/2)，即正前方；逆时针（左）递增
    idx = np.floor((ang + sector / 2.0) / sector).astype(np.int64) % n_sectors
    for i in range(n_sectors):
        mask = idx == i
        if mask.any():
            hist[i] = float(np.clip(r[mask].min() / max_range, 0.0, 1.0))
    return hist


def world_to_body(dx, yaw: float) -> np.ndarray:
    """世界系相对向量 (dx, dy, dz) -> 机体系 (前, 左, 上)."""
    dx = np.asarray(dx, dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([dx[0] * c + dx[1] * s,
                     -dx[0] * s + dx[1] * c,
                     dx[2]], dtype=np.float64)


def build_observation(hist, goal_body, vel_body) -> np.ndarray:
    """拼成 22 维观测（与训练/部署完全一致）.

    hist      : (n_sectors,) 扇形直方图 [0,1]
    goal_body : (3,) 目标相对位置（机体系，米）
    vel_body  : (3,) 机体系速度（前/左/上，米/秒）
    """
    hist = np.asarray(hist, dtype=np.float32).reshape(-1)
    goal = np.asarray(goal_body, dtype=np.float32).reshape(-1)[:3] / GOAL_SCALE
    vel = np.asarray(vel_body, dtype=np.float32).reshape(-1)[:3]
    vel = np.array([vel[0] / MAX_HORIZ_SPEED,
                    vel[1] / MAX_HORIZ_SPEED,
                    vel[2] / MAX_VERT_SPEED], dtype=np.float32)
    return np.concatenate([hist, goal, vel]).astype(np.float32)


def action_to_velocity(action) -> tuple:
    """[-1,1]^4 动作 -> (vx, vy, vz, yaw_rate) 实际机体系速度指令."""
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:4], -1.0, 1.0)
    return (float(a[0] * MAX_HORIZ_SPEED),   # 前
            float(a[1] * MAX_HORIZ_SPEED),   # 左
            float(a[2] * MAX_VERT_SPEED),    # 上
            float(a[3] * MAX_YAW_RATE))      # 偏航角速度


class MlpPolicy(object):
    """训练好的 MLP 策略的纯 numpy 推理（部署侧无需 torch / sb3）.

    权重来自 SB3 的 MlpPolicy：隐藏层 net_arch=[64,64]，输出层 action_net。
    前向：obs -> tanh(W0 x + b0) -> tanh(W1 x + b1) -> W2 x + b2 -> clip[-1,1]。
    """

    def __init__(self, d: Dict[str, np.ndarray]):
        self.n = int(d["n_layers"])
        self.hidden = [(np.asarray(d["h%d_w" % i], dtype=np.float32),
                        np.asarray(d["h%d_b" % i], dtype=np.float32))
                       for i in range(self.n)]
        self.out_w = np.asarray(d["out_w"], dtype=np.float32)
        self.out_b = np.asarray(d["out_b"], dtype=np.float32)

    @classmethod
    def from_npz(cls, path: str) -> "MlpPolicy":
        return cls(np.load(path))

    def forward(self, obs) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        for w, b in self.hidden:
            x = np.tanh(w @ x + b)
        return np.clip(self.out_w @ x + self.out_b, -1.0, 1.0).astype(np.float32)


def extract_sb3_weights(model) -> Dict[str, np.ndarray]:
    """从 SB3 PPO 模型提取 MLP 权重 -> dict（供保存为 .npz，部署侧加载）.

    仅训练时调用（依赖 torch / sb3，部署侧不会走到这里）。
    """
    import torch  # noqa: WPS433 (训练期按需导入)

    p = model.policy
    hidden = []
    for m in p.mlp_extractor.policy_net:
        if isinstance(m, torch.nn.Linear):
            hidden.append((m.weight.detach().cpu().numpy(),
                           m.bias.detach().cpu().numpy()))
    out = p.action_net
    d = {"n_layers": len(hidden)}
    for i, (w, b) in enumerate(hidden):
        d["h%d_w" % i] = w
        d["h%d_b" % i] = b
    d["out_w"] = out.weight.detach().cpu().numpy()
    d["out_b"] = out.bias.detach().cpu().numpy()
    return d
