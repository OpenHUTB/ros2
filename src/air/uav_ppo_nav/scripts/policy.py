#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 导航共享逻辑：观测构造 / 动作映射 / 激光雷达直方图 / 模型推理.

训练环境与 CarlaAir 部署节点**共用本模块**，保证两侧观测与动作接口完全一致。

坐标系说明（重要）
------------------
CarlaAir 里 SimpleFlight 的**偏航保持（yaw hold）不生效**，实测无人机机头会自行
漂移/摆动（±25°、约 9°/s）。因此本模块**完全使用世界系（ENU）**：

* 观测里的障碍直方图、目标方向、速度都是**世界系**的，与机头朝向无关；
* 动作是**世界系速度** `[vx, vy, vz]`（ENU 东-北-天）。

这样机头怎么转都不影响飞行轨迹，策略才能稳定收敛。
"""
from __future__ import annotations

import math
from typing import Dict

import numpy as np

# ---- 常量 ----
N_SECTORS = 16          # 激光雷达水平扇形数量（世界系固定方向，扇形 0 = 正东）
MAX_RANGE = 12.0        # 激光雷达有效距离 (m)，超出视为"无遮挡"
MAX_HORIZ_SPEED = 3.0   # 水平速度上限 (m/s)，对应动作 [-1, 1]
MAX_VERT_SPEED = 1.5    # 垂直速度上限 (m/s)
MAX_YAW_RATE = 1.0      # 偏航角速度上限 (rad/s)
GOAL_SCALE = 8.0        # 目标相对位置归一化尺度 (m)
OBS_DIM = N_SECTORS + 3 + 3   # 16 扇形 + 3 目标 + 3 速度 = 22
ACT_DIM = 4                   # [vx, vy, vz, yaw_rate]

_Z_BAND = 1.5           # 直方图只统计无人机高度 ± 该范围的障碍点，避免地面误报


def lidar_points_to_histogram(points_world, drone_xy=(0.0, 0.0), drone_z=0.0,
                              n_sectors=N_SECTORS, max_range=MAX_RANGE) -> np.ndarray:
    """世界系点云 (N, 3) ENU -> 水平扇形直方图 (n_sectors,).

    以无人机所在位置为原点，按**世界系固定方向**分扇（扇形 0 = 正东，逆时针递增），
    每个扇形返回该方向最近障碍距离，归一化到 [0, 1]（1 = 无遮挡）。
    垂直方向只统计无人机高度 ±_Z_BAND 内的点，避免地面被当成障碍。
    """
    pts = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    hist = np.ones(n_sectors, dtype=np.float32)
    if pts.shape[0] == 0:
        return hist
    pts = pts[np.abs(pts[:, 2] - float(drone_z)) < _Z_BAND]
    if pts.shape[0] == 0:
        return hist
    dx = pts[:, 0] - float(drone_xy[0])
    dy = pts[:, 1] - float(drone_xy[1])
    r = np.hypot(dx, dy)
    ang = np.arctan2(dy, dx)                       # -pi..pi，0 = 正东
    sector = 2.0 * math.pi / n_sectors
    idx = np.floor((ang + sector / 2.0) / sector).astype(np.int64) % n_sectors
    for i in range(n_sectors):
        mask = idx == i
        if mask.any():
            hist[i] = float(np.clip(r[mask].min() / max_range, 0.0, 1.0))
    return hist


def world_to_body(dx, yaw: float) -> np.ndarray:
    """世界系相对向量 -> 机体系（保留给需要机体系的场景；本模块默认不用）."""
    dx = np.asarray(dx, dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([dx[0] * c + dx[1] * s,
                     -dx[0] * s + dx[1] * c,
                     dx[2]], dtype=np.float64)


def build_observation(hist, goal_rel, vel_world) -> np.ndarray:
    """拼成 22 维观测（全部世界系）.

    hist       : (n_sectors,) 扇形直方图 [0,1]
    goal_rel   : (3,) 目标相对无人机的世界系位置（东/北/天，米）
    vel_world  : (3,) 世界系速度（东/北/天，米/秒）
    """
    hist = np.asarray(hist, dtype=np.float32).reshape(-1)
    goal = np.asarray(goal_rel, dtype=np.float32).reshape(-1)[:3] / GOAL_SCALE
    vel = np.asarray(vel_world, dtype=np.float32).reshape(-1)[:3]
    vel = np.array([vel[0] / MAX_HORIZ_SPEED,
                    vel[1] / MAX_HORIZ_SPEED,
                    vel[2] / MAX_VERT_SPEED], dtype=np.float32)
    return np.concatenate([hist, goal, vel]).astype(np.float32)


def action_to_velocity(action) -> tuple:
    """[-1,1]^4 动作 -> (vx, vy, vz, yaw_rate)：**世界系** ENU 速度 + 偏航角速度."""
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:4], -1.0, 1.0)
    return (float(a[0] * MAX_HORIZ_SPEED),   # 东
            float(a[1] * MAX_HORIZ_SPEED),   # 北
            float(a[2] * MAX_VERT_SPEED),    # 天
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
