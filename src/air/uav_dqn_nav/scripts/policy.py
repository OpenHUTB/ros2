#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_dqn_nav 共享逻辑：观测构造 / 连续动作映射 / DQN 离散动作表 / Q 网络 / 推理.

本模块与 PPO 模块（uav_ppo_nav）**共用同一套观测与坐标系约定**：
同一环境、同一 22 维观测、同一世界系（ENU）动作，仅把「策略学习算法」从 PPO
换成 DQN，从而在作业加分项「同样模型不同 RL 算法对比」下保证公平。

坐标系说明（与 PPO 完全一致）
----------------------------
CarlaAir 里 SimpleFlight 的偏航保持（yaw hold）不生效，无人机机头会自行漂移。
因此观测与动作全部使用**世界系（ENU）**：机头怎么转都不影响飞行轨迹。
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

# ---- 常量（与 uav_ppo_nav/policy.py 保持一致，保证对比公平）----
N_SECTORS = 16          # 激光雷达水平扇形数量（世界系固定方向，扇形 0 = 正东）
MAX_RANGE = 12.0        # 激光雷达有效距离 (m)，超出视为"无遮挡"
MAX_HORIZ_SPEED = 3.0   # 水平速度上限 (m/s)，对应连续动作 [-1, 1]
MAX_VERT_SPEED = 1.5    # 垂直速度上限 (m/s)
MAX_YAW_RATE = 1.0      # 偏航角速度上限 (rad/s)
GOAL_SCALE = 8.0        # 目标相对位置归一化尺度 (m)
OBS_DIM = N_SECTORS + 3 + 3   # 16 扇形 + 3 目标 + 3 速度 = 22

# 连续动作维度（与 PPO 的 4 维世界系速度接口一致；DQN 通过离散表映射到同一接口）
ACT_DIM = 4

# ---- DQN 离散动作空间（35 个）----
# 演进：初版 11 个动作（8 方向巡航 + 升降 + 悬停）实测成功率低——无人机能避障但
# 无法精确停靠目标（目标半径 0.4 m）。v2 加"慢速逼近"档（19 动作）仍不收敛，
# 定位到根因是**方向离散精度**：8 方向（45° 间隔）在 2-8 m 目标距离上的横向误差
# 0.85-3 m，远超到达半径，无人机只能在目标附近绕圈。v3 改为 **16 方向**
# （22.5° 间隔，最大角误差 11.25°，2 m 距离横向误差 < 0.4 m），巡航 + 慢速双档：
# 0-15 ：水平 16 方向巡航（速度 = MAX_HORIZ_SPEED*0.7，保持高度）
# 16-31：水平 16 方向慢速逼近（速度 = MAX_HORIZ_SPEED*0.35）
# 32   ：上升（vz = +MAX_VERT_SPEED）
# 33   ：下降（vz = -MAX_VERT_SPEED）
# 34   ：悬停（全零）
N_ACTIONS = 35
_CRUISE = 0.7 * MAX_HORIZ_SPEED       # 巡航水平速度 (m/s)
_SLOW = 0.35 * MAX_HORIZ_SPEED        # 慢速逼近水平速度 (m/s)
_DIRS = [(math.cos(i * math.pi / 8), math.sin(i * math.pi / 8))
         for i in range(16)]          # 16 方向，0 = 正东，逆时针
_DIR_NAMES = ["东", "东北偏东", "东北", "北东北", "北", "北西北", "西北", "西北偏西",
              "西", "西南偏西", "西南", "南西南", "南", "南东南", "东南", "东东南"]
ACTION_NAMES = _DIR_NAMES + [n + "(慢)" for n in _DIR_NAMES] + \
    ["上升", "下降", "悬停"]
# 世界系速度表 [vx, vy, vz, yaw_rate]（ENU，米/秒 + 弧度/秒）
ACTION_VELOCITIES: List[Tuple[float, float, float, float]] = [
    (_CRUISE * dx, _CRUISE * dy, 0.0, 0.0) for (dx, dy) in _DIRS
] + [
    (_SLOW * dx, _SLOW * dy, 0.0, 0.0) for (dx, dy) in _DIRS
] + [
    (0.0, 0.0, MAX_VERT_SPEED, 0.0),                            # 32 上升
    (0.0, 0.0, -MAX_VERT_SPEED, 0.0),                           # 33 下降
    (0.0, 0.0, 0.0, 0.0),                                       # 34 悬停
]

_Z_BAND = 1.5           # 直方图只统计无人机高度 ± 该范围的障碍点，避免地面误报


# ------------------------------------------------------------------ 观测与连续动作（与 PPO 共用）
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


def action_to_velocity(action) -> Tuple[float, float, float, float]:
    """[-1,1]^4 连续动作 -> (vx, vy, vz, yaw_rate)：世界系 ENU 速度（PPO 兼容）."""
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:4], -1.0, 1.0)
    return (float(a[0] * MAX_HORIZ_SPEED),   # 东
            float(a[1] * MAX_HORIZ_SPEED),   # 北
            float(a[2] * MAX_VERT_SPEED),    # 天
            float(a[3] * MAX_YAW_RATE))      # 偏航角速度


def act_index_to_velocity(idx) -> Tuple[float, float, float, float]:
    """DQN 离散动作索引 -> 世界系速度（喂给与 PPO 相同的 NavEnv.step）."""
    idx = int(np.clip(int(idx), 0, N_ACTIONS - 1))
    return ACTION_VELOCITIES[idx]


# ------------------------------------------------------------------ DQN Q 网络（训练侧，依赖 torch）
def build_q_network(obs_dim: int = OBS_DIM, n_actions: int = N_ACTIONS,
                    hidden: Tuple[int, int] = (64, 64)):
    """工厂函数：新建 Q 网络（train.py 与 dqn_agent.py 共用）."""
    import torch.nn as nn  # noqa: WPS433 (训练期按需导入)

    class QNetwork(nn.Module):
        """双隐藏层 MLP Q 网络：obs(22) -> 64 -> 64 -> n_actions.

        每个动作的 Q 值 Q(s, a)，动作选择取 argmax。
        """

        def __init__(self, obs_dim=obs_dim, n_actions=n_actions,
                     hidden=hidden):
            super().__init__()
            layers = []
            prev = obs_dim
            for h in hidden:
                layers.append(nn.Linear(prev, h))
                layers.append(nn.ReLU(inplace=True))
                prev = h
            layers.append(nn.Linear(prev, n_actions))
            self.net = nn.Sequential(*layers)

        def forward(self, x):
            return self.net(x)

    return QNetwork()


# ------------------------------------------------------------------ DQN 策略推理（部署侧，纯 numpy）
class DqnPolicy(object):
    """训练好的 DQN 策略的纯 numpy 推理（部署侧无需 torch）.

    权重来自 Q 网络（MLP，ReLU 激活）：
        q = W2 @ relu(W1 @ relu(W0 @ obs + b0) + b1) + b2
    动作 = argmax(q)。推理只做一次前向，计算量远小于训练。
    """

    def __init__(self, d: Dict[str, np.ndarray]):
        self.n = int(d["n_layers"])
        self.hidden = [(np.asarray(d["h%d_w" % i], dtype=np.float32),
                        np.asarray(d["h%d_b" % i], dtype=np.float32))
                       for i in range(self.n)]
        self.out_w = np.asarray(d["out_w"], dtype=np.float32)
        self.out_b = np.asarray(d["out_b"], dtype=np.float32)

    @classmethod
    def from_npz(cls, path: str) -> "DqnPolicy":
        return cls(np.load(path))

    def q_values(self, obs) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        for w, b in self.hidden:
            x = np.maximum(w @ x + b, 0.0)                 # ReLU
        return (self.out_w @ x + self.out_b).astype(np.float32)

    def act(self, obs) -> int:
        """返回贪心动作索引（部署侧推理）."""
        return int(np.argmax(self.q_values(obs)))

    def forward(self, obs) -> np.ndarray:
        """返回世界系 4 维速度（与 PPO 的 MlpPolicy.forward 接口一致，便于对比）. """
        return np.asarray(act_index_to_velocity(self.act(obs)), dtype=np.float32)


def extract_torch_weights(model) -> Dict[str, np.ndarray]:
    """从 torch Q 网络提取权重 -> dict（供保存 .npz，部署侧 numpy 加载）.

    仅训练时调用（依赖 torch；部署侧不会走到这里）。
    """
    import torch  # noqa: WPS433

    layers = []
    for m in model.net:
        if isinstance(m, torch.nn.Linear):
            layers.append((m.weight.detach().cpu().numpy(),
                           m.bias.detach().cpu().numpy()))
    # 最后一层是输出层（Q 值头），其余是隐藏层
    d = {"n_layers": len(layers) - 1}
    for i, (w, b) in enumerate(layers[:-1]):
        d["h%d_w" % i] = w
        d["h%d_b" % i] = b
    d["out_w"], d["out_b"] = layers[-1]
    return d
