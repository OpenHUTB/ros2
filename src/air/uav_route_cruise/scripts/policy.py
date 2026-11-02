#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""策略侧公共逻辑：观测/动作接口 + 纯 numpy MLP 推理 + 从 sb3 导出权重.

部署侧（ROS 节点）**不需要 torch / sb3**，只加载 .npz 权重做一次前向即可 ——
与 uav_ppo_nav 的做法保持一致，方便在同一台机器上共用一套依赖。

观测 22 维 / 动作 4 维的语义见 route_env.py 的模块注释。
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import math
from typing import Dict

import numpy as np

OBS_DIM = 30
ACT_DIM = 4

MAX_HORIZ_SPEED = 3.0
MAX_VERT_SPEED = 1.5
MAX_YAW_RATE = 1.0


def action_to_velocity(action):
    """[-1,1]^4 -> (vx, vy, vz, yaw_rate)，**世界系** ENU 速度."""
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:ACT_DIM], -1.0, 1.0)
    return (float(a[0] * MAX_HORIZ_SPEED),
            float(a[1] * MAX_HORIZ_SPEED),
            float(a[2] * MAX_VERT_SPEED),
            float(a[3] * MAX_YAW_RATE))


class MlpPolicy(object):
    """训练好的 PPO 策略的纯 numpy 推理（部署侧无需 torch / sb3）.

    权重来自 SB3 的 MlpPolicy：隐藏层 net_arch=[64,64]，输出层 action_net。
    前向：obs -> tanh(W0 x + b0) -> tanh(W1 x + b1) -> W2 x + b2 -> clip[-1,1]
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


def build_observation(hist, cross, cross_angle, goal_rel_xy, speed_h, z_err) -> np.ndarray:
    """把各分量拼成 30 维观测（供 ROS 节点在实机上复用同一顺序）.

    hist           : (24,) 扇形距离 [0,1]
    cross          : 横向偏差（米，左正）
    cross_angle    : 航向偏差（弧度）
    goal_rel_xy    : (2,) 前方 4 m 处航点的相对位置（米）
    speed_h        : 水平速度（米/秒）
    z_err          : 高度偏差（米）
    """
    from route_env import CROSS_SCALE, LOOKAHEAD_SCALE, Z_SCALE  # noqa: WPS433
    hist = np.asarray(hist, dtype=np.float32).reshape(-1)
    g = np.asarray(goal_rel_xy, dtype=np.float32).reshape(-1)[:2]
    extra = np.array([
        float(np.clip(cross / CROSS_SCALE, -3.0, 3.0)),
        float(cross_angle / math.pi),
        float(np.clip(g[0] / LOOKAHEAD_SCALE, -3.0, 3.0)),
        float(np.clip(g[1] / LOOKAHEAD_SCALE, -3.0, 3.0)),
        float(speed_h / MAX_HORIZ_SPEED),
        float(np.clip(z_err / Z_SCALE, -3.0, 3.0)),
    ], dtype=np.float32)
    return np.concatenate([hist, extra]).astype(np.float32)


def extract_sb3_weights(model) -> Dict[str, np.ndarray]:
    """从 SB3 PPO 模型提取 MLP 权重 -> dict（保存为 .npz 供部署侧加载）.

    仅训练时调用（依赖 torch / sb3）。
    """
    import torch  # noqa: WPS433

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
