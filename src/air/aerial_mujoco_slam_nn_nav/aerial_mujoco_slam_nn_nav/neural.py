"""神经网络策略封装：感知去噪 / 局部规划 / 运动控制 / 端到端。

四个网络都是纯 NumPy 推理（模型文件 ``models/*.npz``），输入输出与训练脚本一致。
无人机的控制量是机体系三维速度 $(v_x, v_y)$ 与偏航角速度 $\omega$。
"""

from __future__ import annotations

import math
import os
from typing import Optional

import numpy as np

from . import nn


def _pkg_root() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class NeuralPolicy:
    def __init__(self, model_dir: Optional[str] = None):
        self.model_dir = model_dir or os.path.join(_pkg_root(), "models")
        self.perception = nn.load_model(os.path.join(self.model_dir, "perception.npz"))
        self.planner = nn.load_model(os.path.join(self.model_dir, "planner.npz"))
        self.controller = nn.load_model(os.path.join(self.model_dir, "controller.npz"))
        self.e2e = nn.load_model(os.path.join(self.model_dir, "e2e.npz"))
        self.w = 4

    # ------------------------------------------------------------------
    def denoise(self, scan: np.ndarray) -> np.ndarray:
        win = np.stack([np.roll(scan, k) for k in range(-self.w, self.w + 1)], axis=-1)
        return np.clip(nn.mlp_forward(self.perception, (win / 8.0).astype(np.float32)) * 8.0,
                       0.05, 8.0).ravel()

    def plan_direction(self, patch: np.ndarray, goal_xy, goal_dist: float) -> float:
        gx, gy = goal_xy
        x = np.concatenate([patch.reshape(-1), [gx, gy, goal_dist]]).astype(np.float32)[None]
        out = nn.mlp_forward(self.planner, x)[0]
        return float(math.atan2(out[1], out[0]))

    def control(self, goal_vec, heading_err: float, sectors: np.ndarray, v_prev: float):
        x = np.array([[float(np.linalg.norm(goal_vec)), float(heading_err),
                       sectors[0], sectors[1], sectors[2], v_prev]], dtype=np.float32)
        out = nn.mlp_forward(self.controller, x)[0]
        return (float(np.clip(out[0], -0.8, 0.8)),
                float(np.clip(out[1], -0.8, 0.8)),
                float(np.clip(out[2], -1.0, 1.0)))

    def end_to_end(self, depth: np.ndarray):
        x = (depth / 8.0).astype(np.float32)[None]
        out = nn.cnn_forward(self.e2e, x)[0]
        vx = float(np.clip(out[0], 0.0, 0.35))
        vy = float(np.clip(out[1], -0.35, 0.35))
        return vx, vy, float(np.clip(1.2 * math.atan2(vy, max(vx, 0.08)), -0.8, 0.8))
