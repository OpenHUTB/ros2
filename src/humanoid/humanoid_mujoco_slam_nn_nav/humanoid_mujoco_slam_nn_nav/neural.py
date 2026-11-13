"""神经网络策略封装：感知去噪 / 局部规划 / 运动控制 / 端到端。

四个网络都是纯 NumPy 推理（模型文件 ``models/*.npz``），输入输出与训练脚本
严格一致，ROS 节点直接复用本模块。
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
    """加载并运行四个网络。"""

    def __init__(self, model_dir: Optional[str] = None):
        self.model_dir = model_dir or os.path.join(_pkg_root(), "models")
        self.perception = nn.load_model(os.path.join(self.model_dir, "perception.npz"))
        self.planner = nn.load_model(os.path.join(self.model_dir, "planner.npz"))
        self.controller = nn.load_model(os.path.join(self.model_dir, "controller.npz"))
        self.e2e = nn.load_model(os.path.join(self.model_dir, "e2e.npz"))
        self.w = 4

    # ------------------------------------------------------------------
    def denoise(self, scan: np.ndarray) -> np.ndarray:
        """用 9 线滑窗网络修复激光测距（含噪声/丢点）。"""
        win = np.stack([np.roll(scan, k) for k in range(-self.w, self.w + 1)], axis=-1)
        return np.clip(nn.mlp_forward(self.perception, (win / 8.0).astype(np.float32)) * 8.0,
                       0.05, 8.0).ravel()

    def plan_direction(self, patch: np.ndarray, goal_xy, goal_dist: float) -> float:
        """返回局部前进方向相对机体系的夹角 φ（rad）。"""
        gx, gy = goal_xy
        x = np.concatenate([patch.reshape(-1), [gx, gy, goal_dist]]).astype(np.float32)[None]
        out = nn.mlp_forward(self.planner, x)[0]
        return float(math.atan2(out[1], out[0]))

    def control(self, goal_vec, heading_err: float, sectors: np.ndarray, v_prev: float):
        x = np.array([[float(np.linalg.norm(goal_vec)), float(heading_err),
                       sectors[0], sectors[1], sectors[2], v_prev]], dtype=np.float32)
        out = nn.mlp_forward(self.controller, x)[0]
        return float(np.clip(out[0], 0.0, 0.22)), float(np.clip(out[1], -0.5, 0.5))

    def end_to_end(self, depth: np.ndarray):
        x = (depth / 8.0).astype(np.float32)[None]
        out = nn.cnn_forward(self.e2e, x)[0]
        return float(np.clip(out[0], 0.0, 0.22)), float(np.clip(out[1], -0.5, 0.5))
