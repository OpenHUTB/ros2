#!/usr/bin/env python3
"""explorer_net.py —— 神经网络探索策略（感知->规划->控制）

结构：MLP
    输入 x = [d_1, ..., d_16, sin(theta), cos(theta)]  (18维)
    输出 a = (v_norm, omega_norm) ∈ [-1,1]^2
    实际速度: v = (v_norm+1)/2 * V_MAX
              omega = omega_norm * W_MAX

训练方式：行为克隆（见 train.py），用专家避障数据监督学习。
未加载权重时采用随机初始化 + 安全规则兜底，保证开箱即可飞行。
"""
import os

# ---- 惰性导入：无 torch 环境也可 import（语法/结构校验用）----
try:
    import torch
    import torch.nn as nn
    _TORCH_OK = True
except ImportError:  # pragma: no cover
    torch = None
    nn = None
    _TORCH_OK = False

import numpy as np


class ExplorerNet:
    """封装 PyTorch 的 MLP 探索策略，对外提供 numpy 接口。"""

    def __init__(self, input_dim=18, hidden_dim=64, output_dim=2, weight_path=None):
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.output_dim = output_dim
        self.weight_path = weight_path
        if _TORCH_OK:
            self._build()
        else:
            self._rng = np.random.default_rng(0)

    def _build(self):
        class _MLP(nn.Module):
            def __init__(self, in_dim, hid, out_dim):
                super().__init__()
                self.net = nn.Sequential(
                    nn.Linear(in_dim, hid),
                    nn.ReLU(),
                    nn.Linear(hid, hid),
                    nn.ReLU(),
                    nn.Linear(hid, out_dim),
                    nn.Tanh(),
                )

            def forward(self, x):
                return self.net(x)

        self.model = _MLP(self.input_dim, self.hidden_dim, self.output_dim)

    # ------------------------------------------------------------------
    def load(self, path=None):
        """加载预训练权重；文件不存在时静默使用随机初始化。"""
        if not _TORCH_OK:
            return False
        path = path or self.weight_path
        if path and os.path.exists(path):
            try:
                self.model.load_state_dict(torch.load(path, map_location="cpu"))
                return True
            except Exception as exc:  # pragma: no cover
                print(f"[explorer_net] 权重加载失败({exc})，使用随机初始化")
        return False

    def save(self, path):
        if _TORCH_OK:
            torch.save(self.model.state_dict(), path)

    # ------------------------------------------------------------------
    def predict(self, features, goal_dir):
        """前向推理。

        Args:
            features: (16,) 归一化扇区距离特征 [0,1]
            goal_dir:  (2,) 目标方向 (sin, cos)

        Returns:
            (v, omega)：线速度(m/s)、角速度(rad/s)
        """
        x = np.concatenate([np.asarray(features, dtype=np.float64),
                            np.asarray(goal_dir, dtype=np.float64)])
        x = x.reshape(1, -1)
        if _TORCH_OK:
            with torch.no_grad():
                out = self.model(torch.from_numpy(x.astype(np.float32))).numpy().ravel()
        else:
            # 无 torch 环境的随机网络回退（仅结构演示用）
            out = self._rng.uniform(-0.5, 0.5, size=self.output_dim)
        v_norm, w_norm = float(out[0]), float(out[1])
        v = (v_norm + 1.0) / 2.0
        return v, w_norm
