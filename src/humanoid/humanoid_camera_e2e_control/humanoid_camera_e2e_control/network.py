# -*- coding: utf-8 -*-
"""
network.py —— 人形机器人图像端到端策略网络（PilotNet 风格 CNN，双输出头）

输入：相机图像 N×3×H×W（像素归一化到 [-1, 1]）
共享主干：5 层卷积提取视觉特征；两个任务头：
  1. 回归头：输出归一化指令 (v_n, w_n) ∈ [-1, 1]，推理时反归一化为
     物理线速度 v = v_n · v_max、偏航角速度 w = w_n · w_max；
  2. 方向分类辅助头：输出三分类 logits（0=直行/停车，1=左转，2=右转），
     训练时以交叉熵辅助监督，帮助卷积主干先学到左右障碍判别特征；
     部署推理只使用回归头输出的连续速度指令。

结构（参照 Nvidia PilotNet 的卷积—全连接布局，按 96×128 小图压缩通道数）：
    Conv3→24  5×5 stride2 → ReLU
    Conv24→36 5×5 stride2 → ReLU
    Conv36→48 5×5 stride2 → ReLU
    Conv48→64 3×3 stride1 → ReLU
    Conv64→64 3×3 stride1 → ReLU
    Flatten → 共享特征 → 回归头 / 分类头

全连接层使用 ``nn.LazyLinear``，首次前向自动推导展平维度，避免手工算特征图尺寸。
"""

import os

import torch
import torch.nn as nn


class E2ECNN(nn.Module):
    """单目图像 → (线速度, 偏航角速度) 回归 + 方向三分类辅助监督的端到端网络。"""

    #: 方向分类类别：0=直行/停车，1=左转(w>0)，2=右转(w<0)
    NUM_CLASSES = 3

    def __init__(self, v_max: float = 0.30, w_max: float = 0.60):
        super().__init__()
        self.v_max = float(v_max)
        self.w_max = float(w_max)

        # 共享卷积主干（含展平）
        self.features = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Flatten(),
        )
        # 回归头：归一化速度指令
        self.regressor = nn.Sequential(
            nn.LazyLinear(100), nn.ReLU(), nn.Dropout(0.3),
            nn.LazyLinear(50), nn.ReLU(),
            nn.LazyLinear(2),
        )
        # 方向分类辅助头
        self.classifier = nn.Sequential(
            nn.LazyLinear(50), nn.ReLU(),
            nn.LazyLinear(self.NUM_CLASSES),
        )

    def forward_features(self, x: torch.Tensor) -> torch.Tensor:
        """共享卷积主干特征（展平向量）。"""
        return self.features(x)

    def forward_norm(self, x: torch.Tensor) -> torch.Tensor:
        """回归头输出归一化指令 (v_n, w_n) ∈ [-1, 1]（N×2），训练空间。"""
        return torch.tanh(self.regressor(self.forward_features(x)))

    def forward_class(self, x: torch.Tensor) -> torch.Tensor:
        """分类头输出方向三分类 logits（N×3）。"""
        return self.classifier(self.forward_features(x))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """部署推理：输出物理指令 (v m/s, w rad/s)（N×2），仅用回归头。"""
        u = self.forward_norm(x)
        v = u[:, 0:1] * self.v_max
        w = u[:, 1:2] * self.w_max
        return torch.cat([v, w], dim=1)

    # ------------------------------------------------------------------
    # 权重持久化（含限幅元信息，保证部署端反归一化口径一致）
    # ------------------------------------------------------------------
    def save(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        torch.save({
            "state_dict": self.state_dict(),
            "v_max": self.v_max,
            "w_max": self.w_max,
        }, path)

    @classmethod
    def load(cls, path: str) -> "E2ECNN":
        ckpt = torch.load(path, map_location="cpu")
        net = cls(v_max=ckpt.get("v_max", 0.30), w_max=ckpt.get("w_max", 0.60))
        # 先做一次假前向，初始化 LazyLinear 权重形状，再加载 checkpoint
        net(torch.zeros(1, 3, 96, 128))
        net.load_state_dict(ckpt["state_dict"])
        net.eval()
        return net
