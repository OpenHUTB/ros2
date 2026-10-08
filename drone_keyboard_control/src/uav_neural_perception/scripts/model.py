#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
model.py —— CNN 感知模型

输入：机载前视 RGB 摄像头图像（已 resize 到固定尺寸）
输出：3 维连续控制量 [vx, vy, yaw_rate]
"""
import torch
import torch.nn as nn


class DronePerceptionNet(nn.Module):
    """简单卷积回归网络：图像 -> 3 维控制量。"""

    def __init__(self, input_channels: int = 3, out_dim: int = 3):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(input_channels, 16, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4)),
        )
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 4 * 4, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, out_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.features(x)
        return self.regressor(x)


def build_model() -> DronePerceptionNet:
    return DronePerceptionNet(input_channels=3, out_dim=3)


if __name__ == "__main__":
    net = build_model()
    dummy = torch.randn(1, 3, 120, 160)
    out = net(dummy)
    print(f"模型参数量: {sum(p.numel() for p in net.parameters()):,}")
    print(f"输入 {tuple(dummy.shape)} -> 输出 {tuple(out.shape)}")
    print("model.py 自检通过")
