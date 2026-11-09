# -*- coding: utf-8 -*-
"""
任务 4: NVIDIA PilotNet 风格端到端驾驶卷积网络
================================================
输入: 前视单目相机图像 (160x96x3, 归一化到 [-1,1])
输出: (转向角 steer, 速度 speed) 回归

网络结构 (仿 PilotNet):
  Input (160x96x3)
  -> Conv2D 5x5x24  stride=2, ReLU
  -> Conv2D 5x5x36  stride=2, ReLU
  -> Conv2D 5x5x48  stride=2, ReLU
  -> Conv2D 3x3x64  stride=1, ReLU
  -> Conv2D 3x3x64  stride=1, ReLU
  -> Dropout(0.5) -> Flatten
  -> FC 100 -> ReLU
  -> FC 50  -> ReLU
  -> FC 10  -> ReLU
  -> FC 2   (steer, speed)
"""
import torch
import torch.nn as nn


class PilotNet(nn.Module):
    def __init__(self, out_dim=2, dropout=0.5):
        super(PilotNet, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1), nn.ReLU(),
        )
        self.dropout = nn.Dropout(dropout)
        # 160x96 经 5 层卷积后: 13x5 @64ch -> 4160 维
        self.fc = nn.Sequential(
            nn.Linear(64 * 13 * 5, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 10), nn.ReLU(),
            nn.Linear(10, out_dim),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.dropout(x)
        x = x.view(x.size(0), -1)
        return self.fc(x)


def build_model(device='cpu'):
    return PilotNet().to(device)
