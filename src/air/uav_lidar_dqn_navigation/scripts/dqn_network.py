#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DQN 网络与动作空间定义

状态 32 维（24 扇区距离 + 4 目标相对量 + 3 机体速度 + 1 上一步动作），
动作为 6 个离散指令，采用 Dueling 结构把状态价值 V(s) 与优势 A(s,a) 解耦：

    Q(s, a) = V(s) + A(s, a) - mean_a A(s, a)

这样在多数动作都“平淡”的飞行场景里，网络只需要学好 V(s) 就能快速收敛。
"""

import numpy as np
import torch
import torch.nn as nn

NUM_ACTIONS = 6

# (前进速度 m/s, 偏航角速度 rad/s)，偏航正方向按 ROS 约定为左转。
# 高度由桥接节点的 altitude hold 保持，动作里不含升降分量。
ACTIONS = np.array([
    [0.0, 0.0],    # 0 悬停
    [3.0, 0.0],    # 1 直行
    [1.5, 0.9],    # 2 左前
    [1.5, -0.9],   # 3 右前
    [0.0, 1.0],    # 4 原地左转
    [0.0, -1.0],   # 5 原地右转
], dtype=np.float32)

ACTION_NAMES = ['hover', 'forward', 'forward_left', 'forward_right',
                'turn_left', 'turn_right']


class DuelingQNet(nn.Module):
    def __init__(self, state_dim=32, action_dim=NUM_ACTIONS, hidden=128):
        super(DuelingQNet, self).__init__()
        self.trunk = nn.Sequential(
            nn.Linear(state_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
        )
        self.value_head = nn.Linear(hidden, 1)
        self.advantage_head = nn.Linear(hidden, action_dim)

    def forward(self, x):
        h = self.trunk(x)
        v = self.value_head(h)
        a = self.advantage_head(h)
        return v + a - a.mean(dim=1, keepdim=True)


class ImageQNet(nn.Module):
    """任务 4：图像端到端 DQN 的卷积主干（84x84 灰度图）。"""

    def __init__(self, action_dim=NUM_ACTIONS, extra_dim=4):
        super(ImageQNet, self).__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=5, stride=2),   # 40x40
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2),  # 19x19
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, stride=1),  # 17x17
            nn.ReLU(),
        )
        self.fc = nn.Sequential(
            nn.Linear(32 * 17 * 17 + extra_dim, 128),
            nn.ReLU(),
        )
        self.value_head = nn.Linear(128, 1)
        self.advantage_head = nn.Linear(128, action_dim)

    def forward(self, img, extra):
        z = self.conv(img).view(img.size(0), -1)
        h = self.fc(torch.cat([z, extra], dim=1))
        v = self.value_head(h)
        a = self.advantage_head(h)
        return v + a - a.mean(dim=1, keepdim=True)
