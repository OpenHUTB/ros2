#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 网络结构模块：model（任务四：端到端视觉行为克隆，第二阶段·模型）
# 作用：轻量级卷积神经网络 DronePolicyNet，输入 3x224x224 前视图像，
#       回归输出 4 维动作向量 [vx, vy, vz, yaw_rate]（NED 机体坐标系，
#       与 /drone/cmd_vel 同量纲：线速度 m/s、偏航角速度 rad/s）。
# 为什么不用 MobileNetV2 微调：本仓库不依赖 torchvision 与外部预训练权重，
# 自研 4 层 Conv2D + FC 总计算量约 90M MACs，现代 CPU 上单帧前向约
# 10~30 ms，完全满足 10 Hz 实时推理，且部署端（虚拟机 CPU）零额外依赖。
# 输出层不加激活：速度与角速度都是有符号实数，直接线性回归。

import torch
import torch.nn as nn


class DronePolicyNet(nn.Module):
    """轻量 CNN 策略网络：前视图像 -> 4 维动作回归

    结构：Conv(3,32,k7,s2) -> BN+ReLU -> MaxPool(2)
          -> Conv(32,64,k3,s2) -> BN+ReLU
          -> Conv(64,96,k3,s2) -> BN+ReLU
          -> Conv(96,128,k3,s2) -> BN+ReLU
          -> AdaptiveAvgPool(1) -> FC(128,64) -> ReLU -> Dropout -> FC(64,4)
    逐层 stride=2 下采样把 224x224 压到 7x7，全局平均池化解耦输入分辨率，
    全连接头只做线性回归。
    """

    def __init__(self, in_channels=3, num_outputs=4, dropout=0.2):
        super().__init__()
        self.features = nn.Sequential(
            # 224 -> 112：k7 s2 大感受野先抓整体纹理，紧跟池化再减半到 56
            nn.Conv2d(in_channels, 32, kernel_size=7, stride=2,
                      padding=3, bias=False),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            # 56 -> 28
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            # 28 -> 14
            nn.Conv2d(64, 96, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(96),
            nn.ReLU(inplace=True),
            # 14 -> 7
            nn.Conv2d(96, 128, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )
        self.pool = nn.AdaptiveAvgPool2d(1)     # 7x7 -> 1x1，全局特征 128 维
        self.head = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, num_outputs),
        )

    def forward(self, x):
        feat = self.features(x)
        feat = self.pool(feat).flatten(1)
        return self.head(feat)


def build_model(num_outputs=4, **kwargs):
    """工厂函数：新建策略网络（train.py 与 drone_autonomous_node.py 共用）"""
    return DronePolicyNet(num_outputs=num_outputs, **kwargs)
