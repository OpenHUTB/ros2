#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MNIST 手写数字 CNN 模型定义。

网络结构（输入 1x28x28）：
    Conv(1->16, 3x3, pad=1) -> ReLU -> MaxPool(2)      # 28 -> 14
    Conv(16->32, 3x3, pad=1) -> ReLU -> MaxPool(2)     # 14 -> 7
    Flatten -> Linear(1568->128) -> ReLU -> Dropout(0.25) -> Linear(128->10)

参数量约 2.1e5，CPU 单帧推理 < 1 ms。
"""
import torch
import torch.nn as nn


class DigitCNN(nn.Module):
    """用于 28x28 灰度手写数字识别的小型 CNN。"""

    def __init__(self, num_classes: int = 10) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(32 * 7 * 7, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.25),
            nn.Linear(128, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.classifier(self.features(x))


def load_model(model_path: str, device: str = "cpu") -> DigitCNN:
    """加载训练好的权重并切换到推理模式。"""
    model = DigitCNN()
    state = torch.load(model_path, map_location=device, weights_only=True)
    model.load_state_dict(state)
    model.to(device)
    model.eval()
    return model
