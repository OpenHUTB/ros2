"""Small trainable models for survey perception and control."""

from __future__ import annotations

import torch
from torch import nn


class TinyUNet(nn.Module):
    """A compact encoder-decoder for RGB to binary PV masks."""

    def __init__(self) -> None:
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 8, 3, padding=1), nn.ReLU(),
            nn.Conv2d(8, 16, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(16, 16, 3, padding=1), nn.ReLU(),
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(16, 8, 4, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(8, 1, 1),
        )

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        return self.decoder(self.encoder(images))


class TinyMLP(nn.Module):
    """Small deterministic controller regressor."""

    def __init__(self, hidden: int = 16) -> None:
        super().__init__()
        self.net = nn.Sequential(nn.Linear(3, hidden), nn.Tanh(), nn.Linear(hidden, 2))

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        return self.net(features)


class TinyImageController(nn.Module):
    """End-to-end RGB sensor to velocity command network / 图像到控制指令网络。"""

    def __init__(self) -> None:
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 8, 5, stride=2, padding=2), nn.ReLU(),
            nn.Conv2d(8, 16, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(16, 24, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.head = nn.Sequential(nn.Flatten(), nn.Linear(24, 16), nn.Tanh(), nn.Linear(16, 2))

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        return self.head(self.encoder(images))
