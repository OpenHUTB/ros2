"""Lightweight CNN for visual steering / 视觉转向轻量卷积网络。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import torch
from torch import nn


class LightweightSteeringCNN(nn.Module):
    """Small image classifier with a stable four-class output contract."""

    labels = ("LEFT", "STRAIGHT", "RIGHT", "STOP")

    def __init__(self, image_height: int = 90, image_width: int = 160) -> None:
        super().__init__()
        if image_height <= 0 or image_width <= 0:
            raise ValueError("image dimensions must be positive / 图像尺寸必须为正数")
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 48, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4)),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(48 * 4 * 4, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.1),
            nn.Linear(64, len(self.labels)),
        )
        self.image_height = image_height
        self.image_width = image_width

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        if images.ndim != 4 or images.shape[1] != 3:
            raise ValueError("expected BCHW RGB tensor / 需要 BCHW RGB 张量")
        return self.classifier(self.features(images))

    @classmethod
    def load_checkpoint(cls, path: Path, device: str = "cpu") -> "LightweightSteeringCNN":
        checkpoint: dict[str, Any] = torch.load(path, map_location=device, weights_only=True)
        model = cls(int(checkpoint["image_height"]), int(checkpoint["image_width"]))
        model.load_state_dict(checkpoint["state_dict"])
        model.to(device)
        model.eval()
        return model


