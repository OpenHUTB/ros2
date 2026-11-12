"""CNN 感知模型：前视 RGB 图像 -> 3 维归一化控制量 [vx_n, vy_n, yaw_n] ∈ [-1,1]。"""
import torch
import torch.nn as nn


class DronePerceptionNet(nn.Module):
    def __init__(self, input_channels=3, out_dim=3, dropout=0.3):
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
            nn.Dropout(dropout),
            nn.Linear(128, out_dim),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.regressor(self.features(x))


def build_model(dropout=0.3):
    return DronePerceptionNet(input_channels=3, out_dim=3, dropout=dropout)
