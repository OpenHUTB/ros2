"""PilotNet CNN 模型定义

NVIDIA PilotNet 架构，用于端到端自动驾驶控制。
输入：RGB 图像 (66, 200, 3)
输出：转向角 + 油门（2 维）
"""
import torch
import torch.nn as nn


class PilotNet(nn.Module):
    def __init__(self, input_h=66, input_w=200):
        super(PilotNet, self).__init__()

        self.features = nn.Sequential(
            # 归一化层
            nn.BatchNorm2d(3),
            # Conv1: 5x5, stride 2, 24 filters
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            # Conv2: 5x5, stride 2, 36 filters
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            # Conv3: 5x5, stride 2, 48 filters
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            # Conv4: 3x3, 64 filters
            nn.Conv2d(48, 64, kernel_size=3),
            nn.ELU(inplace=True),
            # Conv5: 3x3, 64 filters
            nn.Conv2d(64, 64, kernel_size=3),
            nn.ELU(inplace=True),
            nn.Dropout(0.2),
        )

        # 计算卷积输出尺寸
        with torch.no_grad():
            dummy = torch.zeros(1, 3, input_h, input_w)
            out = self.features(dummy)
            self.flatten_dim = out.view(1, -1).shape[1]

        self.classifier = nn.Sequential(
            nn.Linear(self.flatten_dim, 100),
            nn.ELU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(100, 50),
            nn.ELU(inplace=True),
            nn.Linear(50, 10),
            nn.ELU(inplace=True),
            nn.Linear(10, 2),  # 输出: steering, throttle
        )

    def forward(self, x):
        x = self.features(x)
        x = x.reshape(x.size(0), -1)
        x = self.classifier(x)
        return x


if __name__ == '__main__':
    model = PilotNet()
    total_params = sum(p.numel() for p in model.parameters())
    print(f"PilotNet 参数量: {total_params:,}")
    dummy = torch.randn(1, 3, 66, 200)
    out = model(dummy)
    print(f"输入: {dummy.shape}, 输出: {out.shape}")
    print(f"输出示例: steering={out[0,0].item():.4f}, throttle={out[0,1].item():.4f}")
