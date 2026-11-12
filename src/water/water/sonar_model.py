"""声呐图像分类网络定义（训练与推理共用，避免两份代码漂移）。"""

import torch
import torch.nn as nn


class SonarCNN(nn.Module):
    """轻量 3 层卷积网络：输入 3x256x256，输出 num_classes 个 logits。"""

    def __init__(self, num_classes=3, input_size=256):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1),   # 3x256x256 -> 16x256x256
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                  # -> 16x128x128
            nn.Conv2d(16, 32, 3, padding=1),  # -> 32x128x128
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                  # -> 32x64x64
            nn.Conv2d(32, 64, 3, padding=1),  # -> 64x64x64
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                  # -> 64x32x32
        )
        # 每次池化减半，共 3 次，所以特征图边长为 input_size // 8
        feat_dim = 64 * (input_size // 8) ** 2
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(feat_dim, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(128, num_classes),
        )

    def forward(self, x):
        return self.classifier(self.features(x))


def load_model(weight_path, num_classes=3, input_size=256, device="cpu"):
    """加载权重并返回 eval 模式模型；加载失败直接抛异常，不做随机权重降级。"""
    model = SonarCNN(num_classes=num_classes, input_size=input_size)
    state = torch.load(weight_path, map_location=device, weights_only=True)
    model.load_state_dict(state)
    model.to(device)
    model.eval()
    return model
