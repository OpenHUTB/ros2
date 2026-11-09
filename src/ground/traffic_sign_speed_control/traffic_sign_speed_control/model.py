# -*- coding: utf-8 -*-
"""字符识别 CNN（PyTorch）与权重加载。

DigitCNN：两层卷积 + 两层全连接，输入 32x32 灰度字符图，输出 0-9 十类。
"""
import os

import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F

MODEL_NAME = "sign_cnn.pth"


class DigitCNN(nn.Module):
    """轻量字符分类网络。

    结构：Conv(1->16,3x3)-ReLU-MaxPool2 -> Conv(16->32,3x3)-ReLU-MaxPool2
          -> Dropout -> FC(2048->128) -> ReLU -> FC(128->10)
    输入 [N,1,32,32]，像素 [0,1]。
    """

    def __init__(self, num_classes: int = 10):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 16, 3, padding=1)
        self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
        self.drop = nn.Dropout(0.25)
        self.fc1 = nn.Linear(32 * 8 * 8, 128)
        self.fc2 = nn.Linear(128, num_classes)

    def forward(self, x):
        x = F.max_pool2d(F.relu(self.conv1(x)), 2)   # 32 -> 16
        x = F.max_pool2d(F.relu(self.conv2(x)), 2)   # 16 -> 8
        x = self.drop(x)
        x = x.flatten(1)
        x = F.relu(self.fc1(x))
        return self.fc2(x)


def find_model_path():
    """优先找安装后的 share 目录，再回退到源码目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("traffic_sign_speed_control"),
                         "models", MODEL_NAME)
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "models", MODEL_NAME)
    return p


def load_model(device=None):
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    model = DigitCNN().to(device)
    path = find_model_path()
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"未找到权重 {MODEL_NAME}，请先运行 scripts/train_signs.py：{path}")
    model.load_state_dict(torch.load(path, map_location=device))
    model.eval()
    return model, device


def preprocess_char(gray_u8):
    """字符裁剪图（HxW uint8 灰度）-> [1,1,32,32] 张量。

    与训练共用 canonicalize_char 归一化，保证训练/推理分布一致。
    """
    from .sign_render import canonicalize_char
    if gray_u8 is None or gray_u8.size == 0:
        return None
    g = canonicalize_char(np.asarray(gray_u8), size=32)
    if g is None:
        return None
    g = g.astype(np.float32) / 255.0
    return torch.from_numpy(g[None, None, :, :])
