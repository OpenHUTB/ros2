"""端到端视觉控制模型：全仓库唯一定义网络结构与预处理的地方。

数据流：BGR 图像 -> 灰度 -> 归一化 -> DrivingCNN -> (vx, vy) 机体系速度（m/s）。

训练脚本与推理节点都从这里 import，避免两份网络定义漂移导致权重加载失败。
"""

import os

import cv2
import numpy as np
import torch
import torch.nn as nn

DEFAULT_IMAGE_SIZE = 64
DEFAULT_OUTPUT_DIM = 2


def preprocess_image(bgr_image, image_size=DEFAULT_IMAGE_SIZE):
    """把一帧图像转换成网络输入张量。

    Args:
        bgr_image: np.ndarray，(H, W, 3) 的 BGR 图或 (H, W) 的灰度图。
        image_size: 网络输入边长，必须是 8 的倍数。

    Returns:
        np.ndarray，shape 为 (1, 1, image_size, image_size) 的 float32，
        取值范围 [0, 1]，直接对应 batch 维为 1 的输入。
    """
    if bgr_image is None:
        raise ValueError('输入图像为空')

    if bgr_image.ndim == 3:
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    elif bgr_image.ndim == 2:
        gray = bgr_image
    else:
        raise ValueError(f'不支持的图像维度: {bgr_image.shape}')

    gray = cv2.resize(gray, (image_size, image_size), interpolation=cv2.INTER_AREA)
    normalized = gray.astype(np.float32) / 255.0
    return normalized.reshape(1, 1, image_size, image_size)


class DrivingCNN(nn.Module):
    """三层卷积 + 两层全连接的回归网络。

    输入: (N, 1, S, S)，S 默认 64
    输出: (N, 2)，依次为机体系前向速度 vx 与右向速度 vy（m/s）
    """

    def __init__(self, image_size=DEFAULT_IMAGE_SIZE, output_dim=DEFAULT_OUTPUT_DIM):
        super().__init__()
        if image_size % 8 != 0:
            raise ValueError('image_size 必须是 8 的倍数（要经过 3 次 2 倍下采样）')

        self.image_size = image_size
        self.output_dim = output_dim

        self.features = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
        )

        flat_dim = 64 * (image_size // 8) * (image_size // 8)
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flat_dim, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, output_dim),
        )

    def forward(self, x):
        """前向：x 为 (N, 1, S, S)，返回 (N, 2)。"""
        return self.regressor(self.features(x))

    def save(self, path):
        """保存权重，自动创建父目录。"""
        path = os.path.expanduser(path)
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        torch.save(self.state_dict(), path)
        return path

    @classmethod
    def load(cls, path, image_size=DEFAULT_IMAGE_SIZE, device='cpu'):
        """按给定输入尺寸重建网络并载入权重，返回 eval 模式的模型。"""
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f'模型文件不存在: {path}，请先运行 ros2 run uav_vision_nn train')

        model = cls(image_size=image_size)
        state = torch.load(path, map_location=device)
        model.load_state_dict(state)
        model.to(device)
        model.eval()
        return model

    def num_parameters(self):
        """可训练参数总量，写进训练指标便于对比模型规模。"""
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
