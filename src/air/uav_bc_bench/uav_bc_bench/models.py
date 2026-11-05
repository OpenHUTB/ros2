"""参与对比的三种行为克隆网络结构。

所有结构都接受 ``image_size`` 参数，卷积后的展平维度由 ``image_size // 8`` 计算，
这样把 ``--image-size`` 改成别的值（例如 32 或 96）时不会出现维度不匹配。
"""

import torch
from torch import nn


def _conv_out_dim(image_size, pools=3):
    """三层 2x2 池化之后特征图边长。"""
    size = int(image_size)
    for _ in range(pools):
        size //= 2
    if size < 1:
        raise ValueError(f'image_size={image_size} 太小，三层池化后特征图为空')
    return size


class MLPNet(nn.Module):
    """把整幅图展平成向量，只用全连接层。"""

    def __init__(self, image_size=64, hidden=256):
        super().__init__()
        self.features = nn.Sequential(
            nn.Flatten(),
            nn.Linear(int(image_size) * int(image_size), hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, hidden),
            nn.ReLU(inplace=True),
        )
        self.head = nn.Linear(hidden, 2)

    def forward(self, x):
        return self.head(self.features(x))


class CNNNet(nn.Module):
    """最简卷积结构：16/32/64 三层，与任务 4 的 DrivingCNN 一致。"""

    def __init__(self, image_size=64):
        super().__init__()
        flat = 64 * _conv_out_dim(image_size) ** 2
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, 3, padding=1), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Flatten(),
            nn.Linear(flat, 128), nn.ReLU(inplace=True),
        )
        self.head = nn.Linear(128, 2)

    def forward(self, x):
        return self.head(self.features(x))


class CNNWideNet(nn.Module):
    """加宽卷积 + 批归一化 + dropout，验证容量与正则化的影响。"""

    def __init__(self, image_size=64):
        super().__init__()
        flat = 128 * _conv_out_dim(image_size) ** 2
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Conv2d(64, 128, 3, padding=1), nn.BatchNorm2d(128), nn.ReLU(inplace=True), nn.MaxPool2d(2),
            nn.Flatten(),
            nn.Dropout(0.2),
            nn.Linear(flat, 256), nn.ReLU(inplace=True),
            nn.Dropout(0.2),
        )
        self.head = nn.Linear(256, 2)

    def forward(self, x):
        return self.head(self.features(x))


MODELS = {
    'mlp': MLPNet,
    'cnn': CNNNet,
    'cnn_wide': CNNWideNet,
}


def build(name, image_size=64):
    """按名字构造网络，``image_size`` 会透传给结构本身。"""
    if name not in MODELS:
        raise KeyError(f'未知结构 {name}，可选 {sorted(MODELS)}')
    return MODELS[name](image_size=image_size)


def num_parameters(model):
    """可训练参数量。"""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)
