"""深度相机 + 参考轨迹的跟踪网络：全仓库唯一定义网络结构与特征的地方。

数据流：
    深度图 (H, W) -> 裁剪到 [0, max_depth] -> 归一化 -> 1x64x64
    参考轨迹偏差 (dx, dy) + 航向误差 (sin, cos) -> 4 维特征
        └──> TrackNet ──> (vx, vy)  机体系速度（m/s）

为什么把偏差也喂给网络：单张深度图里没有"目标在哪"的信息。给定轨迹控制的任务是
「沿着这条轨迹飞」，目标点相对机体的位置属于任务输入而不是图像内容，所以显式拼接
4 个标量，网络只需要学"看到这样的障碍分布 + 目标在右前方 -> 该给多大速度"。

训练脚本与推理节点都从这里 import，避免两份预处理漂移导致权重加载失败。
"""

import math
import os

import cv2
import numpy as np
import torch
import torch.nn as nn

DEFAULT_IMAGE_SIZE = 64
DEFAULT_OUTPUT_DIM = 2
DEFAULT_FEATURE_DIM = 4
DEFAULT_MAX_DEPTH = 40.0


# --------------------------------------------------------------------- 特征
def track_features(pose_xy, yaw, target_xy, scale=5.0):
    """把目标点换算成机体坐标系下的 4 维特征。

    Args:
        pose_xy: (x, y)，当前世界坐标（NED，米）。
        yaw: 当前偏航角（弧度，NED 下顺时针为正）。
        target_xy: 前视点世界坐标。
        scale: 位置特征归一化尺度（米）。

    Returns:
        np.ndarray，shape (4,)，依次为
        ``dx/scale``、``dy/scale``、``sin(err)``、``cos(err)``，
        其中 ``err`` 是机体前向轴到目标方向的夹角。
    """
    wx = float(target_xy[0]) - float(pose_xy[0])
    wy = float(target_xy[1]) - float(pose_xy[1])

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    # 世界系 -> 机体系：机体 x 轴指向机头，y 轴指向右侧（NED 下顺时针为正）
    dx = wx * cos_yaw + wy * sin_yaw
    dy = -wx * sin_yaw + wy * cos_yaw
    err = math.atan2(dy, dx)

    return np.array([dx / scale, dy / scale, math.sin(err), math.cos(err)],
                    dtype=np.float32)


def preprocess_depth(depth, image_size=DEFAULT_IMAGE_SIZE,
                     max_depth=DEFAULT_MAX_DEPTH):
    """把一帧 float32 深度图转换成网络输入张量。

    AirSim 的 ``DepthPerspective`` 单位是米，天空会给一个很大的值（接近 1e4），
    直接除以 255 没有意义，所以先裁剪到 ``max_depth`` 再归一化到 [0, 1]。
    """
    if depth is None:
        raise ValueError('输入深度图为空')

    array = np.asarray(depth, dtype=np.float32)
    if array.ndim != 2:
        raise ValueError(f'深度图应为二维，实际 {array.shape}')

    array = np.nan_to_num(array, nan=max_depth, posinf=max_depth, neginf=0.0)
    array = np.clip(array, 0.0, max_depth) / max_depth
    array = cv2.resize(array, (image_size, image_size),
                       interpolation=cv2.INTER_AREA)
    return array.reshape(1, 1, image_size, image_size).astype(np.float32)


def reference_point(center, radius, height, phase):
    """圆轨迹上相位为 ``phase`` 的点（NED，z 向下为正，所以高度取负）。"""
    cx, cy = float(center[0]), float(center[1])
    return (cx + radius * math.cos(phase),
            cy + radius * math.sin(phase),
            -abs(float(height)))


# --------------------------------------------------------------------- 网络
class TrackNet(nn.Module):
    """图像分支（3 层卷积）+ 标量分支（4 维特征）的回归网络。

    输入：``image`` (N, 1, S, S)、``features`` (N, 4)
    输出：(N, 2)，机体系前向/右向速度（m/s）
    """

    def __init__(self, image_size=DEFAULT_IMAGE_SIZE,
                 output_dim=DEFAULT_OUTPUT_DIM,
                 feature_dim=DEFAULT_FEATURE_DIM):
        super().__init__()
        if image_size % 8 != 0:
            raise ValueError('image_size 必须是 8 的倍数（要经过 3 次 2 倍下采样）')

        self.image_size = image_size
        self.output_dim = output_dim
        self.feature_dim = feature_dim

        self.cnn = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2, 2),
        )

        flat_dim = 64 * (image_size // 8) * (image_size // 8)
        self.image_head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flat_dim, 128),
            nn.ReLU(inplace=True),
        )
        self.head = nn.Sequential(
            nn.Linear(128 + feature_dim, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, output_dim),
        )

    def forward(self, image, features):
        """前向：返回 (N, 2)。"""
        visual = self.image_head(self.cnn(image))
        return self.head(torch.cat([visual, features], dim=1))

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
                f'模型文件不存在: {path}，请先运行 ros2 run uav_depth_track_nn train')

        model = cls(image_size=image_size)
        state = torch.load(path, map_location=device)
        model.load_state_dict(state)
        model.to(device)
        model.eval()
        return model

    def num_parameters(self):
        """可训练参数总量，写进训练指标便于对比模型规模。"""
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
