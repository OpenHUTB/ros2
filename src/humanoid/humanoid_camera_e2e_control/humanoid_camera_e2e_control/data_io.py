# -*- coding: utf-8 -*-
"""
data_io.py —— 采集数据集读写、图像预处理、指令分类与评测指标。

数据集按批次独立存储（避免全量 concat 重写带来的 O(n²) 开销）：
    data/training/images_{batch:04d}.npy  uint8  B×H×W×3  跟随视角相机原图
    data/training/labels_{batch:04d}.npy  float32 B×2  物理指令 (v m/s, w rad/s)

归一化口径（训练 / 推理必须一致）：
    图像：逐帧零均值、单位方差后裁剪到约 [-1, 1]（见 normalize_image）
    指令：u_v = v / v_max，u_w = w / w_max，范围 [-1, 1]
"""

import glob
import os

import numpy as np
import torch
from torch.utils.data import Dataset

from .sim_core import IMG_H, IMG_W, HumanoidSim


def normalize_image(img_uint8: np.ndarray) -> np.ndarray:
    """H×W×3 uint8 → H×W×3 float32，逐帧零均值单位方差后裁剪到约 [-1, 1]。

    跟随相机不同时刻光照/明暗略有差异，逐帧归一化可抵消全局亮度漂移，
    让卷积网络聚焦障碍的左右方位而非绝对亮度。训练与推理必须使用同一口径。
    """
    img = img_uint8.astype("float32") / 255.0
    mean = float(img.mean())
    std = float(img.std()) + 1e-3
    img = (img - mean) / std
    return np.clip(img, -3.0, 3.0) / 3.0


def preprocess(img_uint8: np.ndarray) -> torch.Tensor:
    """H×W×3 uint8 → 3×H×W float32 ∈ [-1, 1]（单帧，无 batch 维）。"""
    img = normalize_image(img_uint8)
    return torch.from_numpy(img).permute(2, 0, 1)


def direction_label(v: float, w: float) -> int:
    """三态方向标签：0=停止，1=左转(w>0)，2=右转(w<0)。"""
    if v < 0.03 and abs(w) < 0.05:
        return 0
    if w > 0.05:
        return 1
    if w < -0.05:
        return 2
    return 0


def save_dataset(images: np.ndarray, vels: np.ndarray, yaws: np.ndarray,
                 out_dir: str, batch: int = 0) -> None:
    """保存一个采集批次到独立 npy 文件，不重写历史批次。"""
    os.makedirs(out_dir, exist_ok=True)
    labels = np.stack([vels, yaws], axis=1).astype("float32")
    np.save(os.path.join(out_dir, f"images_{batch:04d}.npy"), images)
    np.save(os.path.join(out_dir, f"labels_{batch:04d}.npy"), labels)
    print(f"[数据集] 第 {batch} 批 {len(images)} 帧 -> {out_dir}")


def clear_dataset(data_dir: str) -> None:
    """清空历史批次（重新采集前调用）。"""
    for f in glob.glob(os.path.join(data_dir, "images_*.npy")) + \
             glob.glob(os.path.join(data_dir, "labels_*.npy")):
        os.remove(f)


def load_dataset(data_dir: str):
    """拼接全部批次，返回 (images uint8 N×H×W×3, labels float32 N×2)；无数据返回 None。"""
    img_files = sorted(glob.glob(os.path.join(data_dir, "images_*.npy")))
    if not img_files:
        return None
    images, labels = [], []
    for ip in img_files:
        images.append(np.load(ip))
        lp = ip.replace("images_", "labels_")
        labels.append(np.load(lp))
    return np.concatenate(images, axis=0), np.concatenate(labels, axis=0)


class DrivingDataset(Dataset):
    """图像—指令对数据集（PyTorch Dataset）。"""

    def __init__(self, images: np.ndarray, labels: np.ndarray,
                 v_max: float = HumanoidSim.V_MAX, w_max: float = HumanoidSim.W_MAX):
        self.images = images
        self.labels = labels.astype("float32")
        self.v_max, self.w_max = v_max, w_max

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int):
        img = normalize_image(self.images[idx])
        v, w = self.labels[idx]
        # 水平翻转增强默认关闭：翻转后左右镜像与 w 反号的对应关系易造成小网络混淆，
        # 先学习干净的"跟随视角图像—指令"映射；如需开启可置 self.flip_aug=True。
        if getattr(self, "flip_aug", False) and np.random.rand() < 0.5:
            img = img[:, ::-1, :]
            w = -w
        x = torch.from_numpy(img.copy()).permute(2, 0, 1)
        y = torch.tensor([v / self.v_max, w / self.w_max], dtype=torch.float32)
        cls = torch.tensor(direction_label(v, w), dtype=torch.long)
        return x, y, cls
