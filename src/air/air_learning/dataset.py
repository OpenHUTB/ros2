#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 数据集加载模块：dataset（任务四：端到端视觉行为克隆，第二阶段·数据加载）
# 作用：把 record_dataset.py 落盘的数据（data/dataset/images/*.png +
#       data/dataset/labels.csv）封装成 PyTorch Dataset：
#   labels.csv 列：frame,timestamp,vx,vy,vz,yaw_rate
#   frame 列对应 images/frame_000001.png 等图像文件
# 预处理流水线（BGR 读图 -> RGB -> Resize 224x224 -> ToTensor -> ImageNet
# 标准化）收敛在唯一的 preprocess_image 函数里，训练端与部署端
# （drone_autonomous_node.py）共用同一函数，保证两端变换严格一致。
# 训练/验证按 8:2 划分：固定随机种子，多次实例化划分结果一致，
# 且两集合互不重叠，杜绝验证集泄漏进训练集。
# 不依赖 torchvision：ToTensor 与标准化用 numpy + torch 手写实现。

import csv
import os
import random

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset

IMG_SIZE = 224                    # 统一输入分辨率（模型输入 3x224x224）
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


def preprocess_image(img_rgb):
    """把 HxWx3 RGB 图像转成模型输入张量 [3, 224, 224]

    训练与推理共用此函数：Resize 到 224x224（直接拉伸）-> ToTensor（0~1）
    -> ImageNet 标准化。推理端在此结果上 unsqueeze(0) 补批维即可。
    """
    img = cv2.resize(img_rgb, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_LINEAR)
    tensor = torch.from_numpy(img.astype(np.float32) / 255.0).permute(2, 0, 1)
    mean = torch.tensor(IMAGENET_MEAN, dtype=tensor.dtype).view(3, 1, 1)
    std = torch.tensor(IMAGENET_STD, dtype=tensor.dtype).view(3, 1, 1)
    return (tensor - mean) / std


class DroneDataset(Dataset):
    """行为克隆数据集：前视图像 -> 标签向量 [vx, vy, vz, yaw_rate]

    root 目录下应包含 labels.csv 与 images/。加载时自动跳过图像文件缺失的
    标签行（逐行校验存在性），避免训练中途坏帧崩溃；全部缺失时报错退出。
    """

    def __init__(self, root='data/dataset', split='train',
                 val_ratio=0.2, seed=42):
        if split not in ('train', 'val'):
            raise ValueError("split 只能是 'train' 或 'val'，收到 %r" % split)
        self.root = root
        self.images_dir = os.path.join(root, 'images')
        self.csv_path = os.path.join(root, 'labels.csv')
        if not os.path.isfile(self.csv_path):
            raise FileNotFoundError(
                '找不到标签文件 %s，请先在仓库根目录运行 record_dataset 采集数据'
                % os.path.abspath(self.csv_path))
        self.rows = self._load_rows()
        if not self.rows:
            raise ValueError('数据集为空：%s 中没有一条标签能对应到实际图像文件'
                             % os.path.abspath(self.csv_path))

        # 8:2 划分：同一固定种子下 train/val 实例的洗牌结果一致，互不重叠
        order = list(range(len(self.rows)))
        random.Random(seed).shuffle(order)
        n_val = max(1, int(round(len(order) * val_ratio))) if len(order) > 1 else 0
        self.index = order[:n_val] if split == 'val' else order[n_val:]

    def _load_rows(self):
        """逐行读取 labels.csv，返回 (图像路径, [vx,vy,vz,yaw_rate]) 有效样本列表"""
        rows = []
        missing = 0
        with open(self.csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    frame = row['frame']
                    img_path = os.path.join(self.images_dir, frame)
                    if not os.path.isfile(img_path):
                        missing += 1
                        continue
                    label = [float(row['vx']), float(row['vy']),
                             float(row['vz']), float(row['yaw_rate'])]
                except (KeyError, ValueError):
                    continue          # 空行/坏行直接跳过
                rows.append((img_path, label))
        if missing:
            print('[dataset] 跳过 %d 条图像文件缺失的标签行' % missing)
        return rows

    def __len__(self):
        return len(self.index)

    def __getitem__(self, idx):
        img_path, label = self.rows[self.index[idx]]
        # record_dataset 用 cv_bridge bgr8 落盘，因此这里是 BGR -> RGB
        img_bgr = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img_bgr is None:
            raise ValueError('图像读取失败：%s' % img_path)   # 加载时已过滤，此路不应触发
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        image = preprocess_image(img_rgb)                     # [3, 224, 224]
        label = torch.tensor(label, dtype=torch.float32)      # [4]
        return image, label


def make_dataloaders(root='data/dataset', val_ratio=0.2, seed=42,
                     batch_size=16, num_workers=0):
    """创建 (train_loader, val_loader)

    num_workers 默认 0：避免 Windows 上多进程 DataLoader 的 spawn 开销与坑，
    小数据集下单进程加载已足够。
    """
    train_ds = DroneDataset(root, split='train', val_ratio=val_ratio, seed=seed)
    val_ds = DroneDataset(root, split='val', val_ratio=val_ratio, seed=seed)
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              num_workers=num_workers)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False,
                            num_workers=num_workers)
    return train_loader, val_loader
