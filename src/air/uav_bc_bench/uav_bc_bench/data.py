"""数据集读取：行为克隆用的 (图像, 控制量) 配对。"""

import csv
import os

import cv2
import numpy as np


def _to_meters(img):
    """把一张深度/灰度图转成 [0, 1] 的单通道浮点图。

    16 位 PNG 是毫米单位的深度图，先转米再除以 40 m 截断；
    8 位 RGB 图（任务 4 的数据）转灰度后直接归一化。
    """
    if img.dtype == np.uint16:
        depth = img.astype(np.float32) / 1000.0
        depth = np.clip(depth, 0.0, 40.0) / 40.0
        return depth
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img.astype(np.float32) / 255.0


def load_dataset(data_dir, image_size=64):
    """读取 controls.csv 与 frames/ 下的图像，返回 (images, targets)。"""
    csv_path = os.path.join(data_dir, 'controls.csv')
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f'找不到 {csv_path}')

    images = []
    targets = []
    with open(csv_path, encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            path = os.path.join(data_dir, row['image'])
            img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if img is None:
                continue
            img = _to_meters(img)
            img = cv2.resize(img, (image_size, image_size), interpolation=cv2.INTER_AREA)
            images.append(img[None, :, :])
            targets.append((float(row['vx']), float(row['vy'])))

    if not images:
        raise RuntimeError('数据集中没有可用样本')

    return np.stack(images).astype(np.float32), np.asarray(targets, dtype=np.float32)
