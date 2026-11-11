"""合成"相机图像"数据集：把环岛汇入场景的几何关系画成 16x16 灰度图。

每张图对应一个 ego 车在汇入路口的时刻：
  - 画面下半部是路面（灰度 ~90）
  - 左右两条车道线向画面中上方透视收敛（亮线）
  - 环岛内来车画成一个亮块，横向位置 = 来车相对 ego 的方位，纵向位置 = 来车距离
  - ego 自身偏移（需要打多少方向）通过车道线在画面中的整体平移体现

标签 (steering, throttle, brake) 由规则式汇入控制器在该帧几何下给出，
作为行为克隆的"专家演示"。这样数据是自洽的：图像里能看到的东西完全决定标签。
"""

from __future__ import annotations

import numpy as np

IMAGE_H, IMAGE_W = 16, 16


def render_frame(lateral_offset: float,
                 oncoming_x: float,
                 oncoming_y: float,
                 seed: int = 0) -> np.ndarray:
    """根据场景几何渲染一帧 16x16 灰度图。

    参数：
      lateral_offset: ego 相对车道中心的横向偏移（米），正 = 偏右
      oncoming_x: 来车在 ego 前方的纵向距离（米），越大越远
      oncoming_y: 来车在 ego 侧方的横向距离（米），正 = 左侧来车
    返回：
      (16, 16) uint8 图像
    """
    rng = np.random.default_rng(seed)
    img = np.full((IMAGE_H, IMAGE_W), 60, dtype=np.uint8)  # 暗路面

    # 路面梯形：下面更宽
    for r in range(IMAGE_H):
        t = r / (IMAGE_H - 1)  # 0=远处 1=近处
        half = int(3 + 5 * t)  # 近处半宽 8，远处 3
        cx = IMAGE_W // 2 + int(np.clip(lateral_offset * 1.5, -4, 4))
        left = max(0, cx - half)
        right = min(IMAGE_W, cx + half)
        img[r, left:right] = 90

        # 车道线：左右边缘亮线
        if left > 0:
            img[r, left] = 220
        if right < IMAGE_W:
            img[r, right - 1] = 220

    # 来车亮块：纵向越远越靠上、越小
    if oncoming_x < 12.0:
        row = int(np.clip(IMAGE_H - 2 - oncoming_x, 1, IMAGE_H - 2))
        # 横向：左侧来车在画面偏左
        col = int(np.clip(IMAGE_W // 2 - oncoming_y * 1.2, 1, IMAGE_W - 2))
        img[max(0, row - 1):row + 1, max(0, col - 1):col + 1] = 240

    # 轻微噪声，模拟相机
    img = np.clip(img.astype(np.int16) + rng.integers(-8, 9, img.shape), 0, 255).astype(np.uint8)
    return img


def rule_label(lateral_offset: float,
               oncoming_x: float,
               oncoming_y: float) -> np.ndarray:
    """规则式汇入专家：根据来车距离/位置给出 (steer, throttle, brake)。

    - 来车近（<6m）且在冲突区 → 刹车让行
    - 来车较远（>9m）→ 给油汇入
    - 中等距离 → 滑行
    - steer 用于修正 ego 横向偏移
    """
    steer = float(np.clip(-lateral_offset * 0.25, -1.0, 1.0))
    if oncoming_x < 6.0:
        throttle, brake = 0.0, 1.0
    elif oncoming_x > 9.0:
        throttle, brake = 0.8, 0.0
    else:
        throttle, brake = 0.25, 0.1
    return np.array([steer, throttle, brake], dtype=np.float64)


def make_dataset(n: int = 800, seed: int = 0):
    """随机采样场景，返回 (images[N,16,16] uint8, labels[N,3] float)。"""
    rng = np.random.default_rng(seed)
    images = np.zeros((n, IMAGE_H, IMAGE_W), dtype=np.uint8)
    labels = np.zeros((n, 3), dtype=np.float64)
    for i in range(n):
        lateral = rng.uniform(-1.5, 1.5)
        # 来车距离覆盖近中远三档，保证刹车/给油/滑行都有样本
        oncoming_x = rng.uniform(2.0, 12.0)
        oncoming_y = rng.uniform(-3.0, 3.0)
        images[i] = render_frame(lateral, oncoming_x, oncoming_y, seed=seed + i)
        labels[i] = rule_label(lateral, oncoming_x, oncoming_y)
    return images, labels


def split_dataset(images, labels, test_ratio: float = 0.2, seed: int = 0):
    """train/test 拆分，返回 (train_img, train_lab, test_img, test_lab)。"""
    rng = np.random.default_rng(seed)
    n = len(images)
    idx = rng.permutation(n)
    n_test = max(1, int(n * test_ratio))
    test_idx, train_idx = idx[:n_test], idx[n_test:]
    return images[train_idx], labels[train_idx], images[test_idx], labels[test_idx]
