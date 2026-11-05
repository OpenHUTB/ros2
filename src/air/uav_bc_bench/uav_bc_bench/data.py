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


def synthesize_dataset(out_dir, samples=400, image_size=64, seed=0):
    """生成一份可独立复现的合成数据集，供没跑过仿真时做对比实验。

    图像是一面会左右移动的"墙"（灰度越亮 = 越近），控制量取解析律：
    墙偏右时向左修正、离墙越近越慢。真实作业里用的是任务 2 采到的深度图，
    这里只是让基准能在没有仿真器的机器上跑通并给出可比较的数字。
    """
    rng = np.random.default_rng(seed)
    frame_dir = os.path.join(out_dir, 'frames')
    os.makedirs(frame_dir, exist_ok=True)

    rows = []
    for i in range(int(samples)):
        # 墙的横向位置在 [-1, 1] 之间缓慢来回移动，贴近真实飞行时的连续变化
        phase = 2.0 * np.pi * i / max(1, int(samples) // 2)
        offset = float(np.sin(phase))
        depth = np.ones((image_size, image_size), np.float32) * 0.95
        width = max(2, image_size // 6)
        center = int((offset * 0.5 + 0.5) * (image_size - width))
        depth[:, center:center + width] = 0.15 + 0.1 * abs(offset)
        depth += rng.normal(0.0, 0.02, depth.shape).astype(np.float32)
        img = np.clip(depth, 0.0, 1.0) * 255.0
        name = f'frames/syn_{i:05d}.png'
        cv2.imwrite(os.path.join(out_dir, name), img.astype(np.uint8))

        vx = float(np.clip(0.9 - 0.4 * abs(offset), 0.1, 1.0))
        vy = float(np.clip(-1.6 * offset, -1.2, 1.2))
        rows.append({'image': name, 'vx': f'{vx:.3f}', 'vy': f'{vy:.3f}',
                     'x': f'{offset:.3f}', 'y': '0.000', 'z': '-2.500', 'stamp': f'{i * 0.2:.3f}'})

    csv_path = os.path.join(out_dir, 'controls.csv')
    with open(csv_path, 'w', encoding='utf-8', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=['image', 'vx', 'vy', 'x', 'y', 'z', 'stamp'])
        writer.writeheader()
        writer.writerows(rows)
    return out_dir
