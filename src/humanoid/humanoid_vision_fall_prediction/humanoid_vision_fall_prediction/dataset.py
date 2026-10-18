"""
dataset.py —— 数据集采集
========================

随机施加不同方向/大小的外力，采集前视相机图像与"跌倒方向"标签。

5 类标签：
  0 = normal   无外力或小推力（正常站立）
  1 = forward  前向推力 (+x)
  2 = backward 后向推力 (-x)
  3 = left     左向推力 (+y)
  4 = right    右向推力 (-y)

采集逻辑：
  * 每个方向随机采样 force ∈ [fmin, fmax]；
  * 施加推力 0.08s 后继续仿真若干步，截取推力后 0.2s 内的前视图像；
  * 小推力（< threshold）采集为 normal 类；
  * 推力方向即为标签（"即将向该方向倒"）。
"""

from __future__ import annotations

import os
import numpy as np

from .sim import VisionFallSim

# 4 个推力方向（角度，度）：0=前, 180=后, 90=左, 270=右
PUSH_DIRECTIONS_DEG = [0.0, 180.0, 90.0, 270.0]
DIR_TO_LABEL = {0.0: 1, 180.0: 2, 90.0: 3, 270.0: 4}


def _downsample_gray(img: np.ndarray, size: int = 16) -> np.ndarray:
    """把 (H,W) 灰度图平均池化到 size×size，flatten 返回 (size*size,)。"""
    h, w = img.shape
    bh = h // size
    bw = w // size
    cropped = img[:bh * size, :bw * size]
    pooled = cropped.reshape(size, bh, size, bw).mean(axis=(1, 3))
    return pooled.flatten().astype(np.float32)


def collect_dataset(model_path: str, out_path: str,
                    samples_per_class: int = 300,
                    force_min: float = 80.0, force_max: float = 220.0,
                    normal_ratio: float = 0.25,
                    image_h: int = 64, image_w: int = 64,
                    kp: float = 60.0, kd: float = 6.0,
                    seed: int = 0, verbose: bool = True) -> str:
    """采集数据集并保存为 .npz。"""
    rng = np.random.default_rng(seed)
    sim = VisionFallSim(model_path, kp=kp, kd=kd, image_h=image_h, image_w=image_w)

    X_list, y_list = [], []

    # normal 类（小推力或无推）
    n_normal = int(samples_per_class * normal_ratio)
    for _ in range(n_normal):
        sim.reset()
        # 先自由站立 0.5s 稳定
        for _ in range(250):
            sim.step()
        # 小推力或无推
        if rng.random() < 0.5:
            f = rng.uniform(0, 40.0)
            ang = rng.choice(PUSH_DIRECTIONS_DEG)
            fx = f * np.cos(np.radians(ang))
            fy = f * np.sin(np.radians(ang))
            sim.apply_push((fx, fy, 0.0), duration_steps=30)
        # 继续走 100 步，取中间图像
        for _ in range(100):
            sim.step()
        img = sim.render_front(gray=True)
        X_list.append(_downsample_gray(img))
        y_list.append(0)

    # 4 个方向类
    n_per_dir = (samples_per_class - n_normal) // 4
    for ang in PUSH_DIRECTIONS_DEG:
        label = DIR_TO_LABEL[ang]
        for _ in range(n_per_dir):
            sim.reset()
            for _ in range(250):
                sim.step()
            f = rng.uniform(force_min, force_max)
            fx = f * np.cos(np.radians(ang))
            fy = f * np.sin(np.radians(ang))
            sim.apply_push((fx, fy, 0.0), duration_steps=40)
            # 推力后再走 100 步（0.2s），取一帧
            for _ in range(100):
                sim.step()
            if sim.fallen():
                # 跌倒了就用跌倒前最后一帧（重置后取推力后 50 步）
                sim.reset()
                for _ in range(250):
                    sim.step()
                sim.apply_push((fx, fy, 0.0), duration_steps=40)
                for _ in range(50):
                    sim.step()
            img = sim.render_front(gray=True)
            X_list.append(_downsample_gray(img))
            y_list.append(label)

            if verbose and (len(y_list) % 100 == 0):
                print(f"  [collect] 已采集 {len(y_list)} 样本...")

    X = np.stack(X_list).astype(np.float32)
    y = np.array(y_list, dtype=np.int64)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y)
    sim.close()

    if verbose:
        counts = np.bincount(y, minlength=5)
        print(f"[collect] 数据集已保存: {out_path}")
        print(f"  总样本: {len(y)} | 各类分布: {counts.tolist()}")
        print(f"  输入维度: {X.shape[1]}")
    return out_path
