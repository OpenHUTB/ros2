"""dataset.py —— 采集深度图+3D姿态标签"""
from __future__ import annotations
import os
import numpy as np
from .sim import DepthPoseSim


def _downsample(img, size=16):
    """将深度图块平均下采样为 size×size 网格（输出 size² 维向量）。
    要求图像尺寸能被 size 整除；不能整除时先裁剪。"""
    h, w = img.shape
    bh, bw = h//size, w//size
    crop = img[:bh*size, :bw*size]
    return crop.reshape(size, bh, size, bw).mean(axis=(1, 3)).flatten().astype(np.float32)


def collect(model_path, out_path, n_samples=2000, force_min=60, force_max=200,
            seed=0, standing_ratio=0.2, verbose=True):
    """采集训练数据。

    Args:
        standing_ratio: 无外力纯站立样本占比（负例），默认20%。
    """
    rng = np.random.default_rng(seed)
    sim = DepthPoseSim(model_path)
    X, y = [], []
    n_standing = int(n_samples * standing_ratio)
    for i in range(n_samples):
        sim.reset()
        for _ in range(200):
            sim.step()
        # 20%样本不施加外力，纯站立微小晃动
        if i >= n_standing:
            ang = rng.uniform(0, 360)  # 连续角度采样，而非4个固定方向
            f = rng.uniform(force_min, force_max)
            rad = np.radians(ang)
            sim.apply_push((f*np.cos(rad), f*np.sin(rad), 0), duration_steps=30)
            for _ in range(rng.integers(20, 120)):
                sim.step()
        if sim.fallen():
            sim.reset()
            for _ in range(200): sim.step()
        depth = sim.render_depth()
        X.append(_downsample(depth))
        pose = sim.torso_pose()
        y.append(pose)
        if verbose and i % 200 == 0:
            print(f"  [collect] {i}/{n_samples}")
    X = np.stack(X).astype(np.float32)
    y = np.array(y, dtype=np.float32)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y)
    sim.close()
    if verbose:
        print(f"[collect] saved {out_path}: X={X.shape}, y={y.shape}")
    return out_path
