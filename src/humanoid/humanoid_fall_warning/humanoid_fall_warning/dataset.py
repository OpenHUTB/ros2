"""dataset.py —— 采集多模态数据（图像特征+IMU→二分类标签）

标签定义：施加外力后继续仿真N步，若最终跌倒(1)，否则正常(0)。
避免直接用当前IMU阈值作为标签，防止标签泄漏。
"""
from __future__ import annotations
import os
import numpy as np
from .sim import FallWarningSim


def _downsample(img, size=16):
    """将深度图块平均下采样为 size×size 网格（输出 size² 维向量）。"""
    h, w = img.shape
    bh, bw = h//size, w//size
    crop = img[:bh*size, :bw*size]
    return crop.reshape(size, bh, size, bw).mean(axis=(1, 3)).flatten().astype(np.float32)


def collect(model_path, out_path, n_samples=2000, seed=0, standing_ratio=0.2,
             horizon=20, verbose=True):
    """采集多模态训练数据。

    Args:
        standing_ratio: 无外力纯站立样本占比（负例），默认20%。
        horizon: 施加外力后继续仿真的步数，用于判断是否会跌倒。
    """
    rng = np.random.default_rng(seed)
    sim = FallWarningSim(model_path)
    X, y = [], []
    n_standing = int(n_samples * standing_ratio)
    for i in range(n_samples):
        sim.reset()
        for _ in range(200): sim.step()
        # 记录当前帧的图像和IMU（这是网络的输入）
        img_feat = _downsample(sim.render_front(gray=True))
        pitch, roll = sim.torso_pose()
        x = np.concatenate([img_feat, [pitch, roll]])

        # 20%样本不施加外力，纯站立（负例）
        will_fall = 0
        if i >= n_standing:
            ang = rng.uniform(0, 360)  # 连续角度采样
            f = rng.uniform(50, 200)
            rad = np.radians(ang)
            sim.apply_push((f*np.cos(rad), f*np.sin(rad), 0), duration_steps=30)
            # 继续仿真horizon步，判断最终是否跌倒
            for _ in range(horizon):
                sim.step()
            if sim.fallen():
                will_fall = 1

        X.append(x)
        y.append(will_fall)
        if verbose and i % 200 == 0: print(f"  [collect] {i}/{n_samples}")

    X = np.stack(X).astype(np.float32)
    y = np.array(y, dtype=np.int64)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y)
    sim.close()
    print(f"[collect] saved {out_path}: X={X.shape}, y={y.shape}, pos_rate={y.mean():.2f}")
    return out_path
