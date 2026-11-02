"""dataset.py —— 采集多模态数据（图像特征+IMU→二分类标签）"""
from __future__ import annotations
import os
import numpy as np
from .sim import FallWarningSim


def _downsample(img, size=16):
    h, w = img.shape
    bh, bw = h//size, w//size
    crop = img[:bh*size, :bw*size]
    return crop.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten().astype(np.float32)


def collect(model_path, out_path, n_samples=2000, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    sim = FallWarningSim(model_path)
    X, y = [], []
    for i in range(n_samples):
        sim.reset()
        for _ in range(200): sim.step()
        ang = rng.choice([0, 90, 180, 270])
        f = rng.uniform(50, 200)
        rad = np.radians(ang)
        sim.apply_push((f*np.cos(rad), f*np.sin(rad), 0), duration_steps=30)
        for _ in range(rng.integers(20, 100)): sim.step()
        img_feat = _downsample(sim.render_front(gray=True))
        pitch, roll = sim.torso_pose()
        x = np.concatenate([img_feat, [pitch, roll]])
        # 标签：根高<0.9或倾角>0.3 → 即将跌倒(1)，否则正常(0)
        label = 1 if (sim.root_z < 0.9 or abs(pitch) > 0.3 or abs(roll) > 0.3) else 0
        X.append(x); y.append(label)
        if verbose and i % 200 == 0: print(f"  [collect] {i}/{n_samples}")
    X = np.stack(X).astype(np.float32)
    y = np.array(y, dtype=np.int64)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y)
    sim.close()
    print(f"[collect] saved {out_path}: X={X.shape}, y={y.shape}")
    return out_path
