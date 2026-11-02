"""dataset.py —— 采集深度图+3D姿态标签"""
from __future__ import annotations
import os
import numpy as np
from .sim import DepthPoseSim


def _downsample(img, size=16):
    h, w = img.shape
    bh, bw = h//size, w//size
    crop = img[:bh*size, :bw*size]
    return crop.reshape(size, bh, size, bw).mean(axis=(1, 3)).flatten().astype(np.float32)


def collect(model_path, out_path, n_samples=2000, force_min=60, force_max=200, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    sim = DepthPoseSim(model_path)
    X, y = [], []
    angles = [0, 90, 180, 270]
    for i in range(n_samples):
        sim.reset()
        for _ in range(200):
            sim.step()
        ang = rng.choice(angles)
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
