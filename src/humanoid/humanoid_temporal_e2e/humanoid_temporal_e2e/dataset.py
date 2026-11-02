"""dataset.py —— 采集3帧时序图像+关节角标签"""
from __future__ import annotations
import os
import numpy as np
from .sim import TemporalE2ESim, DEFAULT_STAND_TARGET


def _downsample(img, size=16):
    h, w = img.shape
    bh, bw = h//size, w//size
    crop = img[:bh*size, :bw*size]
    return crop.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten().astype(np.float32)


def collect(model_path, out_path, n_episodes=30, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    sim = TemporalE2ESim(model_path)
    X, y = [], []
    for ep in range(n_episodes):
        sim.reset()
        for _ in range(200): sim.step()
        f = rng.uniform(60, 180)
        ang = rng.choice([0, 90, 180, 270])
        rad = np.radians(ang)
        sim.apply_push((f*np.cos(rad), f*np.sin(rad), 0), duration_steps=30)
        frame_buffer = []
        for i in range(150):
            img = sim.render_front(gray=True)
            feat = _downsample(img)
            frame_buffer.append(feat)
            if len(frame_buffer) >= 3:
                x_seq = np.concatenate(frame_buffer[-3:])
                target = DEFAULT_STAND_TARGET.copy()
                X.append(x_seq)
                y.append(target)
            sim.step()
            if sim.fallen: break
        if verbose and ep % 5 == 0:
            print(f"  [collect] ep {ep}/{n_episodes}, samples={len(X)}")
    X = np.stack(X).astype(np.float32)
    y = np.array(y, dtype=np.float32)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y)
    sim.close()
    print(f"[collect] saved {out_path}: X={X.shape}, y={y.shape}")
    return out_path
