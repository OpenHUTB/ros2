"""
dataset.py —— 端到端数据集采集
================================
随机施加外力，用专家控制器维持平衡，同时记录前视图像和专家输出的15维关节目标角。
端到端网络学习：图像 → 关节角。
"""
from __future__ import annotations
import os
import numpy as np
from .sim import E2ESim
from .expert import expert_target


def _img_to_vec(img, size=16):
    h, w = img.shape
    bh, bw = h//size, w//size
    c = img[:bh*size, :bw*size]
    return c.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten().astype(np.float32)


def collect(model_path, out_path, n_episodes=200, force_range=(60, 180),
            kp=60.0, kd=6.0, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    sim = E2ESim(model_path, kp=kp, kd=kd)
    X, Y = [], []
    for ep in range(n_episodes):
        sim.reset()
        for _ in range(250):
            sim.step()
        ang = rng.choice([0, 90, 180, 270])
        f = rng.uniform(*force_range)
        fx = f * np.cos(np.radians(ang))
        fy = f * np.sin(np.radians(ang))
        # 施加推力后，用专家控制器跑 300 步，每步记录图像+专家目标
        sim.apply_push((fx, fy, 0), steps=40)
        for _ in range(300):
            tp = sim.torso_tilt()
            tr = sim.torso_tilt_y()
            target = expert_target(tp, tr)
            X.append(_img_to_vec(sim.render_front(gray=True)))
            Y.append(target.astype(np.float32))
            sim.step(target=target)
            if sim.fallen():
                break
        if verbose and ep % 20 == 0:
            print(f"  [collect] episode {ep}/{n_episodes}, samples={len(X)}")
    X = np.stack(X)
    Y = np.stack(Y)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, Y=Y)
    sim.close()
    if verbose:
        print(f"[collect] 数据集: {out_path} | X={X.shape} Y={Y.shape}")
    return out_path
