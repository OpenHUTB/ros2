"""dataset.py —— 采集深度图 + 3D 姿态标签"""
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
            seed=0, standing_ratio=0.2, max_tilt_deg=45.0, balance=False,
            verbose=True):
    """采集训练数据：64×64 深度图下采样成 256 维，标签是躯干 (pitch, roll, yaw)。

    Args:
        standing_ratio: 无外力纯站立样本占比（负例），默认 20%。
        max_tilt_deg: 保留样本的最大躯干倾角。超过这个角度的帧躯干已经快倒出
            画面，而且 yaw 会跳过 ±90° 的万向锁奇异点，标出来的角没有意义。
        balance: 采集时是否开站姿平衡层。默认 **关**：开着平衡层时姿态被压在
            ±5° 内，实测 1500 帧的标签只覆盖 ±23.5°，模型学不到大倾角的样子。
            采集要的是覆盖，部署（demo / ROS 节点）才用平衡层。

    采样策略：先站立 0.5 s 走到站姿，再（非站立样本）沿 0–360° 连续角度施加
    随机推力，然后随机等待一段时间取帧，使倾角覆盖从微晃到接近倾倒的整个区间。
    """
    rng = np.random.default_rng(seed)
    sim = DepthPoseSim(model_path)
    X, y = [], []
    n_standing = int(n_samples * standing_ratio)
    attempts, dropped = 0, 0
    max_attempts = n_samples * 4
    while len(X) < n_samples and attempts < max_attempts:
        attempts += 1
        idx = len(X)
        sim.reset()
        for _ in range(250):                     # 0.5 s 收敛到站姿
            sim.step(balance=balance)
        if idx < n_standing:
            for _ in range(int(rng.integers(40, 160))):   # 站立微小晃动
                sim.step(balance=balance)
        else:
            ang = rng.uniform(0, 360)            # 连续角度，而非 4 个固定方向
            f = rng.uniform(force_min, force_max)
            rad = np.radians(ang)
            sim.apply_push((f*np.cos(rad), f*np.sin(rad), 0),
                           duration_steps=int(rng.integers(20, 60)))
            for _ in range(int(rng.integers(60, 400))):
                sim.step(balance=balance)
        pitch, roll, yaw = sim.torso_pose()
        tilt = float(np.degrees(max(abs(pitch), abs(roll))))
        # 已跌倒 / 倾角过大 / yaw 逼近万向锁：丢弃这一帧。
        # 不能"跌倒后 reset 再拿近直立姿态配同一张图"，那样标签和图像对不上。
        if sim.fallen() or tilt > max_tilt_deg or abs(np.degrees(yaw)) > 90.0:
            dropped += 1
            continue
        depth = sim.render_depth()
        X.append(_downsample(depth))
        y.append((pitch, roll, yaw))
        if verbose and len(X) % 200 == 0:
            print(f"  [collect] {len(X)}/{n_samples}")
    X = np.stack(X).astype(np.float32)
    y = np.array(y, dtype=np.float32)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y)
    sim.close()
    if verbose:
        print(f"[collect] saved {out_path}: X={X.shape}, y={y.shape}, "
              f"丢弃 {dropped} 帧（{attempts} 次尝试）")
    return out_path
