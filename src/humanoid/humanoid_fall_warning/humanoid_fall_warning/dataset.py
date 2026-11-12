"""dataset.py —— 采集多模态数据（图像特征 + IMU → 未来是否跌倒）

标签口径（修正后）：
    输入是**施加扰动之后**的图像与 IMU 姿态；标签是此后 horizon 步内是否真正跌倒。
    原实现把输入取自扰动之前（机器人还稳稳站着），于是 2000 个样本的图像特征和
    IMU 完全相同，标签又因为 horizon 只有 20 步（0.04 s）而恒为 0，数据集彻底退化，
    训练出来的模型对任何输入都输出同一个概率。

采集流程：
    reset(贴地起手) → 300 步稳定 → 随机水平推力(40~520 N, 0~360°) 40 步
    → 再跑 60 步让姿态偏移显现 → 取图像(16×16 池化, 256 维) + (pitch, roll)
    → 继续仿真 horizon 步，期间跌倒则标签为 1
"""
from __future__ import annotations
import os
import numpy as np
from .sim import FallWarningSim


def _downsample(img, size=16):
    """将灰度图块平均下采样为 size×size 网格（输出 size² 维向量）。"""
    h, w = img.shape
    bh, bw = h//size, w//size
    crop = img[:bh*size, :bw*size]
    return crop.reshape(size, bh, size, bw).mean(axis=(1, 3)).flatten().astype(np.float32)


def feat_from_gray(img, size=16):
    """灰度图（0~1）→ 256 维图像特征。节点与 demo 统一走这里，避免各处复制同一段逻辑。"""
    return _downsample(np.asarray(img, dtype=np.float32), size=size)


def image_feature(sim, size=16):
    """从当前仿真状态取 256 维图像特征（供 demo / 节点复用）。"""
    return feat_from_gray(sim.render_front(gray=True), size=size)


def collect(model_path, out_path, n_samples=1500, seed=0, standing_ratio=0.2,
            settle_steps=300, observe_steps=60, push_steps=40,
            horizon=600, f_min=40.0, f_max=520.0,
            tilt_limit_deg=60.0, verbose=True):
    """采集多模态训练数据。

    Args:
        standing_ratio: 无外力纯站立负例占比，默认 20%。
        settle_steps: 起手后站定步数。
        push_steps: 推力持续时间（步）。
        observe_steps: 推力结束后再跑多少步才取输入，让倾角显现。
        horizon: 取输入后继续仿真的步数，期间跌倒即正例。0.002 s/步，600 步 = 1.2 s；
            实测 150 N 推力约 585 步倒地，窗口再短就会把正例全漏掉。
        tilt_limit_deg: 取输入时倾角超过此值则丢弃重采（已近倒地，无预测意义）。
    """
    rng = np.random.default_rng(seed)
    sim = FallWarningSim(model_path)
    X, y, pitches, rolls, forces = [], [], [], [], []
    dropped = 0
    attempts = 0
    max_attempts = n_samples * 3
    while len(y) < n_samples and attempts < max_attempts:
        attempts += 1
        standing = bool(rng.random() < standing_ratio)
        sim.reset()
        for _ in range(settle_steps):
            sim.step()
        if standing:
            force = 0.0
            for _ in range(observe_steps):
                sim.step()
        else:
            ang = rng.uniform(0.0, 360.0)
            force = float(rng.uniform(f_min, f_max))
            rad = np.radians(ang)
            sim.apply_push((force*np.cos(rad), force*np.sin(rad), 0.0),
                           duration_steps=push_steps)
            for _ in range(observe_steps):
                sim.step()

        pitch, roll = sim.torso_pose()
        if sim.fallen() or abs(np.degrees(pitch)) > tilt_limit_deg \
                or abs(np.degrees(roll)) > tilt_limit_deg:
            dropped += 1
            continue

        feat = image_feature(sim)
        X.append(np.concatenate([feat, [pitch, roll]]).astype(np.float32))
        pitches.append(float(np.degrees(pitch)))
        rolls.append(float(np.degrees(roll)))
        forces.append(force)

        fell = 0
        for _ in range(horizon):
            sim.step()
            if sim.fallen():
                fell = 1
                break
        y.append(fell)

        if verbose and len(y) % 200 == 0:
            print(f"  [collect] {len(y)}/{n_samples} | pos_rate={np.mean(y):.2f}")

    X = np.stack(X).astype(np.float32)
    y = np.array(y, dtype=np.int64)
    pitches = np.array(pitches)
    rolls = np.array(rolls)
    forces = np.array(forces)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, y=y, pitch_deg=pitches, roll_deg=rolls, force=forces)
    sim.close()
    print(f"[collect] saved {out_path}: X={X.shape}, y={y.shape}, "
          f"pos_rate={y.mean():.3f}, dropped={dropped}/{attempts}")
    print(f"[collect] 倾角覆盖: pitch ±{np.abs(pitches).max():.1f}° "
          f"(中位 {np.median(np.abs(pitches)):.1f}°, p90 {np.percentile(np.abs(pitches),90):.1f}°), "
          f"roll ±{np.abs(rolls).max():.1f}°")
    print(f"[collect] 唯一图像特征行数: {len(np.unique(X[:, :256].round(4), axis=0))}")
    return out_path
