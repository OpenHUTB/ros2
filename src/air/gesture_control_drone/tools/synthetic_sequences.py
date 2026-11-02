#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""合成手势序列 —— 让训练/评测**不需要摄像头**也能跑通。

用途
----
1. 单元测试：验证序列模型、缓冲、闸门的接口与行为；
2. 冷启动：真实数据还没攒够时，先跑通"训练 -> 导出 -> 推理"整条链路；
3. 回归：模型结构改动后，用它检查准确率是否崩掉。

重要说明
--------
合成数据**不能代替真实采集**。它的手势是"从任意起始姿态平滑插值到目标姿态"，
只保留了最粗的时序结构（正在张开 vs 正在收拢）；真实手部还有遮挡、抖动、
光照变化。文档里必须写清"合成数据只用来验证流程，最终指标来自真实数据"。

关键点布局与 MediaPipe 一致（21 点，图像坐标，y 向下）。
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

#: 基础手型：手指朝上（图像 y 轴向下，所以指尖 y 为负）
_BASE = np.array([
    [0, 0],                                        # 0 腕
    [-10, 5], [-16, 0], [-20, -5], [-24, -10],     # 1-4  拇指
    [-8, -15], [-9, -30], [-9, -38], [-10, -46],   # 5-8  食指
    [0, -15], [0, -33], [0, -43], [0, -52],        # 9-12 中指
    [8, -15], [8, -30], [8, -38], [8, -46],        # 13-16 无名指
    [15, -13], [15, -25], [15, -32], [15, -38],    # 17-20 小指
], dtype=np.float32)

#: 每个手势"收起"哪些手指（指尖被拉回掌根附近）
_FOLD = {
    "PALM_OPEN": (),
    "FIST": ((8, 5), (12, 9), (16, 13), (20, 17)),
    "POINT": ((12, 9), (16, 13), (20, 17)),
    "VICTORY": ((16, 13), (20, 17)),
    "THUMB_UP": ((8, 5), (12, 9), (16, 13), (20, 17)),
    "THUMB_DOWN": ((8, 5), (12, 9), (16, 13), (20, 17)),
    "OK": ((16, 13), (20, 17)),
}

#: 生成序列时用到的"手势"（与 sequence_model.SEQUENCE_CLASSES 前 7 项一致）
GESTURES = ("PALM_OPEN", "FIST", "POINT", "VICTORY",
            "THUMB_UP", "THUMB_DOWN", "OK")


def canonical_pose(gesture: str) -> np.ndarray:
    """给出手势的"标准手型"（21×2）。"""
    if gesture not in _FOLD:
        raise ValueError("未知手势：%r" % gesture)
    pose = _BASE.copy()
    for tip, mcp in _FOLD[gesture]:
        pose[tip] = pose[mcp] + np.array([0.0, 12.0], dtype=np.float32)
    if gesture in ("THUMB_UP", "THUMB_DOWN", "FIST"):
        pose[4] = pose[3] + np.array([-4.0, 6.0], dtype=np.float32)
    if gesture == "THUMB_UP":
        pose[2] = pose[1] + np.array([-3.0, -12.0], dtype=np.float32)
        pose[3] = pose[2] + np.array([-3.0, -14.0], dtype=np.float32)
        pose[4] = pose[3] + np.array([-2.0, -14.0], dtype=np.float32)
    if gesture == "THUMB_DOWN":
        pose[2] = pose[1] + np.array([-3.0, 12.0], dtype=np.float32)
        pose[3] = pose[2] + np.array([-3.0, 14.0], dtype=np.float32)
        pose[4] = pose[3] + np.array([-2.0, 14.0], dtype=np.float32)
    if gesture == "OK":
        pose[4] = pose[8] + np.array([2.0, 2.0], dtype=np.float32)
        pose[3] = pose[4] + np.array([6.0, 8.0], dtype=np.float32)
    return pose


def _random_pose(rng: np.random.RandomState) -> np.ndarray:
    """随机手型：在某个手势附近加扰动，当作"起始姿态"。"""
    g = GESTURES[int(rng.randint(len(GESTURES)))]
    pose = canonical_pose(g)
    pose += rng.normal(0.0, 3.0, pose.shape)
    scale = 0.7 + 0.6 * rng.rand()
    pose *= scale
    pose += rng.normal(0.0, 20.0, (1, 2))          # 平移到画面任意位置
    return pose.astype(np.float32)


def make_sequence(gesture: str, length: int = 16, seed: int = 0,
                  start: Optional[np.ndarray] = None,
                  noise: float = 1.2) -> np.ndarray:
    """生成一条 ``(length, 21, 2)`` 的关键点序列：从起始姿态平滑过渡到目标手势。

    标签语义：**序列结束时的姿态**就是 ``gesture``。
    """
    rng = np.random.RandomState(seed)
    target = canonical_pose(gesture)
    if start is None:
        start = _random_pose(rng)
    start = np.asarray(start, dtype=np.float32)
    if start.shape != target.shape:
        raise ValueError("start 形状应为 %s，收到 %s" % (target.shape, start.shape))
    if length < 2:
        raise ValueError("length 至少为 2")

    out = np.empty((length, 21, 2), dtype=np.float32)
    for t in range(length):
        # 缓动插值（smoothstep），让"运动方向"信息更明显
        a = t / float(length - 1)
        a = a * a * (3.0 - 2.0 * a)
        pose = (1.0 - a) * start + a * target
        pose = pose + rng.normal(0.0, noise, pose.shape)
        out[t] = pose
    return out


def make_no_hand_sequence(length: int = 16) -> np.ndarray:
    """整条序列都没有手（关键点全 0）。"""
    return np.zeros((length, 21, 2), dtype=np.float32)


def make_dataset(samples_per_class: int = 60, length: int = 16, seed: int = 0,
                 include_none: bool = True,
                 none_ratio: float = 0.25) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """生成数据集：``(x (N, T, 21, 2), y (N,), classes)``。

    默认 7 类手势各 ``samples_per_class`` 条，外加约 ``none_ratio`` 比例的
    "没有手"样本（标签为 ``NONE``，即下标 7）。
    """
    rng = np.random.RandomState(seed)
    xs: List[np.ndarray] = []
    ys: List[int] = []
    for ci, g in enumerate(GESTURES):
        for k in range(samples_per_class):
            xs.append(make_sequence(g, length=length, seed=int(rng.randint(1 << 30))))
            ys.append(ci)
    if include_none:
        n_none = max(1, int(samples_per_class * len(GESTURES) * none_ratio))
        for _ in range(n_none):
            xs.append(make_no_hand_sequence(length))
            ys.append(7)
    return np.stack(xs).astype(np.float32), np.array(ys, dtype=np.int64), list(GESTURES) + ["NONE"]


def to_features(sequences: np.ndarray) -> np.ndarray:
    """``(N, T, 21, 2)`` -> ``(N, T, 42)`` 归一化特征。

    优先用 ``sequence_model.normalize_landmarks``（它可能复用 ``gesture_dataset``）。
    """
    import os
    import sys
    scripts = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import sequence_model as sm

    seqs = np.asarray(sequences, dtype=np.float32)
    if seqs.ndim != 4 or seqs.shape[2:] != (21, 2):
        raise ValueError("输入应为 (N, T, 21, 2)，收到 %s" % (seqs.shape,))
    N, T = seqs.shape[0], seqs.shape[1]
    out = np.empty((N, T, sm.FEATURE_DIM), dtype=np.float32)
    for i in range(N):
        for t in range(T):
            out[i, t] = sm.normalize_landmarks(seqs[i, t])
    return out


def mask_of(sequences: np.ndarray) -> np.ndarray:
    """由序列推出 mask：关键点全 0 的帧视为"没有手"。"""
    seqs = np.asarray(sequences, dtype=np.float32)
    valid = (np.abs(seqs).sum(axis=(2, 3)) > 1e-6).astype(np.float32)
    return valid


if __name__ == "__main__":                          # pragma: no cover
    import sys
    x, y, classes = make_dataset(samples_per_class=int(sys.argv[1]) if len(sys.argv) > 1 else 10)
    print("数据集:", x.shape, "标签:", y.shape, "类别:", classes)
    print("类别分布:", {c: int((y == i).sum()) for i, c in enumerate(classes)})
