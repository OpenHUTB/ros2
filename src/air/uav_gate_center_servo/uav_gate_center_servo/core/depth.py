#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度图预处理：块平均降采样、有效像素比例、u8 可视化。"""
from __future__ import annotations

from typing import Tuple

import numpy as np

DEPTH_MIN = 0.2
DEPTH_MAX = 10.0
DEPTH_HW = (48, 64)
NET_HW = (12, 16)

__all__ = ["DEPTH_MIN", "DEPTH_MAX", "DEPTH_HW", "NET_HW",
           "downsample_depth", "valid_fraction", "depth_to_u8"]


def downsample_depth(depth_m: np.ndarray, out_hw: Tuple[int, int] = NET_HW) -> np.ndarray:
    """(H,W) 米制深度 → (oh,ow) 米制块平均。

    只平均有限且 >0 的像素；整块无有效像素时输出 0。兼容 48×64 与 640×480。
    """
    d = np.asarray(depth_m, dtype=np.float32)
    if d.ndim != 2:
        raise ValueError("depth must be 2-D, got %r" % (d.shape,))
    h, w = d.shape
    oh, ow = int(out_hw[0]), int(out_hw[1])
    if oh <= 0 or ow <= 0:
        raise ValueError("invalid output size %r" % (out_hw,))
    out = np.zeros((oh, ow), dtype=np.float32)
    for i in range(oh):
        r0, r1 = i * h // oh, (i + 1) * h // oh
        for j in range(ow):
            c0, c1 = j * w // ow, (j + 1) * w // ow
            block = d[r0:r1, c0:c1]
            valid = np.isfinite(block) & (block > 0.0)
            if np.any(valid):
                out[i, j] = float(block[valid].mean())
    return out


def valid_fraction(depth_m: np.ndarray) -> float:
    """有效像素比例（有限且 >0）。"""
    d = np.asarray(depth_m, dtype=np.float32)
    if d.size == 0:
        return 0.0
    return float(np.mean(np.isfinite(d) & (d > 0.0)))


def depth_to_u8(depth_m: np.ndarray) -> np.ndarray:
    """米制深度 → 0..255，仅用于可视化；无效/超量程为 0。"""
    a = np.asarray(depth_m, np.float32)
    q = np.rint((a - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * 254.0) + 1.0
    q[(a <= 1e-6) | (a < DEPTH_MIN) | (a > DEPTH_MAX) | ~np.isfinite(a)] = 0.0
    return np.clip(q, 0, 255).astype(np.uint8)
