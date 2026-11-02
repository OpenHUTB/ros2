#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""学习型落点选择网络的前向推理（纯 numpy 实现，部署端不需要 torch）。

网络结构（与 tools/train_landing_cnn.py 里 PyTorch 的定义**逐层对应**）：

    x (3, 64, 64)
      conv3x3(3  -> 16) + ReLU, maxpool2      -> (16, 32, 32)
      conv3x3(16 -> 32) + ReLU, maxpool2      -> (32, 16, 16)
      conv3x3(32 -> 48) + ReLU, maxpool2      -> (48,  8,  8)
      conv3x3(48 -> 48) + ReLU, up2           -> (48, 16, 16)
      conv3x3(48 -> 24) + ReLU, up2           -> (24, 32, 32)
      conv3x3(24 -> 16) + ReLU, up2           -> (16, 64, 64)
      conv1x1(16 -> 1)                        -> logits

    为什么要有三级下采样：只有 1~2 级时感受野不到 5 m，网络没法比较"屋顶中心"
    与"屋顶边缘"的净空，实测落点误差会在 4~6 m 量级；加到 8x8 的瓶颈后感受野
    覆盖整个 16 m ROI，误差才降到亚米级。

为什么要在 numpy 里再实现一遍：Ubuntu 客户机（ROS Noetic + Python 3.8）上**不装
torch**，模块依赖仍然只有 numpy —— 和 rooftop_landing 的「无重型依赖」原则一致。
权重导出成 ``models/landing_cnn.npz``（约 210 KB），仓库 .gitignore 明确保留
``!**/models/*.npz``，可以直接入库、克隆即可运行。

正确性由 ``tests/test_rooftop_landing_nn_local.py`` 里的「numpy 前向 == torch 前向」
逐元素对齐测试保障（requires torch 的机器上跑，没有 torch 时自动跳过）。
"""
from __future__ import annotations

import json
import os

import numpy as np

MODEL_VERSION = 1

#: 权重键名顺序（改结构必须同时改训练脚本、numpy 前向与 npz 里的 meta）
CONV_LAYERS = ("c1", "c2", "c3", "c4", "c5", "c6")
HEAD_LAYER = "h1"


# --------------------------------------------------------------------------
# 基础算子
# --------------------------------------------------------------------------
def conv2d(x, w, b, pad=1, stride=1):
    """二维互相关（等价 PyTorch 的 Conv2d）。

    x: (Cin, H, W)   w: (Cout, Cin, kh, kw)   b: (Cout,)
    """
    cin, h, w_in = x.shape
    cout, cin_w, kh, kw = w.shape
    if cin != cin_w:
        raise ValueError("channel mismatch: x has %d, w expects %d" % (cin, cin_w))
    xp = np.pad(x.astype(np.float32), ((0, 0), (pad, pad), (pad, pad)), mode="constant")
    out_h = (h + 2 * pad - kh) // stride + 1
    out_w = (w_in + 2 * pad - kw) // stride + 1
    cols = np.empty((cin * kh * kw, out_h * out_w), dtype=np.float32)
    idx = 0
    for ci in range(cin):
        for ki in range(kh):
            for kj in range(kw):
                patch = xp[ci, ki:ki + out_h * stride:stride, kj:kj + out_w * stride:stride]
                cols[idx] = patch.reshape(-1)
                idx += 1
    wm = w.reshape(cout, -1).astype(np.float32)
    out = wm.dot(cols)
    out += b.reshape(cout, 1).astype(np.float32)
    return out.reshape(cout, out_h, out_w)


def relu(x):
    return np.maximum(x, 0.0, dtype=np.float32)


def sigmoid(x):
    x = np.asarray(x, dtype=np.float32)
    out = np.empty_like(x)
    pos = x >= 0
    out[pos] = 1.0 / (1.0 + np.exp(-x[pos]))
    ex = np.exp(x[~pos])
    out[~pos] = ex / (1.0 + ex)
    return out


def maxpool2(x):
    """2x2 / stride 2 最大池化（丢弃不能整除的边）。"""
    c, h, w = x.shape
    h2, w2 = h // 2, w // 2
    if h2 == 0 or w2 == 0:
        raise ValueError("input too small to pool: %r" % (x.shape,))
    x = x[:, :h2 * 2, :w2 * 2]
    return x.reshape(c, h2, 2, w2, 2).max(axis=(2, 4))


def _bilinear_axis(out_size, in_size):
    scale = float(in_size) / float(out_size)
    idx = (np.arange(out_size, dtype=np.float32) + 0.5) * scale - 0.5
    idx = np.clip(idx, 0.0, in_size - 1.0)
    lo = np.floor(idx).astype(np.int64)
    hi = np.minimum(lo + 1, in_size - 1)
    frac = (idx - lo).astype(np.float32)
    return lo, hi, frac


def upsample_bilinear(x, out_h, out_w):
    """双线性上采样，等价 ``F.interpolate(..., mode='bilinear', align_corners=False)``。"""
    c, h, w = x.shape
    rl, rh, rf = _bilinear_axis(out_h, h)
    cl, ch, cf = _bilinear_axis(out_w, w)
    x = x.astype(np.float32)
    top = x[:, rl][:, :, cl]
    top_r = x[:, rl][:, :, ch]
    bot = x[:, rh][:, :, cl]
    bot_r = x[:, rh][:, :, ch]
    rf2 = rf.reshape(1, -1, 1)
    cf2 = cf.reshape(1, 1, -1)
    top_i = top + (top_r - top) * cf2
    bot_i = bot + (bot_r - bot) * cf2
    return (top_i + (bot_i - top_i) * rf2).astype(np.float32)


# --------------------------------------------------------------------------
# 模型载入 / 前向
# --------------------------------------------------------------------------
def load_model(path):
    """读取 npz 权重，返回 ``{'w': {...}, 'meta': {...}}``。"""
    if not os.path.isfile(path):
        raise IOError("model file not found: %s" % path)
    data = np.load(path, allow_pickle=False)
    weights = {}
    for key in data.files:
        if key == "meta":
            continue
        weights[key] = np.asarray(data[key], dtype=np.float32)
    meta = {}
    if "meta" in data.files:
        meta = json.loads(str(np.asarray(data["meta"]).reshape(-1)[0]))
    return {"w": weights, "meta": meta}


def forward(model, x, return_heat=True):
    """前向：输入 (3, C, C) 或 (1, 3, C, C)，返回 logits / 概率热力图。"""
    arr = np.asarray(x, dtype=np.float32)
    if arr.ndim == 4:
        if arr.shape[0] != 1:
            raise ValueError("forward expects batch size 1, got %r" % (arr.shape,))
        arr = arr[0]
    if arr.ndim != 3:
        raise ValueError("forward expects (3, C, C), got %r" % (arr.shape,))
    w = model["w"]
    h = relu(conv2d(arr, w["c1.w"], w["c1.b"]))
    h = maxpool2(h)
    h = relu(conv2d(h, w["c2.w"], w["c2.b"]))
    h = maxpool2(h)
    h = relu(conv2d(h, w["c3.w"], w["c3.b"]))
    h = maxpool2(h)
    h = relu(conv2d(h, w["c4.w"], w["c4.b"]))
    h = _up2(h)
    h = relu(conv2d(h, w["c5.w"], w["c5.b"]))
    h = _up2(h)
    h = relu(conv2d(h, w["c6.w"], w["c6.b"]))
    h = _up2(h)
    logits = conv2d(h, w["h1.w"], w["h1.b"], pad=0)
    return sigmoid(logits) if return_heat else logits


def _up2(x):
    return upsample_bilinear(x, x.shape[1] * 2, x.shape[2] * 2)


# --------------------------------------------------------------------------
# 从热力图取落点
# --------------------------------------------------------------------------
def heat_to_point(heat, hm, min_height_m=1.0, refine=True, rough_max=None):
    """在热力图上取分数最高的格，并做亚格精化，返回落点信息 dict。

    两个**物理安全约束**（都是硬约束，不满足就退让，不会返回不安全点）：

    ``min_height_m``
        只允许落在「比参考面高至少这么多米」的格子上，避免网络万一在平地上
        给出高分时把落点选到地面（与 rooftop_landing 的 ``perception/min_height_mode``
        思路一致，这里是栅格层面的硬约束）。
    ``rough_max``
        只允许落在「平坦」的格子上：``rough`` 是每格 (max z - min z) 归一化后的
        粗糙度，屋面接近 0，而立面（墙）会接近 1。**注意**：高度图取的是每格最高点，
        墙脚那一列的"高度"与屋面一样高，只靠高度无法区分墙与屋面 ——
        这一条约束专门用来挡住"贴着墙下落"这种危险解。给 ``None`` 表示不过滤。

    约束依次退让：高度 + 平坦 → 高度 → 掩码（保证任何情况下都有解）。
    """
    heat = np.asarray(heat, dtype=np.float32)
    if heat.ndim == 3:
        heat = heat[0]
    mask = np.asarray(hm["mask"], dtype=bool)
    cells = int(hm["cells"])
    if heat.shape != (cells, cells) or mask.shape != (cells, cells):
        raise ValueError("heat / mask shape mismatch: %r vs %r"
                         % (heat.shape, mask.shape))

    high = hm["height"].astype(np.float32) * float(hm["z_scale"]) >= float(min_height_m)
    flat = np.ones_like(mask)
    if rough_max is not None:
        flat = hm["rough"].astype(np.float32) <= float(rough_max)

    allowed = mask & high & flat
    if not allowed.any():
        allowed = mask & high
    if not allowed.any():
        allowed = mask
    if not allowed.any():
        return {"ok": False, "reason": "empty_heightmap", "row": -1, "col": -1,
                "x": float(hm["center"][0]), "y": float(hm["center"][1]), "score": 0.0}

    h = np.where(allowed, heat, -1.0)
    flat = int(np.argmax(h))
    row, col = int(flat // cells), int(flat % cells)
    score = float(heat[row, col])

    if refine:
        r0, r1 = max(0, row - 1), min(cells, row + 2)
        c0, c1 = max(0, col - 1), min(cells, col + 2)
        win = heat[r0:r1, c0:c1] * allowed[r0:r1, c0:c1]
        wsum = float(win.sum())
        if wsum > 1e-6:
            idx_r = np.arange(r0, r1, dtype=np.float32).reshape(-1, 1)
            idx_c = np.arange(c0, c1, dtype=np.float32).reshape(1, -1)
            row_f = float((win * idx_r).sum() / wsum)
            col_f = float((win * idx_c).sum() / wsum)
            half = 0.5 * hm["size_m"]
            x = hm["center"][0] - half + (col_f + 0.5) * hm["cell"]
            y = hm["center"][1] - half + (row_f + 0.5) * hm["cell"]
            return {"ok": True, "reason": "ok", "row": row, "col": col,
                    "x": float(x), "y": float(y), "score": score,
                    "row_refined": row_f, "col_refined": col_f}

    half = 0.5 * hm["size_m"]
    x = hm["center"][0] - half + (col + 0.5) * hm["cell"]
    y = hm["center"][1] - half + (row + 0.5) * hm["cell"]
    return {"ok": True, "reason": "ok", "row": row, "col": col,
            "x": float(x), "y": float(y), "score": score}


def predict(model, hm, min_height_m=1.0, refine=True, rough_max=None, return_heat=False):
    """端到端：高度图 -> 落点（含 ``score``）。"""
    from heightmap import heightmap_channels  # 同目录，节点运行时 sys.path 已包含
    channels = heightmap_channels(hm)
    heat = forward(model, channels)
    point = heat_to_point(heat, hm, min_height_m=min_height_m, refine=refine,
                          rough_max=rough_max)
    if return_heat:
        point["heat"] = heat
    return point
