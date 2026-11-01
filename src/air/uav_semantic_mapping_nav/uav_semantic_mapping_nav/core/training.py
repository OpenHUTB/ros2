#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分割网络训练与推理。

采集分辨率（160×128）大于网络输入（96×96）：RGB 用面积平均下采样，标签用最近邻。
面积平均会把类别索引平均成不存在的类别（训练 loss 下降、mIoU 停在随机水平）。

类别严重不平衡（地面占了大部分像素），损失按类别频率的逆平方根加权。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .labels import N_CLASS
from .metrics import confusion, iou_per_class, miou, pixel_acc
from .nn import Adam
from .segdata import SegDataset, SegEpisode
from .segnet import SegNet, class_weights

__all__ = ["SegTrainConfig", "resize_nearest", "resize_area", "prepare_batch",
           "train_seg", "predict_episode", "evaluate"]


@dataclass
class SegTrainConfig:
    epochs: int = 12
    batch: int = 4
    lr: float = 2e-3
    weight_decay: float = 1e-6
    grad_clip: float = 1.0
    #: 64×64 是 CPU 训练速度与分割精度的折中。96×96 单轮要 6 分钟（纯 numpy 的
    #: im2col/col2im 在 96×96 上是大头），12 轮要一个多小时。
    in_hw: Tuple[int, int] = (64, 64)
    #: **按帧**留出验证集，不按 episode 留。四个采集 episode 分属三个场景，
    #: 留一集时验证集与训练集分布不同：实测验证损失上升、训练损失下降（域偏移）。
    val_frac: float = 0.15
    #: 每集最多取多少帧。采集时相邻帧只差 0.2 s、高度相关，全取是浪费。
    max_per_episode: int = 150
    weight_power: float = 0.5
    seed: int = 0


# --------------------------------------------------------------------------- #
def resize_nearest(x: np.ndarray, hw: Tuple[int, int]) -> np.ndarray:
    """最近邻缩放。``x`` 为 (H,W) 或 (H,W,C)。"""
    h, w = x.shape[:2]
    oh, ow = hw
    ih = (np.arange(oh) * h) // oh
    iw = (np.arange(ow) * w) // ow
    return x[ih][:, iw]


def _box(x: np.ndarray, n_out: int, axis: int) -> np.ndarray:
    """沿 ``axis`` 做分箱平均。箱子边界按 ``i*n//n_out`` 取，比例任意都成立。"""
    n = x.shape[axis]
    if n == n_out:
        return x
    if n_out > n:
        # 放大时箱子比源像素还窄，分箱平均无定义（箱内计数会取到 0，除零出 NaN）。
        # 退回最近邻：全卷积网络换分辨率推理走的就是这条路。
        return np.take(x, (np.arange(n_out) * n) // n_out, axis=axis)
    idx = (np.arange(n_out) * n) // n_out
    cnt = np.diff(np.append(idx, n)).astype(np.float32)
    s = np.add.reduceat(x, idx, axis=axis)
    shape = [1] * x.ndim
    shape[axis] = n_out
    return s / cnt.reshape(shape)


def resize_area(x: np.ndarray, hw: Tuple[int, int]) -> np.ndarray:
    """面积平均缩放，只用于 RGB。

    源尺寸非整数倍时退回"裁剪"会错位：标签贯穿全图最近邻采样，输入只取
    左上角（实测 128×160 → 96×96 时 RGB 裁掉 44% 画面、标签未裁，mIoU 崩）。
    这里统一按分箱平均做。
    """
    h, w = x.shape[:2]
    oh, ow = hw
    if (h, w) == (oh, ow):
        return np.asarray(x, np.float32)
    a = np.asarray(x, np.float32)
    return _box(_box(a, oh, 0), ow, 1)


def prepare_batch(ep: SegEpisode, idx: Sequence[int], hw: Tuple[int, int]
                  ) -> Tuple[np.ndarray, np.ndarray]:
    """取若干帧，返回 (rgb float (N,H,W,3), seg int32 (N,H,W))。"""
    rgb = np.stack([resize_area(ep.rgb[i], hw) for i in idx]).astype(np.float32)
    rgb = np.clip(rgb, 0.0, 255.0) / 255.0
    seg = np.stack([resize_nearest(ep.seg[i], hw) for i in idx]).astype(np.int32)
    return rgb, seg


# --------------------------------------------------------------------------- #
def _class_counts(ds: SegDataset, idx: Sequence[int], hw, max_per_ep: int) -> np.ndarray:
    cnt = np.zeros(N_CLASS, np.int64)
    for i in idx:
        ep = ds.episodes[i]
        n = len(ep) if max_per_ep <= 0 else min(len(ep), max_per_ep)
        for k in range(n):
            s = resize_nearest(ep.seg[k], hw)
            cnt += np.bincount(s.ravel(), minlength=N_CLASS)
    return cnt


def train_seg(ds: SegDataset, cfg: Optional[SegTrainConfig] = None,
              net: Optional[SegNet] = None, verbose: bool = True
              ) -> Tuple[SegNet, Dict[str, List[float]]]:
    cfg = cfg or SegTrainConfig()
    net = net or SegNet()
    net.cfg.in_hw = tuple(cfg.in_hw)

    all_samples: List[Tuple[int, int]] = []
    for i, ep in enumerate(ds.episodes):
        n = len(ep) if cfg.max_per_episode <= 0 else min(len(ep), cfg.max_per_episode)
        all_samples += [(i, k) for k in range(n)]
    rng = np.random.default_rng(cfg.seed)
    perm = rng.permutation(len(all_samples))
    n_val = max(int(len(all_samples) * cfg.val_frac), 1)
    va_idx = [all_samples[i] for i in perm[:n_val]]
    samples = [all_samples[i] for i in perm[n_val:]]

    counts = _class_counts_pairs(ds, samples, cfg.in_hw)
    wgt = class_weights(counts, cfg.weight_power)
    opt = Adam(net.layers, lr=cfg.lr, weight_decay=cfg.weight_decay,
               clip=cfg.grad_clip)

    hist: Dict[str, List[float]] = {"train": [], "val_loss": [], "val_miou": []}
    for ep_i in range(cfg.epochs):
        rng.shuffle(samples)
        tot, nb = 0.0, 0
        for b in range(0, len(samples), cfg.batch):
            grp = samples[b:b + cfg.batch]
            if len(grp) < 2:
                continue
            rgb = np.stack([resize_area(ds.episodes[i].rgb[k], cfg.in_hw)
                            for i, k in grp]).astype(np.float32)
            rgb = np.clip(rgb, 0.0, 255.0) / 255.0
            seg = np.stack([resize_nearest(ds.episodes[i].seg[k], cfg.in_hw)
                            for i, k in grp]).astype(np.int32)
            net.zero_grad()
            loss, _ = net.loss_and_grad(rgb, seg, weight=wgt)
            opt.step()
            tot += loss
            nb += 1
        hist["train"].append(tot / max(nb, 1))
        m = evaluate_pairs(net, ds, va_idx, cfg.in_hw, wgt, max_frames=48)
        hist["val_loss"].append(m["loss"])
        hist["val_miou"].append(m["miou"])
        if verbose and (ep_i % 5 == 0 or ep_i == cfg.epochs - 1):
            print("  epoch %2d  训练损失 %.4f  验证损失 %.4f  验证 mIoU %.4f"
                  % (ep_i, hist["train"][-1], m["loss"], m["miou"]))
    hist["best_miou"] = [float(max(hist["val_miou"]))] if hist["val_miou"] else [0.0]
    return net, hist


def _class_counts_pairs(ds, pairs, hw) -> np.ndarray:
    cnt = np.zeros(N_CLASS, np.int64)
    for i, k in pairs:
        s = resize_nearest(ds.episodes[i].seg[k], hw)
        cnt += np.bincount(s.ravel(), minlength=N_CLASS)
    return cnt


def evaluate_pairs(net: SegNet, ds, pairs, hw, wgt=None, max_frames: int = 0
                   ) -> Dict[str, float]:
    """在 (集号, 帧号) 列表上算指标。"""
    from .nn import softmax
    if max_frames > 0 and len(pairs) > max_frames:
        step = max(len(pairs) // max_frames, 1)
        pairs = pairs[::step][:max_frames]
    conf = np.zeros((N_CLASS, N_CLASS), np.int64)
    loss, nb = 0.0, 0
    for i, k in pairs:
        rgb = np.clip(resize_area(ds.episodes[i].rgb[k], hw), 0, 255)[None] / 255.0
        seg = resize_nearest(ds.episodes[i].seg[k], hw)[None].astype(np.int32)
        lg = net.forward(rgb)
        p = softmax(lg, axis=1)
        if wgt is None:
            loss += float(-np.mean(np.log(np.maximum(
                p[np.arange(1)[:, None, None], seg], 1e-12))))
        else:
            w = np.asarray(wgt)[seg]
            loss += float(-np.mean(w * np.log(np.maximum(
                p[np.arange(1)[:, None, None], seg], 1e-12))))
        nb += 1
        conf += confusion(np.argmax(lg, axis=1), seg, N_CLASS)
    return {"loss": loss / max(nb, 1), "miou": miou(conf),
            "pixel_acc": pixel_acc(conf), "conf": conf,
            "iou_per_class": iou_per_class(conf)}


# --------------------------------------------------------------------------- #
def predict_episode(net: SegNet, ep: SegEpisode, in_hw: Tuple[int, int] = (96, 96)
                    ) -> Dict[str, np.ndarray]:
    """逐帧预测，结果缩放回原始分辨率。返回 ``seg`` 与 ``conf``。"""
    out, conf = [], []
    for k in range(len(ep)):
        rgb = np.clip(resize_area(ep.rgb[k], in_hw), 0, 255) / 255.0
        p = net.predict_proba(rgb[None])[0]
        lab = np.argmax(p, axis=0).astype(np.int32)
        c = p.max(axis=0)
        out.append(resize_nearest(lab, ep.rgb.shape[1:3]))
        conf.append(resize_nearest(c.astype(np.float32), ep.rgb.shape[1:3]))
    return {"seg": np.asarray(out), "conf": np.asarray(conf)}


def evaluate(net: SegNet, ds: SegDataset, idx: Sequence[int],
             cfg: Optional[SegTrainConfig] = None, max_frames: int = 0
             ) -> Dict[str, float]:
    """在若干集上算混淆矩阵、mIoU、像素准确率与损失。"""
    cfg = cfg or SegTrainConfig()
    conf = np.zeros((N_CLASS, N_CLASS), np.int64)
    loss, nb = 0.0, 0
    counts = _class_counts(ds, list(idx), cfg.in_hw, 8)
    wgt = class_weights(counts, cfg.weight_power)
    for i in idx:
        ep = ds.episodes[i]
        n = len(ep) if max_frames <= 0 else min(len(ep), max_frames)
        for k in range(n):
            rgb = np.clip(resize_area(ep.rgb[k], cfg.in_hw), 0, 255)[None] / 255.0
            seg = resize_nearest(ep.seg[k], cfg.in_hw)[None].astype(np.int32)
            lg = net.forward(rgb)
            from .nn import softmax
            p = softmax(lg, axis=1)
            loss += float(-np.mean(np.log(np.maximum(
                p[np.arange(1)[:, None, None], seg], 1e-12))))
            nb += 1
            conf += confusion(np.argmax(lg, axis=1), seg, N_CLASS)
    return {"loss": loss / max(nb, 1), "miou": miou(conf),
            "pixel_acc": pixel_acc(conf), "conf": conf,
            "iou_per_class": iou_per_class(conf)}
