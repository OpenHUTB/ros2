#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分割与语义导航的评测指标。

分割：逐类 IoU / mIoU、像素准确率、混淆矩阵。
导航：是否到达、路径长度、沿途平均语义不确定性、与真值障碍的碰撞数。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["confusion", "iou_per_class", "miou", "pixel_acc",
           "travelled_through", "collision_count", "path_uncertainty",
           "summarize"]


# --------------------------------------------------------------------------- #
def confusion(pred: np.ndarray, target: np.ndarray, n_class: int,
              ignore_index: int = -1) -> np.ndarray:
    """混淆矩阵，行=真值，列=预测。"""
    p = np.asarray(pred).ravel().astype(np.int64)
    t = np.asarray(target).ravel().astype(np.int64)
    keep = (t != ignore_index) & (t >= 0) & (t < n_class) \
        & (p >= 0) & (p < n_class)
    idx = t[keep] * n_class + p[keep]
    return np.bincount(idx, minlength=n_class * n_class).reshape(n_class, n_class)


def iou_per_class(conf: np.ndarray) -> np.ndarray:
    """逐类 IoU，未出现的类别返回 nan。"""
    c = np.asarray(conf, float)
    inter = np.diag(c)
    union = c.sum(axis=0) + c.sum(axis=1) - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-12), np.nan)


def miou(conf: np.ndarray) -> float:
    v = iou_per_class(conf)
    return float(np.nanmean(v)) if np.any(np.isfinite(v)) else float("nan")


def pixel_acc(conf: np.ndarray) -> float:
    c = np.asarray(conf, float)
    return float(np.diag(c).sum() / max(c.sum(), 1e-12))


def ece(conf_prob: np.ndarray, pred: np.ndarray, target: np.ndarray,
        n_bin: int = 10) -> float:
    """期望标定误差：置信度分桶后的 |准确率 − 平均置信度|。

    分割网络给出的概率如果只是相对大小而不代表真实命中率，语义代价里的
    "不确定性惩罚"就没有意义，所以这一项要单独量。
    """
    conf = np.asarray(conf_prob, float).ravel()
    corr = (np.asarray(pred).ravel() == np.asarray(target).ravel()).astype(float)
    edges = np.linspace(0.0, 1.0, n_bin + 1)
    e = 0.0
    n = len(conf)
    if n == 0:
        return float("nan")
    for i in range(n_bin):
        m = (conf > edges[i]) & (conf <= edges[i + 1])
        if not np.any(m):
            continue
        e += m.sum() / n * abs(corr[m].mean() - conf[m].mean())
    return float(e)


# --------------------------------------------------------------------------- #
def travelled_through(cost: np.ndarray, path_xy: np.ndarray, grid,
                      res: Optional[float] = None) -> float:
    """路径穿过的格子的平均语义代价（不含 inf，inf 另由碰撞数统计）。"""
    if len(path_xy) == 0:
        return float("nan")
    idx = grid.cell_of(path_xy)
    n = cost.shape[0]
    ok = np.all((idx >= 0) & (idx < n), axis=1)
    if not np.any(ok):
        return float("nan")
    v = cost[idx[ok, 1], idx[ok, 0]]
    finite = np.isfinite(v)
    return float(v[finite].mean()) if np.any(finite) else float("nan")


def collision_count(cost: np.ndarray, path_xy: np.ndarray, grid) -> int:
    """路径上落在不可通行格的采样点数。"""
    if len(path_xy) == 0:
        return 0
    idx = grid.cell_of(path_xy)
    n = cost.shape[0]
    ok = np.all((idx >= 0) & (idx < n), axis=1)
    if not np.any(ok):
        return 0
    v = cost[idx[ok, 1], idx[ok, 0]]
    return int((~np.isfinite(v)).sum())


def path_uncertainty(conf: np.ndarray, path_xy: np.ndarray, grid) -> float:
    """路径上平均的语义不确定性 ``1 − 置信度``。"""
    if len(path_xy) == 0:
        return float("nan")
    idx = grid.cell_of(path_xy)
    n = conf.shape[0]
    ok = np.all((idx >= 0) & (idx < n), axis=1)
    if not np.any(ok):
        return float("nan")
    v = 1.0 - conf[idx[ok, 1], idx[ok, 0]]
    return float(v.mean())


def summarize(rows: Sequence[Dict[str, float]]) -> Dict[str, float]:
    keys = set()
    for r in rows:
        keys |= set(r)
    out: Dict[str, float] = {}
    for k in sorted(keys):
        v = [r[k] for r in rows if k in r and np.isfinite(r[k])]
        if v:
            out[k] = float(np.mean(v))
    return out
