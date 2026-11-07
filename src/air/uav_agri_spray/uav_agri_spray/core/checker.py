#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Grid deposition assessment: Gaussian superposition from adjacent scan lines.

沉积评估：田块按 1 m×1 m 栅格离散，每条扫描线段对栅格中心的横向沉积服从
高斯核（σ = 喷幅/4），多行叠加得到沉积分布图；据此统计覆盖率（沉积 ≥ 阈值
0.5 的栅格占比）、重叠率与漏喷区面积。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from .geometry import point_in_polygon, segment_point_distance
from .planner import SprayConfig, SprayPath

TAG = "[checker]"

#: 沉积判定阈值（单行峰值归一化为 1）
DEPOSIT_THRESHOLD = 0.5
#: 重叠判定阈值：叠加剂量 ≥ 1.0 视为受到不止一次有效喷洒贡献
OVERLAP_THRESHOLD = 1.0


@dataclass
class DepositMap:
    """Deposition raster over the field bounding box."""

    deposit: np.ndarray        # (ny, nx) 栅格沉积量
    x0: float
    y0: float
    resolution: float
    inside: np.ndarray         # (ny, nx) 田块内掩膜


@dataclass
class DepositMetrics:
    """Scalar deposition quality metrics."""

    coverage_ratio: float
    overlap_ratio: float
    missed_area_m2: float
    mean_deposit: float
    field_area_m2: float
    threshold: float = DEPOSIT_THRESHOLD


def build_grid(polygon: np.ndarray, resolution: float = 1.0) -> DepositMap:
    """Rasterize cell centers inside the polygon."""
    poly = np.asarray(polygon, dtype=float)
    x0, y0 = poly.min(axis=0)
    x1, y1 = poly.max(axis=0)
    nx = max(1, int(np.ceil((x1 - x0) / resolution)))
    ny = max(1, int(np.ceil((y1 - y0) / resolution)))
    xs = x0 + (np.arange(nx) + 0.5) * resolution
    ys = y0 + (np.arange(ny) + 0.5) * resolution
    xx, yy = np.meshgrid(xs, ys)
    cells = np.stack([xx.ravel(), yy.ravel()], axis=1)
    inside = np.array([point_in_polygon(p, poly) for p in cells])
    dep = np.zeros((ny, nx), dtype=float)
    ins = inside.reshape(ny, nx)
    return DepositMap(deposit=dep, x0=float(x0), y0=float(y0),
                      resolution=resolution, inside=ins)


def accumulate_deposit(grid: DepositMap, pieces, sigma: float) -> np.ndarray:
    """Superpose Gaussian across-track deposition from all sprayed pieces.

    pieces 里每项要么是 (起点, 终点) 直行段，要么是 (N, 2) 弧段采样点
    （按相邻采样点拆成短线段处理）。
    """
    dep = np.zeros_like(grid.deposit)
    res = grid.resolution
    xs = grid.x0 + (np.arange(grid.deposit.shape[1]) + 0.5) * res
    ys = grid.y0 + (np.arange(grid.deposit.shape[0]) + 0.5) * res
    xx, yy = np.meshgrid(xs, ys)
    cells = np.stack([xx.ravel(), yy.ravel()], axis=1)
    two_sig2 = 2.0 * sigma * sigma
    for piece in pieces:
        if isinstance(piece, tuple):
            segs = [piece]
        else:
            arr = np.asarray(piece, dtype=float)
            segs = list(zip(arr[:-1], arr[1:]))
        for s0, s1 in segs:
            dist = segment_point_distance(s0, s1, cells).reshape(dep.shape)
            dep += np.exp(-dist * dist / two_sig2)
    dep[~grid.inside] = 0.0
    return dep


def accumulate_deposit_points(grid: DepositMap, points: np.ndarray,
                              sigma: float, sample_spacing: float = 0.3
                              ) -> np.ndarray:
    """Deposition from a flown trajectory (weighted point sources).

    点源权重 = 采样间距 / (σ√2π)，使沿线的积分剂量与单次线源喷洒一致。
    """
    xs = grid.x0 + (np.arange(grid.deposit.shape[1]) + 0.5) * grid.resolution
    ys = grid.y0 + (np.arange(grid.deposit.shape[0]) + 0.5) * grid.resolution
    xx, yy = np.meshgrid(xs, ys)
    cells = np.stack([xx.ravel(), yy.ravel()], axis=1)
    dep = np.zeros_like(grid.deposit)
    two_sig2 = 2.0 * sigma * sigma
    w = sample_spacing / (sigma * math.sqrt(2.0 * math.pi))
    chunk = 256
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    for i in range(0, len(pts), chunk):
        d2 = ((cells[:, None, :] - pts[None, i:i + chunk, :]) ** 2).sum(axis=2)
        dep += (w * np.exp(-d2 / two_sig2)).sum(axis=1).reshape(dep.shape)
    dep[~grid.inside] = 0.0
    return dep


def evaluate_deposition(dep: np.ndarray, inside: np.ndarray,
                        resolution: float = 1.0,
                        threshold: float = DEPOSIT_THRESHOLD) -> DepositMetrics:
    """Coverage / overlap / missed-area statistics over in-field cells."""
    vals = dep[inside]
    n = int(inside.sum())
    if n == 0:
        return DepositMetrics(0.0, 0.0, 0.0, 0.0, 0.0, threshold)
    covered = vals >= threshold
    overlapped = vals >= OVERLAP_THRESHOLD
    cell_area = resolution * resolution
    return DepositMetrics(
        coverage_ratio=round(float(covered.sum()) / n, 4),
        overlap_ratio=round(float(overlapped.sum()) / n, 4),
        missed_area_m2=round(float((~covered).sum()) * cell_area, 1),
        mean_deposit=round(float(vals.mean()), 4),
        field_area_m2=round(float(n) * cell_area, 1),
        threshold=threshold)


def assess(path: SprayPath, polygon: np.ndarray,
           cfg: SprayConfig, resolution: float = 1.0) -> Tuple[DepositMap, DepositMetrics]:
    """Plan-level assessment: sprayed-piece deposition over one field polygon."""
    grid = build_grid(polygon, resolution)
    dep = accumulate_deposit(grid, path.deposit_sources(), cfg.sigma)
    grid.deposit = dep
    metrics = evaluate_deposition(dep, grid.inside, resolution)
    return grid, metrics
