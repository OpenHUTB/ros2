#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多站交会：把若干条视线投影到地面做加权最小二乘，并给出误差统计。

单站定位的误差 ≈ 高度 × 角误差，随距离线性增长；两站交会能把同一个
角误差变成两个不同方向的分量，用它们的正交性摊薄。代价是交会角变小
时摊薄失效——两条近乎平行的线，交点会沿"线方向"跑掉。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from .geometry import Ray, as_vec3

EPS = 1e-9


@dataclass
class FusionResult:
    """一次交会的解与质量指标。"""

    point: np.ndarray                 # (2,) 地面上 (x, y) 估计
    cov: np.ndarray                   # (2, 2) 估计协方差（由权重推得）
    residual_m: float                 # 各射线到估计点的加权残差 RMS
    n_rays: int
    condition: float                  # 法方程条件数，越大越病态
    ok: bool = True
    reason: str = ""

    def error_to(self, truth: Sequence[float]) -> float:
        t = as_vec3([truth[0], truth[1], 0.0] if len(truth) == 2 else truth, "truth")
        return float(math.hypot(self.point[0] - t[0], self.point[1] - t[1]))


def _plane_ray(ray: Ray, ground_z: float):
    """把三维视线压到地面平面：返回 (交点2, 单位方向2)。"""
    hit = ray.ground_hit(ground_z)
    d = np.array([float(ray.direction[0]), float(ray.direction[1])])
    n = float(np.linalg.norm(d))
    if n < 1e-6:
        raise ValueError("视线水平分量为零，投影到地面后退化")
    return hit[:2], d / n


def fuse(rays: Sequence[Ray], ground_z: float = 0.0,
         weights: Optional[Sequence[float]] = None) -> FusionResult:
    """加权最小二乘求地面交点：min Σ wᵢ·dist(X, lineᵢ)²。

    dist² = (X-a)ᵀ(I - uuᵀ)(X-a)，所以法方程是
    A = Σ wᵢ(I - uᵢuᵢᵀ)，b = Σ wᵢ(I - uᵢuᵢᵀ)aᵢ，解 A X = b。
    """
    if not rays:
        raise ValueError("至少需要一条视线")
    n = len(rays)
    w = np.ones(n) if weights is None else np.asarray(weights, dtype=float).reshape(-1)
    if w.shape != (n,):
        raise ValueError(f"权重长度 {w.shape} 与视线数 {n} 不一致")
    if np.any(~np.isfinite(w)) or np.any(w <= 0.0):
        raise ValueError("权重必须是有限正数")

    A = np.zeros((2, 2))
    b = np.zeros(2)
    for wi, ray in zip(w, rays):
        a, u = _plane_ray(ray, ground_z)
        P = np.eye(2) - np.outer(u, u)
        A += wi * P
        b += wi * (P @ a)

    cond = float(np.linalg.cond(A)) if n >= 1 else float("inf")
    if cond > 1e12:
        # 所有视线平行：法方程奇异，只能说"这条线上某个点"，不能给唯一解
        point = b / max(np.trace(A), EPS) if np.trace(A) > EPS else np.zeros(2)
        return FusionResult(point=point, cov=np.full((2, 2), np.inf),
                            residual_m=float("inf"), n_rays=n, condition=cond,
                            ok=False, reason="视线共线，法方程奇异（交会角≈0）")

    point = np.linalg.solve(A, b)
    resid = []
    for wi, ray in zip(w, rays):
        a, u = _plane_ray(ray, ground_z)
        d = point - a
        perp = d - float(np.dot(d, u)) * u
        resid.append(float(np.dot(perp, perp)))
    dof = max(1, 2 * n - 2)
    s2 = float(sum(r * wi for r, wi in zip(resid, w)) / dof)
    cov = s2 * np.linalg.inv(A) if n > 1 else np.full((2, 2), np.nan)
    return FusionResult(point=point, cov=cov,
                        residual_m=float(math.sqrt(max(0.0, s2))),
                        n_rays=n, condition=cond, ok=True)


def fused_from_directions(positions: Sequence[Sequence[float]],
                          directions: Sequence[Sequence[float]],
                          ground_z: float = 0.0) -> FusionResult:
    """用 (位置, 方向) 两串数组直接交会。"""
    if len(positions) != len(directions):
        raise ValueError("位置与方向数量不一致")
    rays = [Ray(origin=p, direction=d) for p, d in zip(positions, directions)]
    return fuse(rays, ground_z=ground_z)


def mean_direction(directions: Sequence[Sequence[float]]) -> np.ndarray:
    """一簇视线的平均方向（先归一化再平均，再归一化回单位长度）。"""
    arr = np.asarray(directions, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"方向数组必须是 (N,3)，收到 {arr.shape}")
    if len(arr) == 0:
        raise ValueError("没有可平均的方向")
    if not np.all(np.isfinite(arr)):
        raise ValueError("方向数组含非有限值")
    n = np.linalg.norm(arr, axis=1, keepdims=True)
    if np.any(n < 1e-9):
        raise ValueError("存在零方向向量")
    m = (arr / n).mean(axis=0)
    nm = float(np.linalg.norm(m))
    if nm < 1e-9:
        raise ValueError("方向互相抵消，平均方向无定义")
    return m / nm


def angular_spread_deg(directions: Sequence[Sequence[float]]) -> float:
    """一簇视线的角散布（相对平均方向的最大偏差，度）。"""
    m = mean_direction(directions)
    arr = np.asarray(directions, dtype=float)
    arr = arr / np.linalg.norm(arr, axis=1, keepdims=True)
    cos = np.clip(arr @ m, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos)).max())


def radial_errors(points: Sequence[Sequence[float]],
                  truth: Sequence[float]) -> np.ndarray:
    """一组估计点到真值的水平距离（m）。"""
    t = np.asarray(truth, dtype=float).reshape(-1)
    if t.shape[0] < 2:
        raise ValueError("真值至少要有 x, y")
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 2:
        raise ValueError(f"估计点数组形状不对：{arr.shape}")
    if len(arr) == 0:
        return np.zeros(0)
    return np.hypot(arr[:, 0] - t[0], arr[:, 1] - t[1])


def cep(errors: Sequence[float], percentile: float = 0.95) -> float:
    """圆概率误差：给定分位数下的径向误差（m）。"""
    e = np.asarray(errors, dtype=float).reshape(-1)
    if e.size == 0:
        raise ValueError("误差样本为空")
    if np.any(~np.isfinite(e)):
        raise ValueError("误差样本含非有限值")
    return float(np.percentile(e, 100.0 * float(percentile)))


def summarize(errors: Sequence[float]) -> dict:
    """均值 / 中位数 / 最大 / CEP50 / CEP95 一次算齐。"""
    e = np.asarray(errors, dtype=float).reshape(-1)
    if e.size == 0:
        raise ValueError("误差样本为空")
    return {
        "n": int(e.size),
        "mean_m": float(e.mean()),
        "median_m": float(np.median(e)),
        "max_m": float(e.max()),
        "cep50_m": cep(e, 0.50),
        "cep95_m": cep(e, 0.95),
        "rms_m": float(math.sqrt(float(np.mean(e * e)))),
    }
