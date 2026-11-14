#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""几何间距：机体中心到障碍表面的有符号距离，以及按时变障碍外推的扫掠间距。

障碍有两种表示：``(kind, pos, size)`` 三元组（少量点、可读性好）与按类型
堆叠的数组批（大量点、一次算完）。教师层对 ~110 个候选动作做 0.6 s 外推，
必须走批处理，否则每帧光 numpy 调用开销就要 30 ms。
"""
from __future__ import annotations

import math
from typing import Callable, Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

__all__ = ["DRONE_RADIUS", "Primitive", "Batch", "box_sdf", "cylinder_sdf",
           "sphere_sdf", "primitive_distance", "stack_batch",
           "clearance_points_batch", "clearance_points", "clearance_at",
           "swept_clearance", "rollout_clearance"]

DRONE_RADIUS = 0.16
Primitive = Tuple[str, np.ndarray, np.ndarray]
Batch = Dict[str, np.ndarray]

_EMPTY = np.zeros((0, 3), float)


def box_sdf(pts: np.ndarray, c: np.ndarray, half: np.ndarray) -> np.ndarray:
    q = np.abs(pts - c) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
    inside = np.minimum(np.max(q, axis=-1), 0.0)
    return outside + inside


def cylinder_sdf(pts: np.ndarray, c: np.ndarray, r: float, hz: float) -> np.ndarray:
    d = pts - c
    q0 = np.hypot(d[..., 0], d[..., 1]) - float(r)
    q1 = np.abs(d[..., 2]) - float(hz)
    return (np.hypot(np.maximum(q0, 0.0), np.maximum(q1, 0.0))
            + np.minimum(np.maximum(q0, q1), 0.0))


def sphere_sdf(pts: np.ndarray, c: np.ndarray, r: float) -> np.ndarray:
    return np.linalg.norm(pts - c, axis=-1) - float(r)


def primitive_distance(pts: np.ndarray, prim: Primitive) -> np.ndarray:
    """点到单个几何体的有符号距离（内部为负）。"""
    kind, pos, size = prim
    pts = np.asarray(pts, float)
    size = np.asarray(size, float)
    if kind == "box":
        return box_sdf(pts, np.asarray(pos, float), size)
    if kind == "cylinder":
        return cylinder_sdf(pts, np.asarray(pos, float), float(size[0]),
                            float(size[1]))
    if kind == "sphere":
        return sphere_sdf(pts, np.asarray(pos, float), float(size[0]))
    raise ValueError("未知几何体 %r" % (kind,))


# --------------------------------------------------------------------------- #
# 批处理
# --------------------------------------------------------------------------- #
def _empty_batch() -> Batch:
    return {"box_pos": _EMPTY.copy(), "box_half": _EMPTY.copy(),
            "cyl_pos": _EMPTY.copy(), "cyl_r": np.zeros(0),
            "cyl_hz": np.zeros(0), "sph_pos": _EMPTY.copy(),
            "sph_r": np.zeros(0)}


def stack_batch(prims: Iterable[Primitive]) -> Batch:
    """把几何体列表堆成按类型分组的数组批。"""
    box_p, box_h, cyl_p, cyl_r, cyl_h, sph_p, sph_r = [], [], [], [], [], [], []
    for kind, pos, size in prims:
        size = np.asarray(size, float).reshape(-1)
        if kind == "box":
            box_p.append(np.asarray(pos, float))
            box_h.append(size)
        elif kind == "cylinder":
            cyl_p.append(np.asarray(pos, float))
            cyl_r.append(float(size[0]))
            cyl_h.append(float(size[1]))
        elif kind == "sphere":
            sph_p.append(np.asarray(pos, float))
            sph_r.append(float(size[0]))
    out = _empty_batch()
    if box_p:
        out["box_pos"] = np.asarray(box_p, float)
        out["box_half"] = np.asarray(box_h, float)
    if cyl_p:
        out["cyl_pos"] = np.asarray(cyl_p, float)
        out["cyl_r"] = np.asarray(cyl_r, float)
        out["cyl_hz"] = np.asarray(cyl_h, float)
    if sph_p:
        out["sph_pos"] = np.asarray(sph_p, float)
        out["sph_r"] = np.asarray(sph_r, float)
    return out


def merge_batch(a: Batch, b: Batch) -> Batch:
    """合并两个批（静态几何 + 当前时刻的运动障碍）。"""
    keys = ("box_pos", "box_half", "cyl_pos", "cyl_r", "cyl_hz",
            "sph_pos", "sph_r")
    out: Batch = {}
    for k in keys:
        va, vb = np.asarray(a[k]), np.asarray(b[k])
        if va.size == 0:
            out[k] = vb
        elif vb.size == 0:
            out[k] = va
        else:
            out[k] = np.concatenate([va, vb], axis=0)
    return out


def clearance_points_batch(batch: Batch, pts: np.ndarray) -> np.ndarray:
    """点集 (N,3) → 到最近障碍表面与地面的间距，再减去机体半径（向量化）。"""
    pts = np.asarray(pts, float).reshape(-1, 3)
    best = pts[:, 2].copy()

    bp = batch.get("box_pos")
    if bp is not None and len(bp):
        q = np.abs(pts[:, None, :] - bp[None, :, :]) - batch["box_half"][None]
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
        inside = np.minimum(np.max(q, axis=-1), 0.0)
        best = np.minimum(best, (outside + inside).min(axis=1))

    cp = batch.get("cyl_pos")
    if cp is not None and len(cp):
        d = pts[:, None, :] - cp[None, :, :]
        q0 = np.hypot(d[..., 0], d[..., 1]) - batch["cyl_r"][None, :]
        q1 = np.abs(d[..., 2]) - batch["cyl_hz"][None, :]
        dist = (np.hypot(np.maximum(q0, 0.0), np.maximum(q1, 0.0))
                + np.minimum(np.maximum(q0, q1), 0.0))
        best = np.minimum(best, dist.min(axis=1))

    sp = batch.get("sph_pos")
    if sp is not None and len(sp):
        d = np.linalg.norm(pts[:, None, :] - sp[None, :, :], axis=-1)
        best = np.minimum(best, (d - batch["sph_r"][None, :]).min(axis=1))
    return best - DRONE_RADIUS


def clearance_points(prims: Iterable[Primitive], pts: np.ndarray) -> np.ndarray:
    """点集 (N,3) → 间距；少量点用（逐个几何体循环）。"""
    pts = np.asarray(pts, float).reshape(-1, 3)
    best = pts[:, 2].copy()
    for prim in prims:
        best = np.minimum(best, primitive_distance(pts, prim))
    return best - DRONE_RADIUS


def clearance_at(prims: Iterable[Primitive], p: Sequence[float]) -> float:
    return float(clearance_points(prims, np.asarray(p, float).reshape(1, 3))[0])


def swept_clearance(prims: Iterable[Primitive], p: Sequence[float],
                    v: Sequence[float], horizon: float = 0.6,
                    n: int = 12) -> float:
    """沿速度直线外推的间距（不做低通/航向外推的近似）。"""
    p = np.asarray(p, float)
    v = np.asarray(v, float)
    ts = np.linspace(0.0, float(horizon), int(n))
    pts = p[None, :] + v[None, :] * ts[:, None]
    return float(clearance_points(prims, pts).min())


def rollout_clearance(batch_at: Callable[[float], Batch], p: Sequence[float],
                      vs: np.ndarray, yaw: float, us: np.ndarray,
                      horizon: float = 0.6, dt: float = 0.05,
                      tau: float = 0.12, max_yaw_rate: float = 1.5,
                      acc_max: float = 4.0, t0: float = 0.0) -> np.ndarray:
    """按环境的加速度限幅 + 一阶速度低通 + 航向跟随外推，返回路径最小间距。

    ``batch_at(t)`` 给出 ``t`` 时刻的障碍批，因此对运动障碍也是精确的
    （教师在仿真里知道障碍的运动参数）。返回 (K,) 数组。
    """
    p = np.asarray(p, float).reshape(3)
    v = np.asarray(vs, float).reshape(-1, 3).copy()
    u = np.asarray(us, float).reshape(-1, 3)
    if len(u) != len(v):
        raise ValueError("vs 与 us 的候选数必须一致")
    n = max(int(round(horizon / dt)), 1)
    k = min(1.0, dt / max(tau, 1e-6))
    lim_acc = float(acc_max) * dt if acc_max > 0 else 0.0
    yaw_k = np.full(len(v), float(yaw))
    pos = np.repeat(p[None, :], len(v), axis=0)
    best = clearance_points_batch(batch_at(float(t0)), pos)
    t = float(t0)
    for _ in range(n):
        c, s = np.cos(yaw_k), np.sin(yaw_k)
        a_w = np.stack([c * u[:, 0] - s * u[:, 1],
                        s * u[:, 0] + c * u[:, 1], u[:, 2]], axis=1)
        dv = (a_w - v) * k
        if lim_acc > 0:
            dv = np.clip(dv, -lim_acc, lim_acc)
        v = v + dv
        pos = pos + v * dt
        t += dt
        vh = np.hypot(v[:, 0], v[:, 1])
        want = np.arctan2(v[:, 1], v[:, 0])
        d = np.arctan2(np.sin(want - yaw_k), np.cos(want - yaw_k))
        lim = max_yaw_rate * dt
        yaw_k = yaw_k + np.where(vh > 0.4, np.clip(d, -lim, lim), 0.0)
        best = np.minimum(best, clearance_points_batch(batch_at(t), pos))
    return best
