#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景几何的安全间距：机体中心到障碍物表面与地面的最小距离。"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

import numpy as np

__all__ = ["DRONE_RADIUS", "clearance_at", "clearance_points", "min_clearance",
           "swept_clearance", "swept_clearance_many", "rollout_clearance_many"]

DRONE_RADIUS = 0.16


def _box_sdf(p: np.ndarray, c: np.ndarray, half: np.ndarray) -> float:
    q = np.abs(p - c) - half
    outside = float(np.linalg.norm(np.maximum(q, 0.0)))
    inside = float(min(max(q[0], q[1], q[2]), 0.0))
    return outside + inside


def _cylinder_sdf(p: np.ndarray, c: np.ndarray, r: float, hz: float) -> float:
    d = p - c
    q0 = math.hypot(d[0], d[1]) - r
    q1 = abs(d[2]) - hz
    outside = math.hypot(max(q0, 0.0), max(q1, 0.0))
    inside = min(max(q0, q1), 0.0)
    return outside + inside


def _sphere_sdf(p: np.ndarray, c: np.ndarray, r: float) -> float:
    return float(np.linalg.norm(p - c) - r)


def clearance_at(scene, p: Sequence[float]) -> float:
    """机体中心到最近障碍表面的距离减去机体半径；地面按 z = 0 计。"""
    p = np.asarray(p, float)
    best = float(p[2])
    for o in scene.objects:
        c = np.asarray(o.pos, float)
        if o.kind == "box":
            d = _box_sdf(p, c, np.asarray(o.size, float))
        elif o.kind == "cylinder":
            d = _cylinder_sdf(p, c, float(o.size[0]), float(o.size[1]))
        elif o.kind == "sphere":
            d = _sphere_sdf(p, c, float(o.size[0]))
        else:
            continue
        best = min(best, d)
    return float(best - DRONE_RADIUS)


def min_clearance(scene, points: Iterable[Sequence[float]]) -> float:
    return min((clearance_at(scene, p) for p in points), default=float("inf"))


def swept_clearance(scene, p: Sequence[float], v: Sequence[float],
                    horizon: float = 0.6, n: int = 12) -> float:
    """沿当前速度外推 ``horizon`` 秒，路径上的最小间距。"""
    return float(swept_clearance_many(scene, p, np.asarray(v, float)[None, :],
                                      horizon, n)[0])


def clearance_points(scene, pts: np.ndarray) -> np.ndarray:
    """点集 (...,3) → 每个点到最近障碍/地面的间距。"""
    pts = np.asarray(pts, float)
    best = pts[..., 2].copy()
    for o in scene.objects:
        c = np.asarray(o.pos, float)
        q = pts - c
        if o.kind == "box":
            e = np.abs(q) - np.asarray(o.size, float)
            outside = np.linalg.norm(np.maximum(e, 0.0), axis=-1)
            inside = np.minimum(np.max(e, axis=-1), 0.0)
            d = outside + inside
        elif o.kind == "cylinder":
            q0 = np.hypot(q[..., 0], q[..., 1]) - float(o.size[0])
            q1 = np.abs(q[..., 2]) - float(o.size[1])
            d = np.hypot(np.maximum(q0, 0.0), np.maximum(q1, 0.0)) \
                + np.minimum(np.maximum(q0, q1), 0.0)
        elif o.kind == "sphere":
            d = np.linalg.norm(q, axis=-1) - float(o.size[0])
        else:
            continue
        best = np.minimum(best, d)
    return best - DRONE_RADIUS


def swept_clearance_many(scene, p: Sequence[float], vs: np.ndarray,
                         horizon: float = 0.6, n: int = 12) -> np.ndarray:
    """批量版：``vs`` (K,3) → 每个速度的直线扫掠间距 (K,)。"""
    p = np.asarray(p, float)
    vs = np.asarray(vs, float).reshape(-1, 3)
    ts = np.linspace(0.0, float(horizon), int(n))
    pts = p[None, None, :] + vs[:, None, :] * ts[None, :, None]
    return clearance_points(scene, pts).min(axis=1)


def rollout_clearance_many(scene, p: Sequence[float], vs: np.ndarray,
                           yaw: float, us: np.ndarray,
                           horizon: float = 0.6, dt: float = 0.05,
                           tau: float = 0.12, max_yaw_rate: float = 2.5
                           ) -> np.ndarray:
    """按环境的一阶速度低通与航向跟随，外推候选指令，返回最小间距。"""
    p = np.asarray(p, float)
    v = np.asarray(vs, float).reshape(-1, 3).copy()
    u = np.asarray(us, float).reshape(-1, 3)
    n = max(int(round(horizon / dt)), 1)
    k = min(1.0, dt / max(tau, 1e-6))
    yaw_k = np.full(len(v), float(yaw))
    pos = np.repeat(p[None, :], len(v), axis=0)
    best = clearance_points(scene, pos)
    for _ in range(n):
        c, s = np.cos(yaw_k), np.sin(yaw_k)
        a_w = np.stack([c * u[:, 0] - s * u[:, 1],
                        s * u[:, 0] + c * u[:, 1], u[:, 2]], axis=1)
        v = v + k * (a_w - v)
        pos = pos + v * dt
        vh = np.hypot(v[:, 0], v[:, 1])
        want = np.arctan2(v[:, 1], v[:, 0])
        d = np.arctan2(np.sin(want - yaw_k), np.cos(want - yaw_k))
        lim = max_yaw_rate * dt
        yaw_k = yaw_k + np.where(vh > 0.4, np.clip(d, -lim, lim), 0.0)
        best = np.minimum(best, clearance_points(scene, pos))
    return best
