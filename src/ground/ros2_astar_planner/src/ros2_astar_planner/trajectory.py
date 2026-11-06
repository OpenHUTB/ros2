"""时间参数化：把几何路径变成带速度/时间戳的可执行轨迹。

规划器只回答"走哪条线"，控制器还需要知道"每个点什么时候到、该跑多快"。
本模块用**梯形速度剖面**给路径分配速度：

1. 由相邻三点外接圆估计各点曲率 :math:`\\kappa`；
2. 曲率限速 :math:`v \\le \\sqrt{a_{lat}/|\\kappa|}`，避免转弯时侧向加速度超限；
3. 沿弧长做**前向 + 后向两遍扫描**，用 :math:`v^2 - v_0^2 = 2 a \\Delta s`
   保证加减速不超过 ``a_max``（后向扫描负责"提前刹车"）；
4. 逐段按平均速度积分得到时间戳。

输出可直接由纯追踪控制器按时间/弧长插值消费，也便于离线评估总耗时。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Trajectory:
    """时间参数化结果。"""

    points: np.ndarray      # (N, 2) 世界坐标
    s: np.ndarray           # (N,) 累计弧长 (m)
    v: np.ndarray           # (N,) 各点允许速度 (m/s)
    t: np.ndarray           # (N,) 时间戳 (s)
    curvature: np.ndarray   # (N,) 曲率 (1/m)
    total_time: float
    total_length: float

    @property
    def v_max_actual(self) -> float:
        return float(np.max(self.v)) if len(self.v) else 0.0


def compute_curvature(points: np.ndarray) -> np.ndarray:
    """三点外接圆法估计各点曲率，端点曲率置 0。"""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    kappa = np.zeros(n)
    if n < 3:
        return kappa
    for i in range(1, n - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        ab, bc, ac = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(c - a)
        # 三角形面积的两倍（叉积绝对值）
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        if ab < 1e-9 or bc < 1e-9 or ac < 1e-9:
            continue
        # R = (abc) / (4 * Area) = (ab*bc*ac) / (2*cross)
        kappa[i] = 2.0 * cross / (ab * bc * ac)
    return kappa


def time_parameterize(
    points: np.ndarray,
    v_max: float = 0.6,
    a_max: float = 0.5,
    a_lat_max: float = 1.0,
    v_start: float = 0.0,
    v_end: float = 0.0,
) -> Trajectory:
    """给几何路径分配速度剖面与时间戳。

    Args:
        v_max, a_max, a_lat_max: 线速度上限、纵向加速度上限、侧向加速度上限。
        v_start, v_end: 起终点速度（默认 0，即起步停车）。
    """
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n == 0:
        empty = np.zeros(0)
        return Trajectory(pts.copy(), empty, empty, empty, empty, 0.0, 0.0)
    if n == 1:
        z = np.zeros(1)
        return Trajectory(pts.copy(), z, z, z, z, 0.0, 0.0)

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    kappa = compute_curvature(pts)

    # 1) 曲率限速（曲率接近 0 时视为无限速）
    v_cap = np.full(n, float(max(v_max, 1e-6)))
    for i in range(n):
        if kappa[i] > 1e-6:
            v_cap[i] = min(v_cap[i], math.sqrt(max(a_lat_max, 0.0) / kappa[i]))

    # 2) 前向扫描：受加速度限制能加速到多快
    v = np.zeros(n)
    v[0] = min(v_start, v_cap[0])
    for i in range(1, n):
        ds = max(seg[i - 1], 0.0)
        v_acc = math.sqrt(max(0.0, v[i - 1] ** 2 + 2.0 * a_max * ds))
        v[i] = min(v_cap[i], v_acc)

    # 3) 后向扫描：为了能在终点按要求速度停下，必须提前减速
    v[-1] = min(v[-1], v_end, v_cap[-1])
    for i in range(n - 2, -1, -1):
        ds = max(seg[i], 0.0)
        v_dec = math.sqrt(max(0.0, v[i + 1] ** 2 + 2.0 * a_max * ds))
        v[i] = min(v[i], v_dec)

    # 4) 逐段积分时间：匀变速假设 ds = (v0+v1)/2 * dt
    t = np.zeros(n)
    for i in range(1, n):
        v_avg = 0.5 * (v[i - 1] + v[i])
        if v_avg <= 1e-9:
            # 极端情况下用加速度下限兜底，避免除零
            dt = math.sqrt(2.0 * max(seg[i - 1], 0.0) / max(a_max, 1e-6))
        else:
            dt = seg[i - 1] / v_avg
        t[i] = t[i - 1] + dt

    return Trajectory(
        points=pts.copy(),
        s=s,
        v=v,
        t=t,
        curvature=kappa,
        total_time=float(t[-1]),
        total_length=float(s[-1]),
    )


def sample_trajectory(traj: Trajectory, dt: float = 0.05) -> np.ndarray:
    """按固定周期对轨迹线性插值，输出 ``(M, 3)`` 的 ``(x, y, t)``。

    供控制器按时间查表使用；返回列依次为 x, y, 时间戳。
    """
    if len(traj.t) < 2 or traj.total_time <= 0:
        return np.zeros((0, 3))
    ts = np.arange(0.0, traj.total_time + dt * 0.5, dt)
    ts = np.clip(ts, 0.0, traj.total_time)
    x = np.interp(ts, traj.t, traj.points[:, 0])
    y = np.interp(ts, traj.t, traj.points[:, 1])
    return np.stack([x, y, ts], axis=1)


def check_acceleration(traj: Trajectory, a_max: float, tol: float = 1e-6) -> bool:
    """校验速度剖面是否满足加速度约束（用于单元测试）。"""
    for i in range(1, len(traj.v)):
        ds = traj.s[i] - traj.s[i - 1]
        if ds <= 1e-9:
            continue
        a = (traj.v[i] ** 2 - traj.v[i - 1] ** 2) / (2.0 * ds)
        if abs(a) > a_max + tol:
            return False
    return True
