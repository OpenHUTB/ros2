# -*- coding: utf-8 -*-
"""关节轨迹的时间缩放重定时（retiming，全局常数缩放）。

问题
----
机器人工程师常拿到一条"形状不错但太慢"的轨迹：关键帧来自示教或规划，
没人保证它跑在速度/加速度上限附近。本模块回答：

    在给定各关节速度上限 v_max 与加速度上限 a_max 下，这条轨迹最快能压缩多少倍？

两步，都不需要动力学模型：

1. **插值**：自然三次样条穿过关键帧（C² 连续，加速度不跳变）；
2. **重定时**：令 τ = α·t，则 ``q̇_α = α·q̇``、``q̈_α = α²·q̈``，约束变成

       |q̇|·α  ≤ v_max   →   α ≤ v_max / |q̇|
       |q̈|·α² ≤ a_max   →   α ≤ sqrt(a_max / |q̈|)

   α 取所有关节、所有采样点的最小值，即**最快常数缩放**。

   这不是 Pham 意义下的逐点时间最优（TOP）：那种方法让时间律沿路径变化，可以在拐角
   单独减速。本模块只给一个全局常数 α——简单、可解释、可直接用于 ``JointTrajectory``
   的时间轴换算；两者的差别写在文档「方法边界」一节。

为什么不做力矩约束
------------------
力矩要逆动力学（质量矩阵、科氏项、重力项），强依赖整机模型；塞进来会让本模块变成
一个不完整的动力学仿真器。本模块只做**运动学**重定时，力矩校验留给下游控制器
或整机模型——这是边界，不是疏忽。

单位：rad、rad/s、rad/s²、s。
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np


@dataclass
class Limits:
    """逐关节的速度/加速度上限。"""

    v_max: Sequence[float]
    a_max: Sequence[float]

    def n(self) -> int:
        return len(self.v_max)


@dataclass
class Trajectory:
    """采样后的轨迹：时间轴 + 位置/速度/加速度（形状 n_joints × n_samples）。"""

    t: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    ddq: np.ndarray

    @property
    def n_joints(self) -> int:
        return self.q.shape[0]

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])


# --------------------------------------------------------------------- 样条
def _second_derivs(tk: np.ndarray, yk: np.ndarray) -> np.ndarray:
    """自然边界条件下，节点处的二阶导 M（解一个三对角方程组）。"""
    n = tk.size
    h = np.diff(tk)
    A = np.zeros((n, n))
    b = np.zeros(n)
    A[0, 0] = 1.0                       # M0 = 0
    A[-1, -1] = 1.0                     # M_{n-1} = 0
    for i in range(1, n - 1):
        A[i, i - 1] = h[i - 1]
        A[i, i] = 2.0 * (h[i - 1] + h[i])
        A[i, i + 1] = h[i]
        b[i] = 6.0 * ((yk[i + 1] - yk[i]) / h[i] - (yk[i] - yk[i - 1]) / h[i - 1])
    return np.linalg.solve(A, b)


def _eval_spline(tk: np.ndarray, yk: np.ndarray, M: np.ndarray,
                 t: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """在 t 上求值：位置、一阶导、二阶导。"""
    n = tk.size
    h = np.diff(tk)
    seg = np.clip(np.searchsorted(tk, t, side="right") - 1, 0, n - 2)

    y = np.zeros_like(t)
    dy = np.zeros_like(t)
    ddy = np.zeros_like(t)
    for k in range(n - 1):
        m = seg == k
        if not np.any(m):
            continue
        x = t[m] - tk[k]
        hk = h[k]
        y[m] = (M[k] * (hk - x) ** 3 / (6 * hk)
                + M[k + 1] * x ** 3 / (6 * hk)
                + (yk[k] / hk - M[k] * hk / 6.0) * (hk - x)
                + (yk[k + 1] / hk - M[k + 1] * hk / 6.0) * x)
        dy[m] = (-M[k] * (hk - x) ** 2 / (2 * hk)
                 + M[k + 1] * x ** 2 / (2 * hk)
                 + (yk[k + 1] - yk[k]) / hk
                 - hk * (M[k + 1] - M[k]) / 6.0)
        ddy[m] = M[k] * (hk - x) / hk + M[k + 1] * x / hk
    return y, dy, ddy


def build_trajectory(t_knots: Sequence[float], q_knots: np.ndarray,
                     dt: float = 0.001) -> Trajectory:
    """把 (n_joints × n_knots) 的关键帧插值为采样轨迹。"""
    tk = np.asarray(t_knots, dtype=float)
    qk = np.asarray(q_knots, dtype=float)
    if qk.ndim != 2:
        raise ValueError("q_knots 形状应为 (n_joints, n_knots)")
    if qk.shape[1] != tk.size:
        raise ValueError("关键帧数与时间点数不一致")
    if tk.size < 2:
        raise ValueError("至少需要 2 个关键帧")

    steps = np.diff(tk)
    if np.any(~np.isfinite(steps)):
        raise ValueError("时间戳必须是有限值")
    if np.any(steps <= 0):
        i = int(np.argmax(steps <= 0))
        raise ValueError(
            "时间戳必须严格递增：第 %d→%d 个关键帧的间隔为 %.6g s（不允许 0 或负间隔）"
            % (i, i + 1, steps[i]))

    t = np.arange(tk[0], tk[-1] + 1e-12, dt)
    qs, dqs, ddqs = [], [], []
    for j in range(qk.shape[0]):
        M = _second_derivs(tk, qk[j])
        y, dy, ddy = _eval_spline(tk, qk[j], M, t)
        qs.append(y)
        dqs.append(dy)
        ddqs.append(ddy)
    return Trajectory(t=t, q=np.vstack(qs), dq=np.vstack(dqs), ddq=np.vstack(ddqs))


# --------------------------------------------------------------------- 重定时
def max_time_scale(traj: Trajectory, limits: Limits) -> Dict[str, object]:
    """满足所有关节 v/a 上限的最大时间缩放因子 α 及"谁在限制"。"""
    v = np.abs(traj.dq).max(axis=1)
    a = np.abs(traj.ddq).max(axis=1)
    vmax = np.asarray(limits.v_max, dtype=float)
    amax = np.asarray(limits.a_max, dtype=float)
    if vmax.size != v.size or amax.size != a.size:
        raise ValueError("上限维度与关节数不一致")

    with np.errstate(divide="ignore", invalid="ignore"):
        alpha_v = np.where(v > 0, vmax / v, np.inf)
        alpha_a = np.where(a > 0, np.sqrt(amax / a), np.inf)

    alpha = float(min(alpha_v.min(), alpha_a.min()))
    if not math.isfinite(alpha) or alpha <= 0:
        raise ValueError("轨迹静止或上限非正，无法重定时")

    kind = "speed" if alpha_v.min() <= alpha_a.min() else "accel"
    idx = int(np.argmin(alpha_v if kind == "speed" else alpha_a))
    return {
        "alpha": alpha,
        "limited_by": kind,
        "joint": idx,
        "duration_before": traj.duration,
        "duration_after": traj.duration / alpha,
        "alpha_v": alpha_v.tolist(),
        "alpha_a": alpha_a.tolist(),
    }


def retime(traj: Trajectory, alpha: float) -> Trajectory:
    """按 α 压缩时间轴：t → t/α，q̇ → α·q̇，q̈ → α²·q̈；位置曲线不变。"""
    if alpha <= 0:
        raise ValueError("α 必须为正")
    return Trajectory(t=(traj.t - traj.t[0]) / alpha,
                      q=traj.q.copy(),
                      dq=traj.dq * alpha,
                      ddq=traj.ddq * alpha ** 2)


def split_duration(seconds: float) -> Tuple[int, int]:
    """把秒拆成 ``builtin_interfaces/Duration`` 的 (sec, nanosec)。

    直接写 ``int(nt)`` 与 ``round((nt - int(nt)) * 1e9)`` 会在 nt = 3.9999999996
    这类值上得到 ``nanosec = 1000000000``，而 ROS 的 Duration 要求 ``nanosec < 1e9``。
    先换算成整数纳秒再整除/取余，就不会越界。
    """
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("时长必须是非负有限数")
    total_ns = int(round(seconds * 1e9))
    return total_ns // 1_000_000_000, total_ns % 1_000_000_000

def utilization(traj: Trajectory, limits: Limits) -> Dict[str, List[float]]:
    """每个关节的约束占用率（≤1 表示满足），用于定位"谁在拖后腿"。"""
    v = np.abs(traj.dq).max(axis=1) / np.asarray(limits.v_max, dtype=float)
    a = np.abs(traj.ddq).max(axis=1) / np.asarray(limits.a_max, dtype=float)
    return {"speed": v.tolist(), "accel": a.tolist()}


def peak_jerk(traj: Trajectory) -> float:
    """峰值加加速度（rad/s³），用于对比重定时前后的抖动。"""
    return float(np.abs(np.gradient(traj.ddq, traj.t, axis=1)).max())
