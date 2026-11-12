"""参考轨迹：给定时间返回期望位置/速度/加速度与偏航。

用解析式而不是打点插值，好处是速度、加速度都能直接给控制器（几何控制器要用
前馈加速度），做"机动大小"的对比实验时也能一次说清强度。
"""

from __future__ import annotations

import numpy as np

#: 支持的轨迹
TRAJECTORIES = ("hover", "circle", "zigzag", "climb")


def _hover(t: float, alt: float):
    return np.array([0.0, 0.0, alt]), np.zeros(3), np.zeros(3)


def _circle(t: float, radius: float = 0.7, omega: float = 0.6, alt: float = 1.5):
    s, c = np.sin(omega * t), np.cos(omega * t)
    pos = np.array([radius * s, radius * (1.0 - c), alt])
    vel = np.array([radius * omega * c, radius * omega * s, 0.0])
    acc = np.array([-radius * omega ** 2 * s, radius * omega ** 2 * c, 0.0])
    return pos, vel, acc


def _zigzag(t: float, ax: float = 0.9, ay: float = 0.5, alt: float = 1.5):
    wx, wy = 1.2, 2.4
    s, c = np.sin(wx * t), np.cos(wx * t)
    s2, c2 = np.sin(wy * t), np.cos(wy * t)
    pos = np.array([ax * s, ay * s2, alt + 0.1 * np.sin(0.8 * t)])
    vel = np.array([ax * wx * c, ay * wy * c2, 0.08 * np.cos(0.8 * t)])
    acc = np.array([-ax * wx ** 2 * s, -ay * wy ** 2 * s2,
                    -0.064 * np.sin(0.8 * t)])
    return pos, vel, acc


def _climb(t: float, alt0: float = 1.2, alt1: float = 2.4, duration: float = 4.0):
    # 用平滑的 3 次曲线，两个端点速度、加速度都为 0
    u = float(np.clip(t / duration, 0.0, 1.0))
    h = alt1 - alt0
    pos = np.array([0.0, 0.0, alt0 + h * (3.0 * u ** 2 - 2.0 * u ** 3)])
    vel = np.array([0.0, 0.0, h * (6.0 * u - 6.0 * u ** 2) / duration])
    acc = np.array([0.0, 0.0, h * (6.0 - 12.0 * u) / duration ** 2])
    if u >= 1.0:
        vel = np.zeros(3)
        acc = np.zeros(3)
    return pos, vel, acc


def reference(kind: str, t: float):
    """返回 ``(p_d, v_d, a_d, yaw_d)``。"""
    if kind not in TRAJECTORIES:
        raise ValueError(f"未知轨迹 {kind!r}，可选：{TRAJECTORIES}")
    if kind == "hover":
        p, v, a = _hover(t, 1.5)
    elif kind == "circle":
        p, v, a = _circle(t)
    elif kind == "zigzag":
        p, v, a = _zigzag(t)
    else:
        p, v, a = _climb(t)
    return p, v, a, 0.0


def reference_batch(kind: str, times: np.ndarray):
    """把一整段时间批量算出来，``times`` 是 (N,)。"""
    out = [reference(kind, float(t)) for t in np.asarray(times, dtype=float)]
    pos = np.stack([o[0] for o in out])
    vel = np.stack([o[1] for o in out])
    acc = np.stack([o[2] for o in out])
    yaw = np.array([o[3] for o in out])
    return pos, vel, acc, yaw
