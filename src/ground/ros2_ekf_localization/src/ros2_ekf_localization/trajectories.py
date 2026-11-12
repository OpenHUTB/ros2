"""真值参考轨迹：为离线仿真生成无噪声的位姿序列与对应的里程计输入。

设计要点：**先按等弧长重采样，再反推 (v, w)**。
这样在无噪声条件下，用里程计积分可以严格复现真值轨迹，
仿真里的所有误差都可以明确归因到传感器噪声，而不是采样方式本身。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .ekf import normalize_angle


@dataclass
class Trajectory:
    """一条等速参考轨迹。"""

    t: np.ndarray      # 时间戳 (s)
    x: np.ndarray      # 真值位置 x (m)
    y: np.ndarray      # 真值位置 y (m)
    theta: np.ndarray  # 真值航向 (rad)
    v: np.ndarray      # 与该位姿序列对应的线速度输入 (m/s)
    w: np.ndarray      # 角速度输入 (rad/s)
    dt: float
    name: str

    def __len__(self) -> int:
        return len(self.t)

    def state(self, k: int) -> np.ndarray:
        """第 k 步的真值状态向量。"""
        return np.array([self.x[k], self.y[k], self.theta[k]], dtype=float)


def _resample_by_arclength(points: np.ndarray, ds: float) -> np.ndarray:
    """把密集采样的曲线按弧长等间距重采样。"""
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    length = s[-1]
    n = max(int(math.floor(length / ds)), 2)
    s_new = np.arange(n + 1, dtype=float) * ds
    x = np.interp(s_new, s, points[:, 0])
    y = np.interp(s_new, s, points[:, 1])
    return np.column_stack([x, y])


def build(points: np.ndarray, speed: float, dt: float, name: str) -> Trajectory:
    """由密集曲线点构建等速轨迹，并反推里程计输入 (v, w)。"""
    pts = _resample_by_arclength(np.asarray(points, dtype=float), speed * dt)
    n = len(pts)

    # 航向：相邻点的前进方向（最后一步沿用前一步）
    heading = np.arctan2(np.diff(pts[:, 1]), np.diff(pts[:, 0]))
    theta = np.concatenate([heading, [heading[-1]]])

    # 里程计输入：与离散运动学严格互逆
    #   dx = v cos(theta) dt,  dy = v sin(theta) dt
    delta = np.diff(pts, axis=0)
    v = np.empty(n)
    v[:-1] = (delta[:, 0] * np.cos(theta[:-1]) + delta[:, 1] * np.sin(theta[:-1])) / dt
    v[-1] = v[-2]
    w = np.empty(n)
    w[:-1] = np.array([normalize_angle(theta[i + 1] - theta[i]) for i in range(n - 1)]) / dt
    w[-1] = w[-2]

    return Trajectory(
        t=np.arange(n, dtype=float) * dt,
        x=pts[:, 0],
        y=pts[:, 1],
        theta=theta,
        v=v,
        w=w,
        dt=dt,
        name=name,
    )


def circle(radius: float = 8.0, speed: float = 0.8, dt: float = 0.05) -> Trajectory:
    """圆形轨迹。"""
    u = np.linspace(0.0, 2.0 * np.pi, 4000)
    pts = np.column_stack([radius * np.cos(u), radius * np.sin(u)])
    return build(pts, speed, dt, "circle")


def figure_eight(a: float = 10.0, b: float = 5.0, speed: float = 0.8, dt: float = 0.05) -> Trajectory:
    """8 字形（利萨如）轨迹。"""
    u = np.linspace(0.0, 2.0 * np.pi, 4000)
    pts = np.column_stack([a * np.sin(u), b * np.sin(2.0 * u)])
    return build(pts, speed, dt, "eight")


def s_curve(length: float = 24.0, amp: float = 3.0, speed: float = 0.8, dt: float = 0.05) -> Trajectory:
    """S 形轨迹。"""
    u = np.linspace(0.0, 1.0, 4000)
    pts = np.column_stack([length * u, amp * np.sin(2.0 * np.pi * u)])
    return build(pts, speed, dt, "s_curve")


TRAJECTORY_BUILDERS = {
    "circle": circle,
    "eight": figure_eight,
    "s_curve": s_curve,
}
