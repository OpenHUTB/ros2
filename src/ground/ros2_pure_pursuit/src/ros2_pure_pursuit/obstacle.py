"""基于二维激光扫描的前向避障模块。

思路：把 360 度扫描划分为若干扇区，取前半球（车头方向 ±sector_half_angle）
内的最小测距作为危险度；危险度越高，则叠加越强的转向偏置，并线性降低速度。

该方法属于反应式（reactive）局部避障，计算量极小，适合作为全局路径跟踪
之上的安全兜底层。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class AvoidInfo:
    min_front: float   # 前向扇区最小测距 (m)
    risk: float        # 危险度，0(安全) ~ 1( imminent )
    turn_bias: float   # 附加转向角速度 (rad/s)，正为左转
    slow_factor: float # 速度缩放系数 0~1


class ObstacleAvoider:
    def __init__(
        self,
        safe_distance: float = 1.2,
        danger_distance: float = 0.45,
        sector_half_angle: float = 60.0,
        max_range: float = 10.0,
        gain: float = 1.6,
    ) -> None:
        self.safe_distance = safe_distance
        self.danger_distance = danger_distance
        self.sector_half_angle = math.radians(sector_half_angle)
        self.max_range = max_range
        self.gain = gain

    def evaluate(self, ranges: np.ndarray, angle_min: float, angle_increment: float) -> AvoidInfo:
        """输入激光扫描，输出避障建议。

        ranges 中无穷远用 inf/nan 表示，内部统一裁剪到 max_range。
        车体前向定义为扫描角 0 弧度处。
        """
        r = np.asarray(ranges, dtype=float).copy()
        r[~np.isfinite(r)] = self.max_range
        r = np.clip(r, 0.0, self.max_range)

        angles = angle_min + angle_increment * np.arange(len(r))
        mask = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) <= self.sector_half_angle
        if not np.any(mask):
            return AvoidInfo(self.max_range, 0.0, 0.0, 1.0)

        front_r = r[mask]
        front_a = angles[mask]
        min_front = float(np.min(front_r))

        # 危险度：safe 之外为 0，danger 之内为 1，线性过渡
        span = max(self.safe_distance - self.danger_distance, 1e-6)
        risk = float(np.clip((self.safe_distance - min_front) / span, 0.0, 1.0))
        if risk <= 0.0:
            return AvoidInfo(min_front, 0.0, 0.0, 1.0)

        # 朝更空旷的一侧转向：比较左右半扇区的平均余量
        left = front_a > 0
        right = ~left
        left_clear = float(np.mean(front_r[left])) if np.any(left) else 0.0
        right_clear = float(np.mean(front_r[right])) if np.any(right) else 0.0
        direction = 1.0 if left_clear >= right_clear else -1.0

        turn_bias = direction * self.gain * risk
        slow_factor = float(np.clip(1.0 - 0.85 * risk, 0.15, 1.0))
        return AvoidInfo(min_front, risk, turn_bias, slow_factor)


def synthetic_scan(
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    obstacles: list[tuple[float, float, float]],
    n: int = 360,
    max_range: float = 10.0,
) -> tuple[np.ndarray, float, float]:
    """为离线仿真生成合成激光扫描。

    obstacles 为 (x, y, radius) 列表。
    返回 (ranges, angle_min, angle_increment)。
    """
    angle_min = -np.pi
    angle_increment = 2.0 * np.pi / n
    angles = angle_min + angle_increment * np.arange(n)
    ranges = np.full(n, max_range, dtype=float)

    for ox, oy, orad in obstacles:
        dx, dy = ox - robot_x, oy - robot_y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - robot_theta
        # 圆的切线夹角
        if dist <= orad:
            continue
        half = np.arcsin(min(orad / dist, 1.0))
        diff = np.arctan2(np.sin(angles - bearing), np.cos(angles - bearing))
        hit = np.abs(diff) <= half
        if not np.any(hit):
            continue
        # 射线与圆求交（取近端交点）
        along = np.cos(diff[hit]) * dist
        perp = np.sin(diff[hit]) * dist
        inner = orad**2 - perp**2
        inner = np.clip(inner, 0.0, None)
        d_hit = along - np.sqrt(inner)
        ranges[hit] = np.minimum(ranges[hit], np.clip(d_hit, 0.0, max_range))

    return ranges, angle_min, angle_increment
