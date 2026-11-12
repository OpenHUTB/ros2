"""差速底盘运动学、指令限幅与轮速反解。

本模块不依赖 ROS，可在纯 Python + NumPy 环境下单独运行与测试，
便于离线复现与单元测试；ROS2 节点只负责话题收发，算法全部在这里。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Twist:
    """底盘速度指令。``v`` 为机体前进速度 [m/s]，``w`` 为偏航角速度 [rad/s]。"""

    v: float = 0.0
    w: float = 0.0

    def is_zero(self, eps: float = 1e-6) -> bool:
        return abs(self.v) < eps and abs(self.w) < eps

    def scaled(self, k: float) -> "Twist":
        return Twist(self.v * k, self.w * k)


@dataclass(frozen=True)
class Limits:
    """底盘与安全层共用的硬限幅。默认值对应一间教室里的差速小车。"""

    v_max: float = 0.30
    v_back_max: float = 0.15
    w_max: float = 1.20
    a_v: float = 0.60
    a_w: float = 2.50
    wheel_base: float = 0.16


def wrap_angle(angle: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp_twist(cmd: Twist, limits: Limits) -> Twist:
    """按前进/倒车/转向上限做非对称限幅。

    倒车上限比前进更严：本模块在脱困时会主动倒车，而后向通常没有传感器。
    """
    v_hi = limits.v_max
    v_lo = -limits.v_back_max
    return Twist(
        v=min(max(cmd.v, v_lo), v_hi),
        w=min(max(cmd.w, -limits.w_max), limits.w_max),
    )


def limit_accel(prev: Twist, target: Twist, dt: float, limits: Limits) -> Twist:
    """按加速度上限对指令做一阶限幅，避免驱动板过流与轮子打滑。"""
    if dt <= 0.0:
        return prev
    dv = limits.a_v * dt
    dw = limits.a_w * dt
    return Twist(
        v=prev.v + min(max(target.v - prev.v, -dv), dv),
        w=prev.w + min(max(target.w - prev.w, -dw), dw),
    )


def integrate_pose(pose: tuple[float, float, float], cmd: Twist, dt: float) -> tuple[float, float, float]:
    """按圆弧积分把速度指令推进一个周期，返回新的 ``(x, y, yaw)``。

    与一阶欧拉相比，圆弧积分在纯转向（``v=0``）与低速大曲率时不会产生位置漂移。
    """
    x, y, yaw = pose
    v, w = cmd.v, cmd.w
    if abs(w) < 1e-6:
        nx = x + v * dt * math.cos(yaw)
        ny = y + v * dt * math.sin(yaw)
        nyaw = yaw
    else:
        radius = v / w
        nyaw = wrap_angle(yaw + w * dt)
        nx = x + radius * (math.sin(nyaw) - math.sin(yaw))
        ny = y - radius * (math.cos(nyaw) - math.cos(yaw))
    return nx, ny, nyaw


def wheel_speeds(cmd: Twist, wheel_base: float) -> tuple[float, float]:
    """由 (v, w) 反解左右轮线速度 [m/s]。"""
    half = 0.5 * wheel_base
    return cmd.v - cmd.w * half, cmd.v + cmd.w * half


def wheels_to_twist(left: float, right: float, wheel_base: float) -> Twist:
    """由左右轮线速度正解 (v, w)。"""
    return Twist(v=0.5 * (left + right), w=(right - left) / wheel_base)


def body_forward_delta(prev: tuple[float, float, float], now: tuple[float, float, float]) -> float:
    """相邻里程计位姿之间的机体前进位移（带符号），用于判定"车到底动没动"。

    用机体坐标系投影而不是欧氏距离：原地转向时欧氏距离≈0，但车并没有被顶住。
    """
    dx = now[0] - prev[0]
    dy = now[1] - prev[1]
    return dx * math.cos(prev[2]) + dy * math.sin(prev[2])
