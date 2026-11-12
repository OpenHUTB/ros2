# -*- coding: utf-8 -*-
"""机器人相关的坐标与角度小工具。

这里只放"错一次就很难查"的换算，别的模块不允许自己复制一份：

* ``wrap_pi``       角度归一到 (-π, π]，**所有**角度差都必须经过它，
                    否则跟踪误差会在 ±π 处跳变 2π，控制器会突然反向。
* ``quat_to_roll_pitch`` 由 IMU 四元数解横滚/俯仰。显式给出而不是让调用方
                    自己写，避免"用欧拉角中间变量的顺序"这类隐蔽错误。
* ``rot2`` / ``transform`` 二维/三维齐次变换，规划与控制共用同一套约定
                    （x 前、y 左、z 上，逆时针为正）。
"""

from __future__ import annotations

import math


def wrap_pi(angle: float) -> float:
    """把角度归一到 (-π, π]。

    用 ``math.remainder`` 而不是 ``(a + π) % 2π - π``：前者对负数的处理
    也符合"就近取同余"，不会在输入恰为整圆整数倍时得到 -π。
    """
    return float(math.remainder(float(angle), 2.0 * math.pi))


def quat_to_roll_pitch(w: float, x: float, y: float, z: float) -> tuple:
    """由四元数 (w, x, y, z) 解横滚角与俯仰角（弧度）。

    使用 ZYX 欧拉角约定：先绕 z（偏航）、再绕 y（俯仰）、最后绕 x（横滚）。
    MuJoCo ``framequat`` 传感器输出的就是这个分量顺序；ROS 的
    ``sensor_msgs/Imu`` 也一致（x,y,z,w → 调用方按 (w,x,y,z) 传入）。

    横滚用 ``atan2`` 而不是 ``asin``：俯仰接近 ±90° 时 ``asin`` 的输入会
    超出 [-1,1] 产生 NaN，而 atan2 形式在该奇异点附近依然连续。
    """
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    """由四元数解偏航角（弧度，逆时针为正）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rot2(theta: float) -> list:
    """二维旋转矩阵，返回 [[cos, -sin], [sin, cos]]（行优先）。"""
    c, s = math.cos(theta), math.sin(theta)
    return [[c, -s], [s, c]]


def transform(x: float, y: float, theta: float):
    """生成把"本体坐标系"映射到"世界坐标系"的二维齐次变换。

    返回一个函数 ``f(px, py) -> (wx, wy)``，本体系与坐标系约定一致
    （x 前、y 左），因此航向角 θ 直接参与旋转。
    """
    c, s = math.cos(theta), math.sin(theta)

    def _apply(px: float, py: float) -> tuple:
        return (x + c * px - s * py, y + s * px + c * py)

    return _apply


def clamp(value: float, low: float, high: float) -> float:
    """把 value 限制在 [low, high] 内。"""
    return low if value < low else high if value > high else value


def lerp(a: float, b: float, t: float) -> float:
    """线性插值，``t`` 不裁剪（需要裁剪请先调用 :func:`clamp`）。"""
    return a + (b - a) * t


def rate_limit(prev: float, target: float, max_step: float) -> float:
    """把 ``target`` 限制为相对 ``prev`` 最多变化 ``max_step``。

    关节目标量在状态切换（例如支撑腿↔摆动腿）时会出现阶跃，直接下发会让
    位置伺服瞬间输出很大力矩、把机器人自己踢倒；用速率限制平滑掉这一步。
    """
    delta = target - prev
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return prev + delta
