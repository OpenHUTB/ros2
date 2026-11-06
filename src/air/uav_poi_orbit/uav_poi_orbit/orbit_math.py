#!/usr/bin/env python3
"""POI 环绕航拍纯计算函数 (不依赖 ROS), 便于独立单元测试."""

import math

TWO_PI = 2.0 * math.pi


def normalize_angle(angle):
    """把角度归一化到 [0, 2*pi), 避免长时间累积精度损失."""
    return float(angle) % TWO_PI


def compute_orbit_position(poi_x, poi_y, radius, angle):
    """圆周上一点的地面坐标.

    参数:
        poi_x, poi_y: 兴趣点地面坐标 (m)
        radius: 环绕半径 (m)
        angle: 当前环绕角度 (rad)
    返回:
        (x, y): 无人机目标地面坐标 (m)
    """
    return (poi_x + radius * math.cos(angle),
            poi_y + radius * math.sin(angle))


def compute_yaw_to_poi(poi_x, poi_y, x, y):
    """从当前位置指向 POI 的偏航角 (rad).

    无人机在 POI 圆周上时, 指向 POI 即朝向圆心, 相机始终对准目标.
    """
    return math.atan2(poi_y - y, poi_x - x)


def advance_angle(angle, total_angle, omega, dt):
    """推进环绕角度.

    参数:
        angle: 当前归一化角度 [0, 2*pi)
        total_angle: 累计总转角 (rad, 不归一化, 用于螺旋高度)
        omega: 角速度 (rad/s), 负值表示顺时针
        dt: 时间步长 (s)
    返回:
        (new_angle, new_total_angle)
    """
    step = omega * dt
    new_total = total_angle + step
    return normalize_angle(angle + step), new_total


def count_full_turns(total_angle):
    """累计总转角折算成完整圈数."""
    return abs(total_angle) / TWO_PI


def compute_height(base_height, total_angle, spiral, spiral_dh, min_height=0.5):
    """当前飞行高度.

    螺旋模式下高度随总转角连续变化 (每圈增加 spiral_dh),
    而不是按整圈阶跃, 避免圈边界处高度突变.
    """
    if not spiral:
        return float(base_height)
    z = base_height + count_full_turns(total_angle) * spiral_dh
    return max(float(z), float(min_height))


def validate_params(radius, height, omega, dt, min_height=0.5):
    """参数合法性校验, 返回错误信息列表 (空列表表示全部合法)."""
    errors = []
    if radius <= 0.0:
        errors.append(f"orbit_radius 必须 > 0, 当前 {radius}")
    if height < min_height:
        errors.append(f"orbit_height 必须 >= {min_height}, 当前 {height}")
    if abs(omega) < 1e-9:
        errors.append("omega_dps 不能为 0, 否则无人机悬停不动")
    if dt <= 0.0:
        errors.append(f"waypoint_interval_s 必须 > 0, 当前 {dt}")
    return errors
