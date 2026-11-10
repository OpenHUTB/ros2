"""ugv_slam_nav 纯逻辑层：cmd_vel 仲裁、限幅、前向扇区避障。不依赖 ROS。"""
from __future__ import annotations
import math
from dataclasses import dataclass


@dataclass
class Limits:
    max_vx: float = 0.5       # m/s，车辆比无人机慢
    max_vy: float = 0.0        # 差速车没有横向速度
    max_yaw: float = 1.0       # rad/s


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def relay(nav2_vx: float, nav2_yaw: float,
          simple_vx: float, simple_yaw: float,
          nav2_alive: bool,
          limits: Limits = Limits()) -> tuple[float, float]:
    """仲裁：nav2 活着走 nav2，超时走 simple_nav。结果按 limits 限幅。"""
    if nav2_alive:
        vx, yaw = nav2_vx, nav2_yaw
    else:
        vx, yaw = simple_vx, simple_yaw
    return (clamp(vx, -limits.max_vx, limits.max_vx),
            clamp(yaw, -limits.max_yaw, limits.max_yaw))


def sector_speed(scan_ranges: list[float],
                 angle_min: float,
                 angle_increment: float,
                 fov_deg: float = 120.0,
                 safe_dist: float = 0.4,
                 slow_dist: float = 0.8,
                 cruise: float = 0.4) -> float:
    """前向 fov_deg 扇区内取最近距离，决定前向速度。
    距离 < safe_dist 停；safe~slow 线性减速；> slow 巡航。
    scan_ranges 为空或 inf/nan 按巡航处理。"""
    if not scan_ranges:
        return cruise
    half = math.radians(fov_deg) / 2.0
    vx = cruise
    for i, r in enumerate(scan_ranges):
        ang = angle_min + i * angle_increment
        # 折到 [-pi, pi]
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        if abs(ang) > half:
            continue
        if r != r or r == float("inf") or r <= 0:  # nan/inf/0
            continue
        if r < safe_dist:
            return 0.0
        if r < slow_dist:
            vx = min(vx, cruise * (r - safe_dist) / (slow_dist - safe_dist))
    return max(0.0, vx)
