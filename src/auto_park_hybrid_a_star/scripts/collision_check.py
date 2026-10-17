#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""collision_check.py

车辆-障碍物碰撞检测。
车辆模型为旋转矩形（OBB），障碍物在本作业里是路边停放的车辆（轴对齐矩形 AABB）。
把本车四个角变换到障碍物坐标系，做 AABB 包含测试即可。
"""

import math


class VehicleFootprint:
    """车辆外形参数（米）"""
    def __init__(self, length=4.5, width=2.0, wheel_base=2.8):
        self.length = length          # 车长
        self.width = width            # 车宽
        self.wheel_base = wheel_base   # 轴距
        # 参考点（后轴中心）到车头/车尾的距离
        self.rear_overhang = (length - wheel_base) / 2.0
        self.front_overhang = self.rear_overhang
        # 最小转弯半径（后轴中心）
        self.min_turn_radius = wheel_base / math.tan(math.radians(35.0))


def vehicle_corners(x, y, yaw, vp):
    """返回本车四个角点（世界坐标）。后轴中心在 (x,y)，车头方向为 yaw。"""
    # 车体局部坐标：原点在后轴中心，x 轴朝前
    rear_x = -vp.rear_overhang
    front_x = vp.wheel_base + vp.front_overhang
    half_w = vp.width / 2.0
    local = [
        (rear_x, -half_w),
        (rear_x, half_w),
        (front_x, half_w),
        (front_x, -half_w),
    ]
    c, s = math.cos(yaw), math.sin(yaw)
    world = [(x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in local]
    return world


def point_in_aabb(px, py, obs):
    """obs: (cx, cy, w, h) 轴对齐矩形中心/宽/高"""
    cx, cy, w, h = obs
    return (cx - w / 2.0 <= px <= cx + w / 2.0 and
            cy - h / 2.0 <= py <= cy + h / 2.0)


def check_collision(x, y, yaw, vp, obstacles, inflation=0.15):
    """检查位姿 (x,y,yaw) 是否与任何障碍物碰撞。
    obstacles: [(cx, cy, w, h), ...]
    inflation: 安全膨胀（米），避免贴边太近
    """
    # 稍微膨胀车辆
    eff = VehicleFootprint(vp.length + inflation * 2,
                           vp.width + inflation * 2,
                           vp.wheel_base)
    corners = vehicle_corners(x, y, yaw, eff)
    for obs in obstacles:
        # 车辆任一角落在障碍矩形内 → 碰撞
        for (cx, cy) in corners:
            if point_in_aabb(cx, cy, obs):
                return True
        # 反过来：障碍矩形中心是否在本车 OBB 内（覆盖小障碍在车下的情况）
        ox, oy, ow, oh = obs
        # 把障碍角点变换到本车坐标系
        c, s = math.cos(-yaw), math.sin(-yaw)
        for dx in (-ow / 2, ow / 2):
            for dy in (-oh / 2, oh / 2):
                wx, wy = ox + dx, oy + dy
                lx = (wx - x) * c - (wy - y) * s
                ly = (wx - x) * s + (wy - y) * c
                if (-eff.rear_overhang <= lx <= eff.wheel_base + eff.front_overhang and
                        -eff.width / 2 <= ly <= eff.width / 2):
                    return True
    return False


if __name__ == "__main__":
    vp = VehicleFootprint()
    # 车在原点朝 x 轴，前方 5 米有障碍
    obs = [(5.0, 0.0, 2.0, 2.0)]
    print("clear:", not check_collision(0, 0, 0, vp, obs))       # 应 True
    print("hit:  ", check_collision(4.5, 0, 0, vp, obs))         # 应 True
