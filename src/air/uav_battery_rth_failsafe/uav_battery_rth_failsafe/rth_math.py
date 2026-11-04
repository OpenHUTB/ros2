#!/usr/bin/env python3
"""电池 RTH 纯计算函数 (不依赖 ROS)."""

ST_NORMAL = "NORMAL"
ST_WARNING = "WARNING"
ST_RETURN_HOME = "RETURN_HOME"
ST_EMERGENCY_LAND = "EMERGENCY_LAND"


def decide_state(battery, warn_pct, rth_pct, emer_pct,
                 dist_home, power_per_m, emer_margin=3.0):
    """根据电量、距离和耗电系数决定故障状态。纯函数, 便于单测。"""
    needed = dist_home * power_per_m
    if battery <= emer_pct:
        return ST_EMERGENCY_LAND
    if battery <= rth_pct:
        return ST_RETURN_HOME
    if battery <= warn_pct:
        if needed > battery - emer_margin:
            return ST_RETURN_HOME
        return ST_WARNING
    return ST_NORMAL


def dist_3d(p, q):
    return ((p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2) ** 0.5
