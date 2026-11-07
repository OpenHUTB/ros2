#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯追踪（pure pursuit）控制核心，输入里程计位姿与全局路径，输出速度指令。

只使用 /odom 提供的位姿——控制器不读取 Gazebo 真值，符合真实机器人上
只能依赖轮式里程计的情形。纯几何计算，便于离线单元测试。
"""

import math


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def nearest_index(points, pose, start_index=0):
    """在路径上找距离当前位姿最近的点，并限制搜索从 start_index 开始（进度单调）。"""
    x, y = pose[0], pose[1]
    best_i, best_d = start_index, float("inf")
    for i in range(start_index, len(points)):
        d = (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2
        if d < best_d:
            best_i, best_d = i, d
    return best_i, math.sqrt(best_d)


def lookahead_point(points, start_index, distance):
    """沿路径向前走 distance 米，返回目标点与所在索引（终点不足时取最后一点）。"""
    remaining = distance
    idx = start_index
    while idx + 1 < len(points):
        x0, y0 = points[idx]
        x1, y1 = points[idx + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 1e-9:
            idx += 1
            continue
        if remaining <= seg:
            t = remaining / seg
            return (x0 + t * (x1 - x0), y0 + t * (y1 - y0)), idx
        remaining -= seg
        idx += 1
    return points[-1], len(points) - 1


def pure_pursuit_step(path, pose, start_index, lookahead=0.55,
                      v_max=0.75, v_min=0.18, w_max=1.4,
                      goal_tolerance=0.12):
    """执行一次纯追踪。

    参数：
      path  全局路径点列 [(x, y), ...]（map 坐标系，米）
      pose  (x, y, yaw)，来自里程计
      start_index 上一次的最近点索引，保证沿路径单调前进
    返回 dict：
      v, w          线速度/角速度指令
      target        前视点
      index         前视点索引
      nearest       最近点索引
      distance      到终点的欧氏距离
      reached       是否到达目标容差
    """
    if len(path) < 2:
        return {"v": 0.0, "w": 0.0, "target": pose[:2], "index": 0,
                "nearest": 0, "distance": 0.0, "reached": True}

    x, y, yaw = pose
    nearest, _ = nearest_index(path, pose, start_index)
    # 前视距离随最近点剩余距离缩短，接近终点时自然减速
    dist_goal = math.hypot(path[-1][0] - x, path[-1][1] - y)
    look = min(lookahead, max(goal_tolerance, dist_goal + 0.05))
    target, target_index = lookahead_point(path, nearest, look)

    dx, dy = target[0] - x, target[1] - y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    alpha = math.atan2(local_y, local_x)

    if dist_goal <= goal_tolerance:
        return {"v": 0.0, "w": 0.0, "target": target, "index": target_index,
                "nearest": nearest, "distance": dist_goal, "reached": True}

    # 转角越大越慢；靠近终点也减速，避免冲过头
    v = v_max * max(0.25, 1.0 - abs(alpha) / 1.6)
    v = min(v, max(v_min, dist_goal * 1.2))
    # 纯追踪曲率 kappa = 2 * y_local / L^2，w = v * kappa
    look_eff = max(0.15, math.hypot(local_x, local_y))
    curvature = 2.0 * local_y / (look_eff * look_eff)
    w = v * curvature
    w = max(-w_max, min(w_max, w))

    return {"v": v, "w": w, "target": target, "index": target_index,
            "nearest": nearest, "distance": dist_goal, "reached": False}
