# -*- coding: utf-8 -*-
"""评价指标：让"跑通了"变成可复核的数字。

课程评分要求"给出计算原理公式、算法流程、源码解析"，指标必须**有定义**。
这里每个函数都对应一个在文档里给出公式的量：

=====================  ==============================  ==========================
指标                    符号                             定义
=====================  ==============================  ==========================
路径误差                RMSE_p                          实际轨迹到参考路径的距离均方根
航向误差                MAE_ψ                           轨迹跟踪时的航向偏差绝对值均值
平均/最大速度           v̄ / v_max                       躯干水平速度的均值与峰值
关节平滑度              AoS                             相邻控制周期关节目标角变化量均值
能量                    E                               关节力矩×关节速度的时间积分
到达与否                success                         进入目标容差半径且未摔倒
=====================  ==============================  ==========================
"""

from __future__ import annotations

import math


def rms(values) -> float:
    """均方根：``sqrt(mean(v^2))``。"""
    values = list(values)
    if not values:
        return 0.0
    return math.sqrt(sum(float(v) ** 2 for v in values) / len(values))


def mean(values) -> float:
    values = list(values)
    if not values:
        return 0.0
    return sum(float(v) for v in values) / len(values)


def max_abs(values) -> float:
    values = list(values)
    if not values:
        return 0.0
    return max(abs(float(v)) for v in values)


def point_to_polyline_distance(px, py, points) -> float:
    """点到折线的最短距离（米）。

    必须算"到线段的垂距"而不是"到最近顶点的距离"：路点稀疏时，
    顶点距离会在两个路点之间虚高到半个路段长，把误差报警淹没。
    """
    pts = list(points)
    if not pts:
        return float("inf")
    if len(pts) == 1:
        return math.hypot(px - pts[0][0], py - pts[0][1])

    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 <= 1e-12:
            d = math.hypot(px - ax, py - ay)
        else:
            t = ((px - ax) * dx + (py - ay) * dy) / seg2
            t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        if d < best:
            best = d
    return best


def action_smoothness(target_history) -> float:
    """关节平滑度 AoS：相邻控制周期目标角变化量的绝对值均值（弧度/周期）。

    这是"指令抖动"的度量，与"实际关节抖动"分开统计：目标角抖动说明
    轨迹生成有问题，实际抖动说明伺服/接触有问题，两者的排查方向不同。
    """
    seq = list(target_history)
    if len(seq) < 2:
        return 0.0
    total = 0.0
    count = 0
    for prev, cur in zip(seq, seq[1:]):
        n = min(len(prev), len(cur))
        for k in range(n):
            total += abs(float(cur[k]) - float(prev[k]))
            count += 1
    return total / count if count else 0.0


def energy(torques, velocities, dt) -> float:
    """关节耗散能量 ``E = Σ |τ·q̇|·dt``（焦耳）。"""
    if len(torques) != len(velocities):
        raise ValueError("力矩与速度长度必须一致")
    return sum(abs(float(t) * float(v)) for t, v in zip(torques, velocities)) * float(dt)


def summarize(samples) -> dict:
    """把一帧帧的采样字典汇总成一份指标表。

    每个采样至少包含 ``t``、``x``、``y``、``yaw``、``speed``；
    可选 ``path_error``、``heading_error``、``targets``、``energy``。
    """
    samples = list(samples)
    if not samples:
        return {}

    speeds = [s.get("speed", 0.0) for s in samples]
    path_errors = [s["path_error"] for s in samples if "path_error" in s]
    heading_errors = [s["heading_error"] for s in samples if "heading_error" in s]
    targets = [s["targets"] for s in samples if "targets" in s]
    energies = [s.get("energy", 0.0) for s in samples]

    duration = float(samples[-1]["t"]) - float(samples[0]["t"])
    return {
        "duration_s": round(max(0.0, duration), 3),
        "samples": len(samples),
        "speed_mean": round(mean(speeds), 4),
        "speed_max": round(max_abs(speeds), 4),
        "path_rmse": round(rms(path_errors), 4) if path_errors else None,
        "heading_mae": round(mean([abs(h) for h in heading_errors]), 4) if heading_errors else None,
        "aos": round(action_smoothness(targets), 6) if targets else None,
        "energy_j": round(sum(energies), 4),
        "final_xy": (round(float(samples[-1]["x"]), 3), round(float(samples[-1]["y"]), 3)),
    }
