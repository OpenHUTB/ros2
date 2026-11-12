# -*- coding: utf-8 -*-
"""打靶弹道解算与成绩统计。

为什么单独抽出来
----------------
``main.py`` 里原来的发射逻辑是"固定飞行时间 0.45 s + 手写抛物线"，公式直接硬编码
在循环里：

* 想加侧风就得改公式，改错了也不容易发现；
* 想算成绩（命中率、平均几发一个靶）只能靠人眼看日志；
* 不打开 MuJoCo 窗口就没法跑，实验室机器 / 服务器上没法批量评测。

这个模块只做**纯数学**：给定起点、目标、飞行时间和恒定风加速度，解出初速度；
再提供一次成绩汇总。不 import mujoco，所以可以单独跑单元测试。

弹道模型
--------
设飞行时间 ``T``，起点 ``p0``、目标 ``p``、重力 ``g``、水平风加速度 ``a``：

    p = p0 + v·T + ½·a·T²        （水平，a 只有 x/y 分量）
    z = z0 + vz·T − ½·g·T²       （竖直）

反解：

    vx = (x − x0)/T − ½·ax·T
    vy = (y − y0)/T − ½·ay·T
    vz = (z − z0 + ½·g·T²)/T

风只影响水平分量；竖直方向由重力决定。风做**恒定加速度**处理（对应仿真里每步
给小球加 ``m·a`` 的外力），比"按速度偏移"更接近真实匀加速场。
"""

from __future__ import annotations

import math

__all__ = [
    "GRAVITY",
    "wrap_angle",
    "solve_launch_velocity",
    "predict_position",
    "required_speed",
    "score_shots",
]

#: 标准重力加速度
GRAVITY = 9.81


def wrap_angle(angle):
    """把角度归一化到 (-π, π]。"""
    value = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if value <= -math.pi else value


def solve_launch_velocity(start, target, flight_time,
                          gravity=GRAVITY, wind=(0.0, 0.0)):
    """给定飞行时间，解出让小球正好落在 target 的初速度。

    参数
    ----
    start       发射点 (x, y, z)，单位米
    target      目标点 (x, y, z)
    flight_time 期望飞行时间 T，必须为正
    gravity     重力加速度，默认 9.81
    wind        水平风加速度 (ax, ay)，单位 m/s²

    返回 ``(vx, vy, vz)``，单位 m/s。

    说明
    ----
    这里**固定飞行时间**求速度，而不是固定初速求仰角。原因是对固定炮塔的
    打靶场景来说，时间可控意味着落点误差可控：距离越远速度越快，但弹道
    形状（相对目标的抛物线）保持一致。
    """
    if flight_time <= 0:
        raise ValueError("flight_time 必须为正")
    if len(start) != 3 or len(target) != 3:
        raise ValueError("start / target 必须是三维坐标")
    ax, ay = float(wind[0]), float(wind[1])
    t = float(flight_time)
    vx = (target[0] - start[0]) / t - 0.5 * ax * t
    vy = (target[1] - start[1]) / t - 0.5 * ay * t
    vz = (target[2] - start[2] + 0.5 * gravity * t * t) / t
    return vx, vy, vz


def predict_position(start, velocity, t, gravity=GRAVITY, wind=(0.0, 0.0)):
    """按初速度积分 t 秒后的位置，用来验证解算结果。"""
    ax, ay = float(wind[0]), float(wind[1])
    x = start[0] + velocity[0] * t + 0.5 * ax * t * t
    y = start[1] + velocity[1] * t + 0.5 * ay * t * t
    z = start[2] + velocity[2] * t - 0.5 * gravity * t * t
    return x, y, z


def required_speed(distance, flight_time):
    """固定飞行时间下的平均速度（用于日志里显示"力度"）。"""
    if flight_time <= 0:
        raise ValueError("flight_time 必须为正")
    return float(distance) / float(flight_time)


def score_shots(shots, downed, duration, reason="", total_targets=6):
    """把逐发记录汇总成成绩。

    参数
    ----
    shots   逐发记录列表，每项是 dict，至少含 ``target``；命中时含 ``hit=True``
    downed  已击倒的靶数
    duration 任务耗时 (s)
    reason  结束原因（全部击倒 / 球用尽 / 超时）
    total_targets 靶子总数

    返回 dict：
        shots            总发数
        downed           击倒靶数
        hit_rate         命中率 = 击倒数 / 发数（1.0 表示一发一个）
        first_shot_hits  只用一发就击倒的靶数
        avg_shots_per_down 平均每个靶用几发（没有击倒时为 None）
        duration         耗时
        finished        是否全部击倒
        reason           结束原因
    """
    shots = list(shots or [])
    fired = len(shots)
    per_target = [[] for _ in range(total_targets)]
    for shot in shots:
        target = shot.get("target")
        if isinstance(target, int) and 0 <= target < total_targets:
            per_target[target].append(shot)

    # "一发一个靶"：这个靶只打了一发，而且这一发就打倒了
    first_shot_hits = sum(1 for group in per_target
                          if len(group) == 1 and group[0].get("hit"))

    downed = int(downed)
    return {
        "shots": fired,
        "downed": downed,
        "hit_rate": (downed / float(fired)) if fired else 0.0,
        "first_shot_hits": first_shot_hits,
        "avg_shots_per_down": (fired / float(downed)) if downed else None,
        "duration": float(duration),
        "finished": downed >= total_targets,
        "reason": reason,
    }
