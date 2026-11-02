# -*- coding: utf-8 -*-
"""交通信号灯响应逻辑（纯 Python，不依赖 CARLA）。

这一层把"看到什么灯、离停止线多远、现在多快"映射成"该怎么做"，与 CARLA
完全解耦，因此可以单独写单元测试。

三个动作
--------
GO        正常通过
SLOW      减速（黄灯且还能停住）
STOP      制动停车（红灯，或黄灯已停不住/已过线）

黄灯的处理
----------
黄灯不是简单的"减速"，要看**当前速度下能不能在停止线前停住**：

    制动距离 d_brake = v² / (2·a_max)

* d_brake ≤ 距停止线距离  → 能停，减速停车
* 否则                    → 停不住，判断为"冲过去更安全"，保持通过

这也是真实 AEB / 信号灯辅助的通行做法：强行急刹反而更容易被追尾。

停车曲线
--------
要正好停在停止线前，任意时刻的目标速度应满足

    v_target = sqrt(2 · a_comfort · d)

其中 d 是剩余距离，a_comfort 是舒适减速度。这样速度会随距离平方根下降，
到停止线时自然归零，不会冲过头。
"""

from __future__ import annotations

import math

__all__ = [
    "RED", "YELLOW", "GREEN", "OFF",
    "GO", "SLOW", "STOP",
    "LightParams",
    "braking_distance",
    "required_deceleration",
    "target_speed_for_stop",
    "decide",
]

# 灯色（与 CARLA 的 TrafficLightState 命名对齐）
RED = "Red"
YELLOW = "Yellow"
GREEN = "Green"
OFF = "Off"

# 动作
GO = "GO"
SLOW = "SLOW"
STOP = "STOP"


class LightParams(object):
    """一套信号灯响应参数。

    comfort_decel    舒适减速度 (m/s²)，用于停车曲线，通常 1.5~2.5
    max_decel        最大减速度 (m/s²)，用于判断"停不停得住"，通常 6~8
    yellow_min_gap   黄灯时距停止线的最小可停距离 (m)，低于它一律通过
    stop_margin      目标停车点相对停止线的前移量 (m)，留一点余量
    min_speed        低于这个速度就直接刹停 (m/s)
    """

    def __init__(self, comfort_decel=2.0, max_decel=6.0,
                 yellow_min_gap=1.0, stop_margin=0.5, min_speed=0.3):
        if comfort_decel <= 0:
            raise ValueError("comfort_decel 必须为正")
        if max_decel <= 0:
            raise ValueError("max_decel 必须为正")
        if max_decel < comfort_decel:
            raise ValueError("max_decel 不应小于 comfort_decel")
        if yellow_min_gap < 0:
            raise ValueError("yellow_min_gap 不能为负")
        if stop_margin < 0:
            raise ValueError("stop_margin 不能为负")
        if min_speed < 0:
            raise ValueError("min_speed 不能为负")

        self.comfort_decel = float(comfort_decel)
        self.max_decel = float(max_decel)
        self.yellow_min_gap = float(yellow_min_gap)
        self.stop_margin = float(stop_margin)
        self.min_speed = float(min_speed)

    def __repr__(self):
        return ("LightParams(comfort=%.2f, max=%.2f, margin=%.2f)"
                % (self.comfort_decel, self.max_decel, self.stop_margin))


def braking_distance(v, a):
    """从速度 v 以减速度 a 刹到 0 所需距离：d = v² / (2a)。"""
    if a <= 0:
        raise ValueError("减速度必须为正")
    v = abs(float(v))
    return v * v / (2.0 * a)


def required_deceleration(v, d):
    """要在距离 d 内停下所需的减速度：a = v² / (2d)。

    d ≤ 0 时返回无穷大（已经过线）。
    """
    v = abs(float(v))
    d = float(d)
    if d <= 0.0:
        return float("inf")
    return v * v / (2.0 * d)


def target_speed_for_stop(d, a_comfort):
    """要正好停在距停止线 d 处，当前允许的最大速度：v = sqrt(2ad)。"""
    if a_comfort <= 0:
        raise ValueError("a_comfort 必须为正")
    d = float(d)
    if d <= 0.0:
        return 0.0
    return math.sqrt(2.0 * a_comfort * d)


def decide(state, distance, speed, params=None, passed_stop_line=False):
    """根据灯色、距停止线距离、当前速度给出动作。

    参数
    ----
    state            灯色，RED / YELLOW / GREEN / OFF
    distance         车辆到停止线的前方距离 (m)，负数表示已过线
    speed            当前速度 (m/s)
    params           LightParams，不传用默认
    passed_stop_line 是否已越过停止线（越线后不再因红灯停车）

    返回
    ----
    (action, target_speed, note)

    * action       GO / SLOW / STOP
    * target_speed 建议的目标速度 (m/s)，GO 时为 None 表示不限制
    * note         简短原因，便于日志输出
    """
    if state not in (RED, YELLOW, GREEN, OFF):
        raise ValueError("未知灯色: %r" % (state,))
    if params is None:
        params = LightParams()
    # 距离正好为 0 表示车头已经停在线上，仍应视为"未过线"；
    # 只有严格为负（越过停止线）才算过线。
    if distance < 0 and not passed_stop_line:
        passed_stop_line = True

    speed = max(0.0, float(speed))
    gap = max(0.0, float(distance))

    # 灯不亮或绿灯：正常通过
    if state in (GREEN, OFF):
        return GO, None, "%s 通过" % state

    # 已过线：不再停
    if passed_stop_line:
        return GO, None, "已越过停止线"

    # 红灯：必定停车
    if state == RED:
        if gap <= params.stop_margin + 1e-9:
            return STOP, 0.0, "已到停止线，刹停"
        v_t = target_speed_for_stop(gap - params.stop_margin, params.comfort_decel)
        enough = braking_distance(speed, params.max_decel) <= gap
        if not enough:
            return STOP, 0.0, "红灯，距离 %.2f m 内停不住，全力制动" % gap
        return STOP, v_t, "红灯，按停车曲线减速到 %.2f m/s" % v_t

    # 黄灯：看能不能停住
    d_brake = braking_distance(speed, params.max_decel)
    # 速度已经很低就别管距离了，直接停——慢慢蹭到线前停下总比越线安全
    if speed < params.min_speed:
        return STOP, 0.0, "黄灯，速度已很低，停车"
    # 距离太近且还有速度：停不住，通过更安全
    if gap < params.yellow_min_gap:
        return GO, None, "黄灯，距线仅 %.2f m，通过" % gap
    if d_brake <= gap:
        v_t = target_speed_for_stop(gap - params.stop_margin, params.comfort_decel)
        return SLOW, v_t, ("黄灯，%.2f m 内可停住（需 %.2f m），减速"
                           % (gap, d_brake))
    return GO, None, ("黄灯，停住需 %.2f m 但只剩 %.2f m，保持通过"
                      % (d_brake, gap))
