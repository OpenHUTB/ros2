#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""问题数据模型：无人机、任务点、场景与配置。

所有长度单位 m、时间单位 s、坐标 ENU（x 东、y 北）。一架 UAV 从自己的
起点出发，按分配到的任务序列依次飞行：到达任务点 → 悬停等待（若早到）
→ 服务 ``service_time`` → 飞往下一个点。每个任务点有时间窗
``[earliest, latest]``：服务**开始**时刻落在窗内即算不违窗，早到需悬停
等待（耗能折算进成本），晚到计入正延误。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Config:
    """算法参数（与 config/uav_time_window.yaml 一一对应）。"""

    lambda_balance: float = 0.5     # 机间均衡项权重 λ
    default_speed: float = 8.0      # m/s
    default_endurance: float = 420.0  # s（续航，按巡航折算）
    service_time: float = 15.0      # s
    hover_weight: float = 0.2       # 悬停 1 s 折算 0.2 s 等效延误
    max_improve_rounds: int = 200
    alt: float = 30.0
    climb_speed: float = 2.5

    @classmethod
    def from_mapping(cls, data: Dict) -> "Config":
        keys = set(cls.__dataclass_fields__)
        return cls(**{k: v for k, v in data.items() if k in keys})


@dataclass(frozen=True)
class Uav:
    """一架无人机：起点位置 + 巡航速度 + 续航。"""

    uid: str
    x: float
    y: float
    speed: float = 8.0
    endurance: float = 420.0        # s，最大飞行时间（不含悬停）
    label: str = ""

    @property
    def pos(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def range_m(self) -> float:
        """续航折算的最大航程（m）。"""
        return self.endurance * self.speed


@dataclass(frozen=True)
class Task:
    """一个任务点：位置 + 时间窗 [earliest, latest] + 服务时长 + 优先级。"""

    tid: str
    x: float
    y: float
    earliest: float                 # 服务开始不得早于
    latest: float                   # 服务开始不得晚于（晚到计延误）
    service_time: float = 15.0
    priority: int = 0               # 数值越大越紧急（urgent 场景用）

    @property
    def pos(self) -> Tuple[float, float]:
        return (self.x, self.y)

    @property
    def window_width(self) -> float:
        return self.latest - self.earliest


@dataclass
class Scenario:
    """一个可复现的运行场景。"""

    name: str
    title: str
    description: str
    uavs: List[Uav]
    tasks: List[Task]
    config: Config = field(default_factory=Config)


# --------------------------------------------------------------------------- #
# 可行性预检：窗口不可达的任务标记并报告原因，不硬塞
# --------------------------------------------------------------------------- #
def unreachable_reason(task: Task, uav: Uav) -> Optional[str]:
    """判断该任务对该机是否物理不可达；返回原因或 ``None``。

    两条硬判据（与领域要求一致）：

    1. **距离过远**——起点到任务点的直线航程已超出该机续航折算航程，
       后续无论怎么排都飞不到；
    2. **窗口过窄**——时间窗宽度小于服务时长，即使准点到达也无法在
       窗口内完成服务。
    """
    if math.dist(task.pos, uav.pos) > uav.range_m() + 1e-9:
        return "距离过远（%.0f m > 续航 %.0f m）" % (
            math.dist(task.pos, uav.pos), uav.range_m())
    if task.window_width < task.service_time - 1e-9:
        return "窗口过窄（%.0f s < 服务时长 %.0f s）" % (
            task.window_width, task.service_time)
    return None


def screen_tasks(scenario: Scenario) -> List[Dict]:
    """对全场景做可行性预检。

    一旦**所有** UAV 都飞不到某任务，或窗口本身过窄，该任务即被标记为
    ``infeasible``，分配器不再把它塞给任何一架机。返回逐任务的诊断行。
    """
    rows = []
    for task in scenario.tasks:
        reasons = [r for r in
                   (unreachable_reason(task, u) for u in scenario.uavs) if r]
        all_far = all(
            math.dist(task.pos, u.pos) > u.range_m() + 1e-9
            for u in scenario.uavs)
        too_narrow = task.window_width < task.service_time - 1e-9
        infeasible = all_far or too_narrow
        rows.append(dict(
            tid=task.tid,
            infeasible=infeasible,
            reason="；".join(sorted(set(reasons))) if infeasible else "",
            reachable_by=[u.uid for u in scenario.uavs
                          if unreachable_reason(task, u) is None],
        ))
    return rows
