#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""时间窗约束检验：任务序列 → 逐点到达时刻 / 早到等待 / 晚到延误。

给定一架机的起点与任务序列，逐段前推：

1. **到达时刻** = 前序离开时刻 + 航段飞行时间（欧氏距离 / 巡航速度）；
   第一段的"前序"是起点，起飞时刻为 0；
2. 前序离开时刻 = max(到达, earliest) + service_time：早到悬停等待到
   ``earliest``，等待时长按 ``hover_weight`` 折算进成本；准点或晚到不等待；
3. 逐点判 ``early / on-time / late``：服务开始时刻相对窗口的位置，
   晚到延误 = max(0, 到达服务时刻 − latest)。

全部解析计算，无迭代、无随机，是分配算法与单元测试的共同基准。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

from .mission import Config, Task, Uav

EARLY, ON_TIME, LATE = "early", "on-time", "late"


@dataclass
class StopRecord:
    """一架机在单个任务点的时间窗检验结果。"""

    tid: str
    arrival: float          # 到达时刻（服务开始前）
    wait: float             # 早到悬停等待时长
    serve_start: float      # 服务开始时刻 = arrival + wait
    departure: float        # 服务结束时刻 = serve_start + service_time
    window: Tuple[float, float]
    status: str             # early / on-time / late
    lateness: float         # 正延误 = max(0, serve_start − latest)

    @property
    def classify(self) -> str:
        return self.status

    def as_dict(self) -> dict:
        return dict(tid=self.tid, arrival=round(self.arrival, 2),
                    wait=round(self.wait, 2), serve_start=round(self.serve_start, 2),
                    departure=round(self.departure, 2),
                    window=[round(self.window[0], 2), round(self.window[1], 2)],
                    status=self.status, lateness=round(self.lateness, 2))


@dataclass
class UavSchedule:
    """一架机的完整时刻表。"""

    uid: str
    stops: List[StopRecord] = field(default_factory=list)

    @property
    def lateness_total(self) -> float:
        return sum(s.lateness for s in self.stops)

    @property
    def lateness_max(self) -> float:
        return max((s.lateness for s in self.stops), default=0.0)

    @property
    def wait_total(self) -> float:
        return sum(s.wait for s in self.stops)

    @property
    def flight_time(self) -> float:
        """纯飞行时间（不含等待与服务）。"""
        return sum(s.arrival - (self.stops[i - 1].departure if i else 0.0)
                   for i, s in enumerate(self.stops))

    @property
    def makespan(self) -> float:
        return self.stops[-1].departure if self.stops else 0.0

    def as_dict(self) -> dict:
        return dict(uid=self.uid,
                    stops=[s.as_dict() for s in self.stops],
                    lateness_total=round(self.lateness_total, 2),
                    lateness_max=round(self.lateness_max, 2),
                    wait_total=round(self.wait_total, 2),
                    flight_time=round(self.flight_time, 2),
                    makespan=round(self.makespan, 2))


def classify_arrival(arrival: float, task: Task) -> str:
    """按到达时刻相对时间窗判定：早到（需等待）/ 准点 / 晚到。

    早到悬停等待到 ``earliest`` 再开始服务，因此服务开始时刻不会早于
    窗口起点；早到与否只看**到达**时刻。
    """
    if arrival < task.earliest - 1e-9:
        return EARLY
    serve_start = max(arrival, task.earliest)
    if serve_start > task.latest + 1e-9:
        return LATE
    return ON_TIME


def schedule_uav(uav: Uav, tids: Sequence[str], tasks: Dict[str, Task],
                 cfg: Config) -> UavSchedule:
    """前推一架机按 ``tids`` 顺序执行任务的完整时刻表。

    到达时刻 = 前序离开时刻 + 欧氏距离 / 巡航速度；早到悬停等待到
    ``earliest`` 才开始服务，晚到照常服务但记正延误。
    """
    stops: List[StopRecord] = []
    px, py = uav.pos
    t = 0.0                                    # 起飞时刻
    for tid in tids:
        task = tasks[tid]
        flight = math.dist((px, py), task.pos) / uav.speed
        arrival = t + flight
        wait = max(0.0, task.earliest - arrival)
        serve_start = arrival + wait
        departure = serve_start + task.service_time
        stops.append(StopRecord(
            tid=tid, arrival=arrival, wait=wait, serve_start=serve_start,
            departure=departure,
            window=(task.earliest, task.latest),
            status=classify_arrival(arrival, task),
            lateness=max(0.0, serve_start - task.latest)))
        px, py = task.pos
        t = departure
    return UavSchedule(uid=uav.uid, stops=stops)


def uav_cost(schedule: UavSchedule, cfg: Config) -> float:
    """单机成本 = 正延误 + hover_weight × 悬停等待（耗能折算）。"""
    return schedule.lateness_total + cfg.hover_weight * schedule.wait_total


def objective(schedules: Dict[str, UavSchedule], cfg: Config) -> float:
    """机队目标函数 = Σ正延误 + λ×(最大单机延误 − 最小单机延误)。

    第二项是 max-min 公平项：只有当所有机都有任务（min 有定义）时才计，
    无任务的机延误为 0。悬停等待只进单机成本（能耗），不进机队延误和。
    """
    delays = [s.lateness_total for s in schedules.values()]
    spread = (max(delays) - min(delays)) if schedules else 0.0
    return sum(delays) + cfg.lambda_balance * spread


def fleet_report(schedules: Dict[str, UavSchedule], cfg: Config) -> dict:
    """机队级指标汇总（结果 JSON 与 README 共用同一来源）。"""
    delays = [s.lateness_total for s in schedules.values()]
    waits = [s.wait_total for s in schedules.values()]
    late_stops = [s for sch in schedules.values() for s in sch.stops
                  if s.status == LATE]
    early_stops = [s for sch in schedules.values() for s in sch.stops
                   if s.status == EARLY]
    on_time = [s for sch in schedules.values() for s in sch.stops
               if s.status == ON_TIME]
    return dict(
        delay_total=round(sum(delays), 2),
        delay_max_uav=round(max(delays) if delays else 0.0, 2),
        delay_min_uav=round(min(delays) if delays else 0.0, 2),
        delay_spread=round(max(delays) - min(delays) if delays else 0.0, 2),
        objective=round(objective(schedules, cfg), 2),
        hover_total=round(sum(waits), 2),
        n_early=len(early_stops),
        n_on_time=len(on_time),
        n_late=len(late_stops),
    )
