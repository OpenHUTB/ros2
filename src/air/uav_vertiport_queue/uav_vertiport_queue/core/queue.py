#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""低空起降场（vertiport）进离场排队：请求、起降位占用与三种排队策略。

领域模型
--------
一个 vertiport 拥有固定数量的起飞位与降落位（默认各 2 个）。每架无人机的
一次运行（架次）表现为一个**请求**：离场（departure，占用起飞位）或进场
（arrival，占用降落位），并带有计划时刻与任务优先级（医疗/警用 = 10，
普通 = 0）。

冲突判定（题目硬性定义）：**同一起降位的占用时间区间发生重叠即冲突**。
占用区间为 ``[时刻, 时刻 + 占用时长)``。

三种排队策略（判定逻辑全部在本模块，无 ROS 依赖，可直接单元测试）：

1. ``fifo``       基线：按请求时刻先后依次分配起降位，位忙则等待；
2. ``priority``   优先级抢占：优先级高的请求先排队（可插队），同优先级
                   内仍按请求时刻；
3. ``interval``   固定间隔：在 FIFO 基础上，同一机型流（进/离场分别）
                   相邻两次运行之间强制最小间隔（离场 60 s、进场 90 s，
                   可配置）。

所有策略的延误都只计正值：``delay = max(0, 排定时刻 - 请求时刻)``。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

DEPARTURE = "departure"
ARRIVAL = "arrival"
KINDS = (DEPARTURE, ARRIVAL)

#: 起降位名称前缀（报告与 JSON 里显示 T1/T2…、L1/L2…）
PAD_PREFIX = {DEPARTURE: "T", ARRIVAL: "L"}


# --------------------------------------------------------------------------- #
# 数据模型
# --------------------------------------------------------------------------- #
@dataclass
class Request:
    """一次架次请求。"""

    uid: str
    kind: str                      # DEPARTURE / ARRIVAL
    t_request: float               # 计划时刻（s）：离场=离地，进场=接地
    priority: int = 0              # 越大越优先（医疗/警用 = 10）
    label: str = ""

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise ValueError("kind 必须是 %s：%r" % (KINDS, self.kind))
        if self.t_request < 0.0:
            raise ValueError("计划时刻不能为负: %r" % (self.t_request,))

    @property
    def key(self) -> Tuple[str, str]:
        return (self.uid, self.kind)


@dataclass
class QueueConfig:
    """起降场资源配置（与 config/uav_vertiport_queue.yaml 对应）。"""

    pads_departure: int = 2
    pads_arrival: int = 2
    occupancy_departure: float = 90.0   # 起飞位单次占用时长（s）
    occupancy_arrival: float = 90.0     # 降落位单次占用时长（s）
    min_interval_departure: float = 60.0   # 相邻两次离场最小间隔（s）
    min_interval_arrival: float = 90.0     # 相邻两次进场最小间隔（s）

    def pads(self, kind: str) -> int:
        return self.pads_departure if kind == DEPARTURE else self.pads_arrival

    def occupancy(self, kind: str) -> float:
        return (self.occupancy_departure if kind == DEPARTURE
                else self.occupancy_arrival)

    def min_interval(self, kind: str) -> float:
        return (self.min_interval_departure if kind == DEPARTURE
                else self.min_interval_arrival)


@dataclass
class Assignment:
    """排队结果：一个请求被排到某个起降位的某个时刻。"""

    uid: str
    kind: str
    pad: str
    t_request: float
    t_scheduled: float
    priority: int = 0

    @property
    def delay(self) -> float:
        """延误只计正值。"""
        return max(0.0, self.t_scheduled - self.t_request)

    def busy_until(self, cfg: QueueConfig) -> float:
        return self.t_scheduled + cfg.occupancy(self.kind)

    def as_dict(self) -> dict:
        return dict(uid=self.uid, kind=self.kind, pad=self.pad,
                    t_request=round(self.t_request, 2),
                    t_scheduled=round(self.t_scheduled, 2),
                    wait=round(self.delay, 2), priority=self.priority)


@dataclass
class Schedule:
    """一次排队的完整输出。"""

    strategy: str
    assignments: List[Assignment] = field(default_factory=list)

    def by_uid_kind(self) -> Dict[Tuple[str, str], Assignment]:
        return {(a.uid, a.kind): a for a in self.assignments}

    def of_kind(self, kind: str) -> List[Assignment]:
        return sorted((a for a in self.assignments if a.kind == kind),
                      key=lambda a: a.t_scheduled)


STRATEGIES = ("fifo", "priority", "interval")


# --------------------------------------------------------------------------- #
# 排队核心
# --------------------------------------------------------------------------- #
def _order(requests: Sequence[Request], strategy: str) -> List[Request]:
    if strategy == "priority":
        return sorted(requests, key=lambda r: (-r.priority, r.t_request,
                                               r.uid, r.kind))
    # fifo / interval 都按请求时刻先来后到
    return sorted(requests, key=lambda r: (r.t_request, r.uid, r.kind))


def schedule(requests: Sequence[Request], cfg: QueueConfig,
             strategy: str = "interval") -> Schedule:
    """把一组请求按策略排到起降位上。

    贪心逐个处理：对每个请求取同类型最早空闲的起降位，排定时刻不得早于
    其请求时刻；``interval`` 策略额外要求与上一次同类型运行至少相隔
    ``min_interval(kind)``。
    """
    if strategy not in STRATEGIES:
        raise ValueError("未知策略 %r，可选：%s" % (strategy, STRATEGIES))
    pad_free: Dict[str, List[float]] = {
        k: [0.0] * cfg.pads(k) for k in KINDS}
    last_time: Dict[str, Optional[float]] = {k: None for k in KINDS}
    out: List[Assignment] = []

    for r in _order(requests, strategy):
        t = float(r.t_request)
        # 固定间隔对 priority 同样生效：急救机可以插队，但进离场流本身
        # 仍必须满足最小间隔，否则排出来的序列在物理上不可执行
        if strategy in ("interval", "priority") and last_time[r.kind] is not None:
            t = max(t, last_time[r.kind] + cfg.min_interval(r.kind))
        free = pad_free[r.kind]
        i = min(range(len(free)), key=free.__getitem__)
        t = max(t, free[i])
        free[i] = t + cfg.occupancy(r.kind)
        if strategy in ("interval", "priority"):
            last_time[r.kind] = t
        out.append(Assignment(uid=r.uid, kind=r.kind,
                              pad="%s%d" % (PAD_PREFIX[r.kind], i + 1),
                              t_request=float(r.t_request), t_scheduled=t,
                              priority=r.priority))
    out.sort(key=lambda a: (a.t_scheduled, a.uid, a.kind))
    return Schedule(strategy=strategy, assignments=out)


def assign_pads_no_wait(requests: Sequence[Request],
                        cfg: QueueConfig) -> Schedule:
    """排队前基线：所有请求都按计划时刻运行（不排队），起降位按请求顺序
    轮流分配。这正是"不做排队管理"时起降位被双重占用的来源。"""
    counters = {k: 0 for k in KINDS}
    out: List[Assignment] = []
    for r in sorted(requests, key=lambda r: (r.t_request, r.uid, r.kind)):
        i = counters[r.kind] % cfg.pads(r.kind)
        counters[r.kind] += 1
        out.append(Assignment(uid=r.uid, kind=r.kind,
                              pad="%s%d" % (PAD_PREFIX[r.kind], i + 1),
                              t_request=float(r.t_request),
                              t_scheduled=float(r.t_request),
                              priority=r.priority))
    out.sort(key=lambda a: (a.t_scheduled, a.uid, a.kind))
    return Schedule(strategy="no_wait", assignments=out)


# --------------------------------------------------------------------------- #
# 冲突判定：同一起降位占用区间重叠
# --------------------------------------------------------------------------- #
def find_conflicts(assignments: Sequence[Assignment],
                   cfg: QueueConfig) -> List[dict]:
    """返回所有"同一起降位占用区间重叠"的冲突对。

    每个冲突形如 ``{"pad", "a", "b", "overlap"}``，``overlap`` 为重叠秒数。
    """
    conflicts: List[dict] = []
    by_pad: Dict[str, List[Assignment]] = {}
    for a in assignments:
        by_pad.setdefault(a.pad, []).append(a)
    for pad, group in sorted(by_pad.items()):
        group = sorted(group, key=lambda a: a.t_scheduled)
        for i, a in enumerate(group):
            for b in group[i + 1:]:
                start = max(a.t_scheduled, b.t_scheduled)
                end = min(a.busy_until(cfg), b.busy_until(cfg))
                if end - start > 1e-9:
                    conflicts.append(dict(pad=pad, a=a.uid, b=b.uid,
                                          kind=a.kind,
                                          overlap=round(end - start, 2)))
    return conflicts


# --------------------------------------------------------------------------- #
# 指标
# --------------------------------------------------------------------------- #
def interval_stats(assignments: Sequence[Assignment],
                   kind: str) -> dict:
    """同类型相邻两次运行的实际时间间隔统计。"""
    times = sorted(a.t_scheduled for a in assignments if a.kind == kind)
    gaps = [b - a for a, b in zip(times[:-1], times[1:])]
    return dict(n=len(times),
                min_gap=(round(min(gaps), 2) if gaps else None),
                gaps=[round(g, 2) for g in gaps])


def max_queue_length(assignments: Sequence[Assignment]) -> int:
    """任一时刻"已请求但尚未排到"的最大架次（排队长度峰值）。"""
    events = sorted({a.t_request for a in assignments}
                    | {a.t_scheduled for a in assignments})
    best = 0
    for t in events:
        waiting = sum(1 for a in assignments
                      if a.t_request <= t < a.t_scheduled)
        best = max(best, waiting)
    return best


def schedule_metrics(assignments: Sequence[Assignment], cfg: QueueConfig,
                     requested: int) -> dict:
    """一张"排队前 vs 排队后"指标表的一行。"""
    waits = [a.delay for a in assignments]
    dep = interval_stats(assignments, DEPARTURE)
    arr = interval_stats(assignments, ARRIVAL)
    return dict(
        requested=requested,
        scheduled=len(assignments),
        delay_total=round(sum(waits), 2),
        avg_wait=round(sum(waits) / len(waits), 2) if waits else 0.0,
        max_wait=round(max(waits), 2) if waits else 0.0,
        delayed_ops=int(sum(1 for w in waits if w > 1e-9)),
        pad_conflicts=len(find_conflicts(assignments, cfg)),
        min_dep_interval=dep["min_gap"],
        min_arr_interval=arr["min_gap"],
        dep_interval_violations=(
            sum(1 for g in dep["gaps"]
                if g < cfg.min_interval(DEPARTURE) - 1e-6)),
        arr_interval_violations=(
            sum(1 for g in arr["gaps"]
                if g < cfg.min_interval(ARRIVAL) - 1e-6)),
        max_queue_length=max_queue_length(assignments),
    )


# --------------------------------------------------------------------------- #
# ROS 节点使用的纯逻辑载荷（节点是薄壳，本函数有单元测试）
# --------------------------------------------------------------------------- #
def build_queue_payload(requests: Sequence[Request], cfg: QueueConfig,
                        strategy: str = "interval",
                        now: Optional[float] = None) -> dict:
    """生成当前排队序列 + 预计等待时间（JSON 友好，std_msgs/String 发布）。"""
    sched = schedule(requests, cfg, strategy)
    metrics = schedule_metrics(sched.assignments, cfg, len(requests))
    queue = []
    for a in sorted(sched.assignments, key=lambda a: a.t_scheduled):
        queue.append(a.as_dict())
    payload = dict(strategy=strategy, queue=queue, summary=metrics)
    if now is not None:
        payload["now"] = round(float(now), 2)
    return payload
