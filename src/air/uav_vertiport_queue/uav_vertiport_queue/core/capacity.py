#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""容量评估：固定请求率扫描 → 吞吐量 / 平均等待 / 最大队列长度 / 饱和点。

方法：给定请求率 ``rate``（架次/小时），在 1 小时评估窗口内生成**等间隔**
的确定性请求流（离场/进场交替，无随机性，保证可复现），用 ``interval``
策略排队，统计：

- ``throughput``   评估窗口内实际排定运行的架次/小时（积压不计入）；
- ``avg_wait``     全体架次的平均等待（只计正值延误）；
- ``max_queue``    排队长度峰值；
- ``backlog``      窗口结束时仍未排上（排到窗口外）的架次。

饱和点：平均等待首次超过 ``wait_threshold``（默认 60 s）之前最大的请求率。
低于饱和点时请求间隔均不小于固定间隔，平均等待为 0。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from .queue import (ARRIVAL, DEPARTURE, QueueConfig, Request, schedule,
                    schedule_metrics)

#: 默认扫描的请求率（架次/小时）：覆盖题目要求的 12~36 档，
#: 并向上扩展到容量上限以外以暴露饱和点
DEFAULT_RATES: Sequence[int] = (12, 16, 20, 24, 28, 32, 36, 40,
                                48, 56, 64, 72, 80, 88)
DEFAULT_HORIZON = 3600.0    # 评估窗口（s）
DEFAULT_WAIT_THRESHOLD = 60.0


def generate_demand(rate_per_hour: float,
                    horizon: float = DEFAULT_HORIZON) -> List[Request]:
    """固定请求率的确定性请求流：等间隔、离场/进场交替。"""
    rate = int(round(rate_per_hour))
    if rate <= 0:
        return []
    headway = horizon / rate
    out: List[Request] = []
    for i in range(rate):
        t = i * headway
        kind = DEPARTURE if i % 2 == 0 else ARRIVAL
        out.append(Request(uid="REQ-%03d" % (i + 1), kind=kind, t_request=t))
    return out


def evaluate_rate(rate_per_hour: float, cfg: QueueConfig,
                  strategy: str = "interval",
                  horizon: float = DEFAULT_HORIZON) -> dict:
    """评估单个请求率下的排队表现。"""
    requests = generate_demand(rate_per_hour, horizon)
    sched = schedule(requests, cfg, strategy)
    a = sched.assignments
    metrics = schedule_metrics(a, cfg, len(requests))
    served = sum(1 for x in a if x.t_scheduled <= horizon)
    return dict(
        rate=rate_per_hour,
        requested=len(requests),
        throughput=round(served * 3600.0 / horizon, 2),
        backlog=len(requests) - served,
        avg_wait=metrics["avg_wait"],
        max_wait=metrics["max_wait"],
        max_queue_length=metrics["max_queue_length"],
        pad_conflicts=metrics["pad_conflicts"],
        dep_interval_violations=metrics["dep_interval_violations"],
        arr_interval_violations=metrics["arr_interval_violations"],
    )


def capacity_scan(rates: Sequence[float] = DEFAULT_RATES,
                  cfg: Optional[QueueConfig] = None,
                  strategy: str = "interval",
                  horizon: float = DEFAULT_HORIZON) -> List[dict]:
    """请求率扫描，返回按请求率升序的指标列表（画容量曲线用）。"""
    cfg = cfg or QueueConfig()
    rows = [evaluate_rate(r, cfg, strategy, horizon) for r in rates]
    rows.sort(key=lambda d: d["rate"])
    return rows


def saturation_point(scan: Sequence[dict],
                     wait_threshold: float = DEFAULT_WAIT_THRESHOLD) -> dict:
    """饱和点 = 平均等待仍不超过阈值的最大请求率。

    若最低请求率即饱和返回 ``{"rate": 0, "avg_wait": 0}``；若全部未饱和
    则 ``saturated=True`` 提示需要向更高请求率扩展扫描。
    """
    ok = [row for row in scan if row["avg_wait"] <= wait_threshold]
    if not ok:
        return dict(rate=0.0, avg_wait=0.0, saturated=True, note="最低请求率即饱和")
    best = max(ok, key=lambda row: row["rate"])
    saturated = best["rate"] < max(row["rate"] for row in scan)
    out = dict(rate=best["rate"], avg_wait=best["avg_wait"],
               saturated=saturated)
    if saturated:
        over = min(row["rate"] for row in scan if row["avg_wait"] > wait_threshold)
        out["first_saturated_rate"] = over
    return out


def apron_occupancy(assignments: Sequence[dict],
                    flight_time: float = 600.0) -> int:
    """停机坪峰值占用（架）：已进场但尚未再次离场的机体数。

    ``assignments`` 为 ``{"uid", "kind", "t"}`` 字典列表（t 为运行时刻）。
    进场接地后机体停入场内，经过 ``flight_time`` 后视为再次离场放行。
    只在请求率扫描之外的场景评估里使用（info 指标）。
    """
    events: List[tuple] = []
    for a in assignments:
        if a["kind"] == ARRIVAL:
            events.append((a["t"], 1))
        else:
            events.append((a["t"] + flight_time, -1))
    events.sort()
    cur = best = 0
    for _t, d in events:
        cur += d
        best = max(best, cur)
    return best
