#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通信链路损伤模型与接收端链路质量估计（纯 numpy，无 ROS 依赖）。

链路损伤用确定性种子建模，包含三类可独立开关的机制：

* 单向传输延迟 ``one_way_delay`` + 高斯抖动 ``jitter_std``；
* Gilbert–Elliott 两态丢包：好态偶发丢包、坏态连续丢包，用来产生成片丢包而不是均匀丢包；
* 计划内中断窗口 ``outages``，模拟遮挡、超视距、基站切换导致的完全失联。

发送端按固定频率打点，接收端按到达时间顺序收到消息。质量估计只看接收端可见的量
（到达间隔、命令年龄、窗口内投递率），不读取真实丢包标签。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class LinkProfile:
    """一条链路的损伤配置。"""

    name: str
    label: str
    one_way_delay: float = 0.0       # s
    jitter_std: float = 0.0          # s，截断到非负
    loss_good: float = 0.0           # 好态逐包丢包率
    loss_bad: float = 0.0            # 坏态逐包丢包率
    p_good_to_bad: float = 0.0       # 好→坏 逐包转移概率
    p_bad_to_good: float = 0.0       # 坏→好 逐包转移概率
    outages: Tuple[Tuple[float, float], ...] = ()   # [(t_start, t_end), ...]

    def in_outage(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.outages)


@dataclass
class Delivery:
    """一条消息的发送/到达记录；``arrival`` 为 None 表示未送达。"""

    send_t: float
    arrival: float | None

    @property
    def delivered(self) -> bool:
        return self.arrival is not None


class LinkEmulator:
    """按链路配置生成发送→到达的时间序列。

    每个方向（上行命令/下行遥测）用独立实例、独立种子，避免两个方向同步丢包
    产生现实中不存在的“同时断”规律。
    """

    def __init__(self, profile: LinkProfile, seed: int = 0):
        self.profile = profile
        self.rng = np.random.default_rng(seed)
        self._bad = False

    def _sample_delay(self) -> float:
        d = self.profile.one_way_delay
        if self.profile.jitter_std > 0:
            d += max(0.0, float(self.rng.normal(0.0, self.profile.jitter_std)))
        return d

    def _drop(self) -> bool:
        p = self.profile
        if self._bad:
            drop = self.rng.random() < p.loss_bad
            if self.rng.random() < p.p_bad_to_good:
                self._bad = False
            return drop
        drop = self.rng.random() < p.loss_good
        if self.rng.random() < p.p_good_to_bad:
            self._bad = True
        return drop

    def transmit(self, send_t: float) -> Delivery:
        if self.profile.in_outage(send_t):
            return Delivery(send_t, None)
        if self._drop():
            return Delivery(send_t, None)
        return Delivery(send_t, send_t + self._sample_delay())

    def schedule(self, send_times: Sequence[float]) -> List[Delivery]:
        """批量生成，按到达时间排序（允许后发先至）。"""
        out = [self.transmit(float(t)) for t in send_times]
        delivered = sorted((d for d in out if d.delivered), key=lambda d: d.arrival)
        lost = [d for d in out if not d.delivered]
        return delivered + lost


@dataclass
class LinkStatus:
    """接收端可观测的链路质量快照。"""

    t: float
    age: float            # 距最近一条成功消息的时长（s）；从无消息时为 inf
    pdr: float            # 统计窗口内的投递率
    jitter: float         # 到达-发送时延的标准差（s）
    silent_s: float       # 连续静默时长（s）
    warm: bool = True     # 统计窗口是否已填满（未满时 pdr 不可信）

    @property
    def fresh(self) -> bool:
        return self.age < 1e9


class LinkMonitor:
    """接收端质量估计器：滑窗投递率 + 命令年龄 + 时延抖动。"""

    def __init__(self, rate_hz: float, window: float = 2.0):
        self.rate_hz = float(rate_hz)
        self.window = float(window)
        self.arrivals: List[Tuple[float, float]] = []   # (send_t, arrival_t)
        self.last_arrival: float | None = None
        # 首包到达前，年龄从链路启动时刻开始累计，而不是无穷大：
        # 否则开机瞬间就会被状态机误判为“已失联”。
        self.start: float | None = None

    def push(self, send_t: float, arrival_t: float) -> None:
        self.arrivals.append((float(send_t), float(arrival_t)))
        self.last_arrival = float(arrival_t)

    def update(self, now: float) -> LinkStatus:
        if self.start is None:
            self.start = float(now)
        lo = now - self.window
        self.arrivals = [(s, a) for s, a in self.arrivals if a >= lo]
        expected = max(1.0, self.window * self.rate_hz)
        pdr = min(1.0, len(self.arrivals) / expected)
        if self.arrivals:
            delays = np.array([a - s for s, a in self.arrivals])
            jitter = float(np.std(delays))
        else:
            jitter = float("nan")
        age = (now - self.last_arrival) if self.last_arrival is not None else (now - self.start)
        warm = (now - self.start) >= self.window
        return LinkStatus(t=now, age=float(age), pdr=pdr, jitter=jitter,
                          silent_s=float(age), warm=bool(warm))


# --------------------------------------------------------------------------- #
# 预置链路场景
# --------------------------------------------------------------------------- #
def default_profiles() -> Dict[str, LinkProfile]:
    """四档链路：理想 / 中等 / 恶劣 / 计划中断。

    参数取自课程演示常用值：图传 120–250 ms 单向、公网 15–35% 丢包、
    8 s 完全遮挡。数值是可调起点，不是对真实电台的标定。
    """
    return {
        "clean": LinkProfile(
            name="clean", label="理想链路",
            one_way_delay=0.02, jitter_std=0.005,
        ),
        "moderate": LinkProfile(
            name="moderate", label="中等链路",
            one_way_delay=0.12, jitter_std=0.04,
            loss_good=0.08, loss_bad=0.35, p_good_to_bad=0.06, p_bad_to_good=0.25,
        ),
        "severe": LinkProfile(
            name="severe", label="恶劣链路",
            one_way_delay=0.25, jitter_std=0.09,
            loss_good=0.18, loss_bad=0.55, p_good_to_bad=0.12, p_bad_to_good=0.18,
        ),
        "outage": LinkProfile(
            name="outage", label="计划中断",
            one_way_delay=0.12, jitter_std=0.04,
            loss_good=0.05, loss_bad=0.30, p_good_to_bad=0.05, p_bad_to_good=0.30,
            outages=((7.0, 20.0),),
        ),
    }
