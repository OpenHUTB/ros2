#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线监视的纯逻辑内核：从一帧多机位置里挑出最接近机对并判定是否侵入。

这一层刻意不依赖 ROS：节点只负责订阅里程计、把 ``(x, y, z)`` 塞进字典、
把结果发布成话题与 Marker，判断逻辑全在这里，因此可以在没有 ROS 的机器上
用单元测试覆盖（``test/test_monitor.py``）。

监视与计划层的关系：计划层（``resolve``）在飞之前把冲突解掉；本层在飞的过程
中回答"现在谁离谁最近、有没有破间隔"，只告警、不改航迹。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple

from .conflict import Separation

Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Closest:
    """一帧里最接近的一对机。

    ``min_h`` / ``min_v`` 在机队不足两架（或全部数据过期）时为空。
    """

    pair: Tuple[str, str] = ("", "")
    min_h: Optional[float] = None
    min_v: Optional[float] = None
    violating: bool = False
    aircraft: int = 0

    def as_dict(self, sep: Separation, violations: int) -> dict:
        """转成发布用的 JSON 友好结构（保留两位小数）。"""
        return dict(
            aircraft=self.aircraft,
            min_h=None if self.min_h is None else round(self.min_h, 2),
            min_v=None if self.min_v is None else round(self.min_v, 2),
            pair=list(self.pair),
            violating=self.violating,
            violations=violations,
            sep=dict(horizontal=sep.horizontal, vertical=sep.vertical))


def fresh_positions(positions: Mapping[str, Point3],
                    stamps: Mapping[str, float],
                    now: float,
                    stale_timeout: float) -> Dict[str, Point3]:
    """剔除超过 ``stale_timeout`` 未更新的机（掉线/未起飞）。"""
    return {uid: p for uid, p in positions.items()
            if now - stamps.get(uid, -math.inf) <= stale_timeout}


def closest(positions: Mapping[str, Point3], sep: Separation) -> Closest:
    """在给定的一帧位置里找水平最接近的一对，并判定圆柱间隔是否被侵入。

    约定：按机号字典序枚举，先比较水平距离；水平相同时保留枚举顺序靠前的一对，
    因此同一帧数据的输出是确定的（便于回归测试与日志对账）。
    """
    names = sorted(positions)
    if len(names) < 2:
        return Closest(aircraft=len(names))

    best_h = math.inf
    best_v = math.inf
    pair: Tuple[str, str] = ("", "")
    for i, a in enumerate(names):
        pa = positions[a]
        for b in names[i + 1:]:
            pb = positions[b]
            dx = pa[0] - pb[0]
            dy = pa[1] - pb[1]
            h = math.hypot(dx, dy)
            if h < best_h:
                best_h = h
                best_v = abs(pa[2] - pb[2])
                pair = (a, b)

    violating = best_h < sep.horizontal and best_v < sep.vertical
    return Closest(pair=pair, min_h=best_h, min_v=best_v,
                   violating=violating, aircraft=len(names))


def evaluate(positions: Mapping[str, Point3],
             stamps: Mapping[str, float],
             now: float,
             sep: Separation,
             stale_timeout: float) -> Closest:
    """新鲜度过滤 + 最近机对判定，节点每帧调用这一次。"""
    return closest(fresh_positions(positions, stamps, now, stale_timeout), sep)
