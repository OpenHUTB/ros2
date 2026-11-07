#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四个可复现的分配场景：tight / loose / unbalanced / urgent。

所有场景共用同一套机队底子（3 架机：西南基地 + 南库 + 西库，速度
8–10 m/s，续航 420 s），差别只在任务点布置与时间窗松紧——这样四个
场景之间的对比才有意义：

- **tight**：6 个点围绕基地四周，服务窗口紧（latest=60 s），必然大面积
  晚到，考验总延误与机间均衡的取舍；
- **loose**：与 tight 相同布置，窗口放宽到 [60, 260] s，基本都能准点
  （早到需悬停等待），分配退化为尽量少飞路；
- **unbalanced**：9 个点全部压在东北角（150–300 m 外），机队驻扎西南，
  每机要连跑 3 个点，检验机间迁移能否拉开均衡；
- **urgent**：2 个 priority=10 的紧急任务（窗口早而窄）+ 8 个普通任务，
  检验 EDF 排序对紧急任务窗口的保护。

窗口参数经过标定：tight / unbalanced / urgent 三个场景的贪心解均有
明显正延误，局部搜索有实质改进空间（数值见 README 结果表）。
"""
from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

from .mission import Config, Scenario, Task, Uav

CONFIG = Config()


def _uavs() -> List[Uav]:
    return [
        Uav("UAV-1", 0.0, 0.0, speed=8.0, endurance=420.0, label="西南基地"),
        Uav("UAV-2", 20.0, 0.0, speed=9.0, endurance=420.0, label="南库"),
        Uav("UAV-3", 0.0, 20.0, speed=10.0, endurance=420.0, label="西库"),
    ]


def _grid(prefix: str, cells: Sequence[Tuple[str, float, float]],
          earliest: float, latest: float, service: float = 15.0,
          priority: int = 0) -> List[Task]:
    return [Task("%s-%s" % (prefix, sfx), x, y, earliest, latest,
                 service_time=service, priority=priority)
            for sfx, x, y in cells]


#: tight / loose 共用的 6 个任务点（基地四周）
RING_CELLS = [("A", 80, 60), ("B", -70, 80), ("C", 120, -40),
              ("D", -100, -60), ("E", 40, 140), ("F", -140, 20)]

#: unbalanced 的 9 个任务点（全在东北角）
NE_CELLS = [("A", 180, 120), ("B", 240, 160), ("C", 150, 200),
            ("D", 260, 80), ("E", 200, 240), ("F", 300, 150),
            ("G", 220, 100), ("H", 170, 170), ("I", 280, 200)]

#: urgent 的 8 个普通任务点
URGENT_NORMAL_CELLS = [("A", 120, -40), ("B", -100, -60), ("C", 40, 140),
                       ("D", -140, 20), ("E", 80, 100), ("F", -60, 60),
                       ("G", 150, 60), ("H", -30, -100)]


def tight() -> Scenario:
    """窗口紧：6 个点，服务开始不得晚于 60 s。"""
    return Scenario(
        "tight", "紧时间窗",
        "6 个任务点分布基地四周，服务开始统一不得晚于 60 s：三架机无论"
        "怎么排序都必然产生晚到，考验目标函数在总延误与机间均衡间的取舍。",
        _uavs(), _grid("T", RING_CELLS, 40.0, 60.0), CONFIG)


def loose() -> Scenario:
    """窗口宽：同样的 6 个点，窗口 [60, 260] s。"""
    return Scenario(
        "loose", "宽时间窗",
        "与 tight 相同的任务布置，但窗口放宽到 200 s：绝大多数任务可以"
        "准点甚至早到（早到需悬停等待），分配退化为尽量少飞路的旅行商。",
        _uavs(), _grid("L", RING_CELLS, 60.0, 260.0), CONFIG)


def unbalanced() -> Scenario:
    """任务分布偏一侧：9 个点全在东北角，机队停在西南。"""
    return Scenario(
        "unbalanced", "任务分布偏一侧",
        "9 个任务点全部集中在东北角（单程 150–300 m）、机队驻扎西南角，"
        "每机要连跑 3 个点：一架机包揽全部分配会延误爆炸，必须靠机间"
        "迁移与序列重排拉开均衡。",
        _uavs(), _grid("U", NE_CELLS, 60.0, 100.0), CONFIG)


def urgent() -> Scenario:
    """含紧急任务：2 个 priority=10 紧急点（窗口早而窄）+ 8 个普通点。"""
    tasks = _grid("E", [("EM1", 60, 80), ("EM2", -60, 100)],
                  40.0, 70.0, priority=10)
    tasks += _grid("N", URGENT_NORMAL_CELLS, 70.0, 110.0)
    return Scenario(
        "urgent", "含紧急任务",
        "2 个 priority=10 的紧急任务（窗口早而窄，如医疗投送）与 8 个"
        "普通任务混排：EDF 贪心应优先保紧急点，改进阶段不得为均衡而"
        "牺牲紧急任务的窗口。",
        _uavs(), tasks, CONFIG)


SCENARIOS: Dict[str, callable] = {
    "tight": tight,
    "loose": loose,
    "unbalanced": unbalanced,
    "urgent": urgent,
}


def get_scenario(name: str) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError("未知场景 %r，可选：%s" % (name, ", ".join(SCENARIOS)))
    return SCENARIOS[name]()
