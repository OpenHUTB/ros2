#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分配算法：EDF 贪心初始解 + 局部交换改进（互换 / 机间迁移）。

阶段一（贪心）：任务按最早截止优先（EDF，``latest`` 升序、高优先级优先）
逐个插入——对每个任务枚举"分配给哪架机、插在序列哪个位置"，取使目标函数
增量最小且不产生新的不可行（航程超续航）的方案。

阶段二（局部搜索）：反复尝试两类邻域动作，直到无严格下降动作或达到轮数
上限——

1. **两任务互换**：不同机（或同机不同位置）的两个任务对调；
2. **机间迁移**：把一个任务从一架机摘下插到另一架机的序列里。

接受条件 = 目标函数严格下降。目标函数 = Σ正延误 +
λ×(最大单机延误 − 最小单机延误)。贪心不保证全局最优，但每一步都可解释、
可复现，且改进阶段只接受严格下降，结果不会比初始解差。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .mission import Config, Scenario, Task, Uav
from .window import UavSchedule, objective, schedule_uav, uav_cost

# 分配结果：uid -> 有序任务 id 列表
Assignment = Dict[str, List[str]]


@dataclass
class AllocateResult:
    """分配结果与全过程记录。"""

    assign: Assignment                       # 改进后分配
    schedules: Dict[str, UavSchedule]        # 改进后时刻表
    greedy_assign: Assignment                # 贪心初始解
    greedy_objective: float
    final_objective: float
    swaps: int                               # 互换动作数
    transfers: int                           # 迁移动作数
    rounds: int
    dropped: List[dict] = field(default_factory=list)  # 不可达任务诊断

    def as_dict(self) -> dict:
        return dict(
            greedy_assign={k: list(v) for k, v in self.greedy_assign.items()},
            assign={k: list(v) for k, v in self.assign.items()},
            greedy_objective=round(self.greedy_objective, 2),
            final_objective=round(self.final_objective, 2),
            swaps=self.swaps, transfers=self.transfers, rounds=self.rounds,
            dropped=self.dropped,
        )


# --------------------------------------------------------------------------- #
# 工具
# --------------------------------------------------------------------------- #
def _empty_assignment(uavs: Sequence[Uav]) -> Assignment:
    return {u.uid: [] for u in uavs}


def _objective_of(assign: Assignment, uavs: Dict[str, Uav],
                  tasks: Dict[str, Task], cfg: Config) -> float:
    scheds = {uid: schedule_uav(uavs[uid], tids, tasks, cfg)
              for uid, tids in assign.items()}
    return objective(scheds, cfg)


def _schedules_of(assign: Assignment, uavs: Dict[str, Uav],
                  tasks: Dict[str, Task], cfg: Config) -> Dict[str, UavSchedule]:
    return {uid: schedule_uav(uavs[uid], tids, tasks, cfg)
            for uid, tids in assign.items()}


def _route_distance(uav: Uav, tids: Sequence[str],
                    tasks: Dict[str, Task]) -> float:
    """任务序列的地面航程（起点 → 逐任务点），用于续航检验。"""
    pts = [uav.pos] + [tasks[t].pos for t in tids]
    return sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))


def _within_endurance(uav: Uav, tids: Sequence[str],
                      tasks: Dict[str, Task]) -> bool:
    """航程不超续航折算里程（服务/等待不耗巡航油量，按地面航程近似）。"""
    return _route_distance(uav, tids, tasks) <= uav.range_m() + 1e-9


# --------------------------------------------------------------------------- #
# 阶段一：EDF 贪心
# --------------------------------------------------------------------------- #
def greedy_edf(scenario: Scenario,
               feasible: Sequence[str]) -> Tuple[Assignment, Dict[str, UavSchedule]]:
    """最早截止优先贪心：任务按 EDF 排序后逐个插入最优（机, 位置）。"""
    cfg = scenario.config
    uavs = {u.uid: u for u in scenario.uavs}
    tasks = {t.tid: t for t in scenario.tasks}

    # EDF 排序：紧急（priority 大）优先，其后按 latest 升序
    order = sorted(feasible,
                   key=lambda tid: (-tasks[tid].priority, tasks[tid].latest,
                                    tid))
    assign = _empty_assignment(scenario.uavs)
    for tid in order:
        best: Optional[Tuple[float, str, int]] = None
        for uid, u in sorted(uavs.items()):
            seq = assign[uid]
            for i in range(len(seq) + 1):
                trial = seq[:i] + [tid] + seq[i:]
                # 续航硬约束：塞进去航程超限的方案直接跳过（不硬塞）
                if not _within_endurance(u, trial, tasks):
                    continue
                trial_assign = {k: list(v) for k, v in assign.items()}
                trial_assign[uid] = trial
                delta = _objective_of(trial_assign, uavs, tasks, cfg) \
                    - _objective_of(assign, uavs, tasks, cfg)
                cand = (delta, uid, i)
                if best is None or cand < best:
                    best = cand
        if best is None:
            # 所有 (机, 位置) 都超续航——理论上预检已拦住，保守兜底不分配
            continue
        _, uid, i = best
        assign[uid] = assign[uid][:i] + [tid] + assign[uid][i:]
    scheds = _schedules_of(assign, uavs, tasks, cfg)
    return assign, scheds


# --------------------------------------------------------------------------- #
# 阶段二：局部搜索（互换 / 迁移，严格下降才接受）
# --------------------------------------------------------------------------- #
def _neighbor_moves(assign: Assignment) -> List[Tuple[str, ...]]:
    """枚举全部邻域动作（规模 O((Σ|seq|)²)，场景规模下足够快）。"""
    moves: List[Tuple[str, ...]] = []
    uids = sorted(assign)
    for a in uids:
        for i in range(len(assign[a])):
            # 1) 同机相邻前移/后移（2-opt 交换）
            for j in range(len(assign[a])):
                if i != j:
                    moves.append(("swap", a, i, a, j))
            # 2) 机间互换与迁移
            for b in uids:
                if b == a:
                    continue
                for j in range(len(assign[b])):
                    moves.append(("swap", a, i, b, j))
                for j in range(len(assign[b]) + 1):
                    moves.append(("move", a, i, b, j))
    return moves


def _apply_move(assign: Assignment, move: Tuple[str, ...]) -> Assignment:
    kind, a, i, b, j = move
    out = {k: list(v) for k, v in assign.items()}
    if kind == "swap":
        out[a][i], out[b][j] = out[b][j], out[a][i]
    else:
        tid = out[a].pop(i)
        out[b].insert(j, tid)
    return out


def improve(assign: Assignment, scenario: Scenario,
            max_rounds: Optional[int] = None) -> Tuple[Assignment, int, int, int]:
    """局部搜索主循环，返回 (新分配, swaps, transfers, rounds)。"""
    cfg = scenario.config
    uavs = {u.uid: u for u in scenario.uavs}
    tasks = {t.tid: t for t in scenario.tasks}
    max_rounds = max_rounds if max_rounds is not None else cfg.max_improve_rounds

    cur = {k: list(v) for k, v in assign.items()}
    cur_obj = _objective_of(cur, uavs, tasks, cfg)
    swaps = transfers = rounds = 0
    for _ in range(max_rounds):
        rounds += 1
        improved = False
        for move in _neighbor_moves(cur):
            trial = _apply_move(cur, move)
            # 续航硬约束：任一架超限即拒绝
            if any(not _within_endurance(uavs[uid], tids, tasks)
                   for uid, tids in trial.items()):
                continue
            obj = _objective_of(trial, uavs, tasks, cfg)
            if obj < cur_obj - 1e-9:            # 严格下降才接受
                cur, cur_obj = trial, obj
                if move[0] == "swap":
                    swaps += 1
                else:
                    transfers += 1
                improved = True
                break
        if not improved:
            break
    return cur, swaps, transfers, rounds


# --------------------------------------------------------------------------- #
# 入口
# --------------------------------------------------------------------------- #
def allocate(scenario: Scenario,
             feasible: Optional[Sequence[str]] = None,
             dropped: Optional[List[dict]] = None) -> AllocateResult:
    """完整分配：EDF 贪心 → 局部交换改进。

    Parameters
    ----------
    feasible
        参与分配的任务 id（预检可达的）；缺省为全部任务。
    dropped
        预检判为不可达的任务诊断行，原样并入结果。
    """
    cfg = scenario.config
    uavs = {u.uid: u for u in scenario.uavs}
    tasks = {t.tid: t for t in scenario.tasks}
    all_ids = [t.tid for t in scenario.tasks]
    if feasible is None:
        feasible = all_ids

    g_assign, g_scheds = greedy_edf(scenario, feasible)
    g_obj = objective(g_scheds, cfg)

    f_assign, swaps, transfers, rounds = improve(g_assign, scenario)
    f_scheds = _schedules_of(f_assign, uavs, tasks, cfg)
    f_obj = objective(f_scheds, cfg)

    assigned = {tid for tids in f_assign.values() for tid in tids}
    miss = [tid for tid in feasible if tid not in assigned]
    drop_rows = list(dropped or [])
    drop_rows += [dict(tid=tid, infeasible=True,
                       reason="无可用方案（所有 (机,位置) 均超续航）", reachable_by=[])
                  for tid in miss]

    return AllocateResult(
        assign=f_assign, schedules=f_scheds,
        greedy_assign=g_assign, greedy_objective=g_obj,
        final_objective=f_obj, swaps=swaps, transfers=transfers,
        rounds=rounds, dropped=drop_rows)


def report_rows(result: AllocateResult) -> List[dict]:
    """逐机分配表（改进后），供控制台 / JSON / 甘特图共用。"""
    rows = []
    for uid in sorted(result.assign):
        sched = result.schedules[uid]
        rows.append(dict(
            uid=uid,
            tasks=list(result.assign[uid]),
            lateness_total=round(sched.lateness_total, 2),
            lateness_max=round(sched.lateness_max, 2),
            wait_total=round(sched.wait_total, 2),
            makespan=round(sched.makespan, 2),
            stops=sched.as_dict()["stops"],
        ))
    return rows


# --------------------------------------------------------------------------- #
# 在线调度：ROS 节点的判定逻辑（无 ROS 依赖，可单测）
# --------------------------------------------------------------------------- #
def schedule_from_positions(positions: Dict[str, Tuple[float, float]],
                            tasks: Sequence[Task],
                            cfg: Config,
                            speeds: Optional[Dict[str, float]] = None,
                            endurance: float = 420.0) -> dict:
    """以各机**当前位置**为起点重排任务，返回分配表与预计到达时刻。

    供 ``ros/schedule_node.py`` 周期调用：机队位置变了（或任务表变了）
    就整体重算一次 EDF 贪心 + 局部改进。返回::

        dict(assign={uid: [tid, ...]},
             eta={tid: 预计服务开始时刻},
             lateness={uid: 单机正延误合计},
             objective=机队目标函数)
    """
    uavs = [Uav(uid=uid, x=p[0], y=p[1],
                speed=(speeds or {}).get(uid, cfg.default_speed),
                endurance=endurance)
            for uid, p in sorted(positions.items())]
    scenario = Scenario(name="online", title="", description="",
                        uavs=uavs, tasks=list(tasks), config=cfg)
    alloc = allocate(scenario)
    eta = {}
    for uid, sched in alloc.schedules.items():
        for stop in sched.stops:
            eta[stop.tid] = round(stop.serve_start, 2)
    return dict(
        assign={uid: list(tids) for uid, tids in alloc.assign.items()},
        eta=eta,
        lateness={uid: round(s.lateness_total, 2)
                  for uid, s in alloc.schedules.items()},
        objective=round(alloc.final_objective, 2),
        dropped=[d["tid"] for d in alloc.dropped if d.get("infeasible")],
    )
