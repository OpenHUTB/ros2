#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""冲突解脱：在三种真实可用的空域手段里按代价择优。

低空交通管理里可用的解脱手段，按对任务的影响从小到大排序：

===============  ==================================================  ==============
手段             做法                                                代价
===============  ==================================================  ==============
地面等待          推迟起飞时刻，航路不动                              延误 Δt
高度层调整        换一个巡航高度（相当于换高度层）                    额外爬升/下降
侧向绕行          在冲突段插入横向绕行点，过冲突后回原航路            额外航程
===============  ==================================================  ==============

高优先级任务（``priority`` 大，例如医疗急救）默认不参与让路；同级任务里
起飞较晚的一架让路——这与实际运行中"先到先得"的排队直觉一致。

算法是贪心的：每次挑一个冲突，为该让路飞机枚举候选动作，用**整队重新检测**
后的冲突数与总代价评分，取最优者应用，直到无冲突或所有冲突都试过仍无改善。
贪心不保证全局最优，但每一步都可解释、可复现，且不会制造新的冲突对。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .conflict import Conflict, Separation, detect
from .plan import Plan


@dataclass(frozen=True)
class Limits:
    """运行限制：可用高度层范围与最大可接受地面等待。"""

    alt_min: float = 30.0
    alt_max: float = 120.0
    max_delay: float = 120.0
    delay_steps: Tuple[float, ...] = (5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0)
    alt_steps: Tuple[float, ...] = (15.0, 30.0, 45.0)
    lateral_steps: Tuple[float, ...] = (20.0, 40.0, 60.0)


@dataclass
class Action:
    """一次解脱动作。"""

    kind: str            # delay / altitude / lateral
    uid: str
    value: float
    cost: float          # 折算成"等效延误秒"的代价
    note: str = ""

    def describe(self) -> str:
        if self.kind == "delay":
            return "%s 地面等待 %.0f s" % (self.uid, self.value)
        if self.kind == "altitude":
            return "%s 高度层 %+.0f m" % (self.uid, self.value)
        return "%s 侧向绕行 %.0f m" % (self.uid, self.value)


@dataclass
class Resolution:
    """解脱结果与全过程记录。"""

    plans: Dict[str, Plan]
    actions: List[Action]
    initial: List[Conflict]
    remaining: List[Conflict]
    iterations: int
    protected: Set[str] = field(default_factory=set)

    @property
    def solved(self) -> bool:
        return not self.remaining

    def delay_of(self, uid: str, baseline: Dict[str, Plan]) -> float:
        return self.plans[uid].end_time - baseline[uid].end_time

    def summary(self) -> dict:
        return dict(initial_conflicts=len(self.initial),
                    remaining_conflicts=len(self.remaining),
                    actions=len(self.actions),
                    solved=self.solved,
                    iterations=self.iterations,
                    total_action_cost=round(sum(a.cost for a in self.actions), 1))


@dataclass
class _Candidate:
    action: Action
    plan: Plan
    cost: float
    delay_equiv: float


# --------------------------------------------------------------------------- #
# 候选生成
# --------------------------------------------------------------------------- #
def _candidates(plan: Plan, limits: Limits,
                baseline_end: float) -> List[_Candidate]:
    out: List[_Candidate] = []

    # 1) 地面等待
    for dt in limits.delay_steps:
        if plan.t0 + dt > limits.max_delay:
            continue
        cand = plan.with_ground_delay(dt)
        out.append(_Candidate(
            Action("delay", plan.uid, dt, cost=dt,
                   note="起飞推迟 %.0f s" % dt),
            cand, cost=dt, delay_equiv=dt))

    # 2) 高度层调整（改巡航高度，需重新爬升/下降）
    for dz in limits.alt_steps:
        for sign in (+1, -1):
            alt = plan.alt + sign * dz
            if alt < limits.alt_min - 1e-9 or alt > limits.alt_max + 1e-9:
                continue
            cand = plan.with_altitude(alt)
            extra = 2.0 * abs(sign * dz) / plan.climb_speed
            cost = extra + 5.0
            out.append(_Candidate(
                Action("altitude", plan.uid, sign * dz, cost=cost,
                       note="巡航高度 %.0f → %.0f m" % (plan.alt, alt)),
                cand, cost=cost, delay_equiv=cand.end_time - baseline_end))

    # 3) 侧向绕行（航向左右各一个候选）
    for off in limits.lateral_steps:
        for sign in (+1, -1):
            cand = plan.with_lateral_offset(sign * off)
            extra = 2.0 * off / plan.speed * 1.5 + 5.0
            out.append(_Candidate(
                Action("lateral", plan.uid, sign * off, cost=extra,
                       note="航路中点侧向偏移 %+.0f m" % (sign * off)),
                cand, cost=extra, delay_equiv=cand.end_time - baseline_end))

    # 先试代价最小的
    out.sort(key=lambda c: c.cost)
    return out


def _give_way(conflict: Conflict, plans: Dict[str, Plan],
              protected: Set[str]) -> Optional[str]:
    a, b = plans[conflict.a], plans[conflict.b]
    if a.uid in protected and b.uid in protected:
        return None
    if a.uid in protected:
        return b.uid
    if b.uid in protected:
        return a.uid
    if a.priority != b.priority:
        return b.uid if b.priority < a.priority else a.uid
    # 同优先级：起飞较晚者让（先到先得）
    return b.uid if (b.t0, b.uid) >= (a.t0, a.uid) else a.uid


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def resolve_fleet(plans: Sequence[Plan], sep: Separation,
                  limits: Optional[Limits] = None,
                  protected: Iterable[str] = (),
                  max_rounds: int = 60) -> Resolution:
    """迭代消解全机队冲突。"""
    limits = limits or Limits()
    protected = set(protected)

    base: Dict[str, Plan] = {p.uid: p for p in plans}
    work: Dict[str, Plan] = {p.uid: p.copy_with() for p in plans}
    baseline_end = {uid: p.end_time for uid, p in base.items()}

    initial = detect(list(base.values()), sep)
    current = list(initial)
    actions: List[Action] = []
    blocked: Set[Tuple[str, str]] = set()
    iterations = 0

    for iterations in range(1, max_rounds + 1):
        if not current:
            break
        applied = False
        for conflict in current:
            if conflict.pair in blocked:
                continue
            uid = _give_way(conflict, work, protected)
            if uid is None:
                blocked.add(conflict.pair)
                continue

            best: Optional[Tuple[Tuple[int, float, float], _Candidate]] = None
            for cand in _candidates(work[uid], limits, baseline_end[uid]):
                trial = dict(work)
                trial[uid] = cand.plan
                cons = detect(list(trial.values()), sep)
                if len(cons) >= len(current):
                    # 只接受严格减少冲突数的动作，避免在原地来回换策略
                    continue
                score = (len(cons), sum(a.cost for a in actions) + cand.cost,
                         cand.cost)
                if best is None or score < best[0]:
                    best = (score, cand)

            if best is None:
                blocked.add(conflict.pair)
                continue

            cand = best[1]
            work[uid] = cand.plan
            actions.append(cand.action)
            current = detect(list(work.values()), sep)
            applied = True
            break

        if not applied:
            break

    return Resolution(plans=work, actions=actions, initial=initial,
                      remaining=current, iterations=iterations,
                      protected=protected)


def report_rows(resolution: Resolution, baseline: Dict[str, Plan]) -> List[dict]:
    """每架飞机的延误与动作摘要，供表格/作图使用。"""
    rows = []
    for uid, plan in sorted(resolution.plans.items()):
        base = baseline[uid]
        acts = [a.describe() for a in resolution.actions if a.uid == uid]
        rows.append(dict(
            uid=uid,
            label=plan.label,
            t0_base=round(base.t0, 1),
            t0_new=round(plan.t0, 1),
            alt_base=round(base.alt, 1),
            alt_new=round(plan.alt, 1),
            delay=round(plan.end_time - base.end_time, 1),
            path_xy=round(plan.path_length_xy(), 1),
            actions="; ".join(acts) if acts else "—",
        ))
    return rows
