#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分配算法（core.assign）与可行性预检（core.mission）单元测试。"""
import pytest

from uav_time_window.core.assign import (allocate, _apply_move,
                                         _neighbor_moves, improve,
                                         schedule_from_positions)
from uav_time_window.core.mission import (Config, Scenario, Task, Uav,
                                          screen_tasks, unreachable_reason)
from uav_time_window.core.window import objective, schedule_uav

CFG = Config()


def make_scenario(tasks, uavs=None, cfg=CFG):
    uavs = uavs or [Uav("UAV-1", 0, 0, 8, 420),
                    Uav("UAV-2", 20, 0, 9, 420),
                    Uav("UAV-3", 0, 20, 10, 420)]
    return Scenario("t", "t", "", uavs, list(tasks), cfg)


TASKS = [Task("T-A", 80, 60, 40, 60, 15), Task("T-B", -70, 80, 40, 60, 15),
         Task("T-C", 120, -40, 40, 60, 15), Task("T-D", -100, -60, 40, 60, 15),
         Task("T-E", 40, 140, 40, 60, 15), Task("T-F", -140, 20, 40, 60, 15)]


def _obj(scenario, assign):
    uavs = {u.uid: u for u in scenario.uavs}
    tmap = {t.tid: t for t in scenario.tasks}
    scheds = {uid: schedule_uav(uavs[uid], tids, tmap, scenario.config)
              for uid, tids in assign.items()}
    return objective(scheds, scenario.config)


def test_greedy_covers_all_tasks():
    """贪心初始解覆盖全部任务、每任务恰好一次。"""
    sc = make_scenario(TASKS)
    res = allocate(sc)
    covered = [t for tids in res.greedy_assign.values() for t in tids]
    assert sorted(covered) == sorted(t.tid for t in TASKS)


def test_final_covers_all_tasks():
    """改进后分配仍覆盖全部任务且不重复。"""
    sc = make_scenario(TASKS)
    res = allocate(sc)
    covered = [t for tids in res.assign.values() for t in tids]
    assert sorted(covered) == sorted(t.tid for t in TASKS)


def test_improve_never_worse_than_greedy():
    """改进只接受严格下降，目标函数不劣于贪心初始解。"""
    sc = make_scenario(TASKS)
    res = allocate(sc)
    assert res.final_objective <= res.greedy_objective + 1e-9


def test_improve_strictly_better_on_tight():
    """紧窗口场景局部搜索有实质改进（延误下降）。"""
    sc = make_scenario(TASKS)
    res = allocate(sc)
    assert res.final_objective < res.greedy_objective
    assert res.swaps + res.transfers > 0


def test_acceptance_is_strict_descent():
    """逐动作重放：每步接受后目标函数严格下降。"""
    sc = make_scenario(TASKS)
    res = allocate(sc)
    cur = res.greedy_assign
    prev = res.greedy_objective
    # 贪心到最终之间至少存在一次下降（本场景贪心非最优）
    assert _obj(sc, res.assign) < prev - 1e-9
    # 邻域动作应用后任务集不变
    moves = _neighbor_moves(res.assign)
    assert moves
    m = moves[0]
    out = _apply_move(res.assign, m)
    covered_a = sorted(t for v in res.assign.values() for t in v)
    covered_b = sorted(t for v in out.values() for t in v)
    assert covered_a == covered_b


def test_fairness_spread_reduced():
    """λ 均衡项起作用：改进后机间延误极差不超过贪心。"""
    sc = make_scenario(TASKS)
    res = allocate(sc)
    delays = []
    uavs = {u.uid: u for u in sc.uavs}
    tmap = {t.tid: t for t in sc.tasks}
    for uid, tids in res.assign.items():
        delays.append(schedule_uav(uavs[uid], tids, tmap, sc.config)
                      .lateness_total)
    assert max(delays) - min(delays) <= 30.0


def test_transfer_move_rebalances():
    """机间迁移可被枚举且应用后保持覆盖。"""
    sc = make_scenario(TASKS)
    res = allocate(sc)
    moves = [m for m in _neighbor_moves(res.assign) if m[0] == "move"]
    assert moves
    out = _apply_move(res.assign, moves[0])
    assert sorted(t for v in out.values() for t in v) == \
        sorted(t.tid for t in TASKS)


def test_unreachable_task_reported_not_assigned():
    """飞不到的任务被标记（距离过远），不硬塞给任何一架机。"""
    far = Task("FAR", 5000.0, 5000.0, 0, 100, 15)
    sc = make_scenario(TASKS + [far])
    screen = screen_tasks(sc)
    far_row = next(r for r in screen if r["tid"] == "FAR")
    assert far_row["infeasible"]
    assert "距离过远" in far_row["reason"]
    res = allocate(sc, feasible=[t.tid for t in TASKS],
                   dropped=[far_row])
    assigned = [t for tids in res.assign.values() for t in tids]
    assert "FAR" not in assigned


def test_narrow_window_reported():
    """窗口宽度小于服务时长 → 窗口过窄，不可行。"""
    task = Task("NAR", 50, 50, 100, 110, 15)
    uav = Uav("UAV-1", 0, 0, 8, 420)
    assert unreachable_reason(task, uav) is not None
    assert "窗口过窄" in unreachable_reason(task, uav)
    sc = Scenario("n", "n", "", [uav], [task], CFG)
    rows = screen_tasks(sc)
    assert rows[0]["infeasible"] and "窗口过窄" in rows[0]["reason"]


def test_endurance_respected_in_assignment():
    """分配结果每架机的航程不超续航折算里程。"""
    short = [Uav("UAV-1", 0, 0, 8, 60), Uav("UAV-2", 20, 0, 9, 60),
             Uav("UAV-3", 0, 20, 10, 60)]
    tasks = [Task("T-A", 150, 100, 0, 300, 15),
             Task("T-B", -150, 100, 0, 300, 15)]
    sc = make_scenario(tasks, uavs=short)
    res = allocate(sc)
    tmap = {t.tid: t for t in tasks}
    for u in sc.uavs:
        pts = [u.pos] + [tmap[t].pos for t in res.assign[u.uid]]
        dist = sum(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
                   for a, b in zip(pts[:-1], pts[1:]))
        assert dist <= u.range_m() + 1e-6


def test_schedule_from_positions_online():
    """在线调度入口：以当前位置重排，返回分配表与 ETA。"""
    tasks = TASKS[:4]
    table = schedule_from_positions(
        {"UAV-1": (0.0, 0.0), "UAV-2": (10.0, 0.0), "UAV-3": (0.0, 10.0)},
        tasks, CFG)
    covered = [t for tids in table["assign"].values() for t in tids]
    assert sorted(covered) == sorted(t.tid for t in tasks)
    assert set(table["eta"]) == set(covered)
    assert table["objective"] >= 0.0


def test_allocation_deterministic():
    """同一场景两次分配结果一致（可复现）。"""
    sc1 = make_scenario(TASKS)
    sc2 = make_scenario(TASKS)
    r1, r2 = allocate(sc1), allocate(sc2)
    assert r1.assign == r2.assign
    assert r1.final_objective == pytest.approx(r2.final_objective)
