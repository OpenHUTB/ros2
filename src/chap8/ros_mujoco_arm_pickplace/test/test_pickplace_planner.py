#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""决策层测试：解析 IK / 限位 / 5 状态机（不需要安装 mujoco 即可跑）。

对应文档第 7 节「测试」中的运动学与状态机部分。
"""
import os
import subprocess
import sys

import numpy as np
import pytest

from pickplace_planner import (
    CTRL_HZ, JOINT_NAMES, JOINT_RANGE, STATE_LIFT_PLACE, STATE_RESET,
    WPT_APPROACH, WPT_GRASP, WPT_HOME, WPT_LIFT, WPT_PLACE, WPT_PLACE_A,
    PickPlacePlanner, clamp_joints, fk, ik,
)

WAYPOINTS = [
    ("HOME", WPT_HOME),
    ("APPROACH", WPT_APPROACH),
    ("GRASP", WPT_GRASP),
    ("LIFT", WPT_LIFT),
    ("PLACE_A", WPT_PLACE_A),
    ("PLACE", WPT_PLACE),
]


def drive_planner(planner, cycles=1, max_steps=4000):
    """理想跟踪下驱动状态机：每拍把上次下发的目标当作当前关节角。

    返回 (transitions, cmds)，cmds 是所有下发指令的列表。
    """
    dt = 1.0 / CTRL_HZ
    planner.start()
    q_now = np.zeros(planner.nj)
    cmds = []
    finished = 0
    prev_state = planner.state
    for _ in range(max_steps):
        ns, nhc, cmd = planner.decide(q_now, planner.has_cube, dt)
        planner.state, planner.has_cube = ns, nhc
        if cmd is not None:
            for c in (cmd if isinstance(cmd, list) else [cmd]):
                cmds.append(c)
                if c[0] == "target":
                    q_now = np.asarray(c[1], dtype=float)
        if planner.state == STATE_RESET and prev_state == STATE_LIFT_PLACE:
            finished += 1
            if finished >= cycles:
                break
        prev_state = planner.state
    return planner.transitions, cmds


def test_joint_table_consistent():
    assert len(JOINT_NAMES) == 6
    assert len(JOINT_RANGE) == 6
    assert JOINT_RANGE[5] == (0.00, 1.20)      # 夹爪 0..1.2
    assert JOINT_RANGE[1] == (-1.60, 1.60)     # shoulder_lift


@pytest.mark.parametrize("name,xyz", WAYPOINTS, ids=[w[0] for w in WAYPOINTS])
def test_ik_roundtrip_each_waypoint(name, xyz):
    q = ik(xyz)
    err = float(np.linalg.norm(fk(q) - np.asarray(xyz, dtype=float)))
    # 解析 FK 自校验：六个路点误差均应在亚毫米级
    assert err < 1e-3, f"{name} IK 往返误差 {err*1000:.3f} mm 超限"
    assert q.shape == (6,)
    assert q[5] == 0.0


def test_ik_pan_flips_for_negative_x():
    q = ik(WPT_PLACE)
    # x<0 -> pan≈±π；关节限位是 ±3.14，所以被裁到 3.14（末端误差 <1 mm）
    assert abs(q[0]) > 3.13
    q2 = ik(WPT_GRASP)
    assert abs(q2[0]) < 1e-9                      # x>0 -> pan = 0


def test_ik_respects_joint_limits():
    for _, xyz in WAYPOINTS:
        q = ik(xyz)
        for (lo, hi), v in zip(JOINT_RANGE, q):
            assert lo - 1e-9 <= v <= hi + 1e-9, f"关节值 {v} 越界 [{lo},{hi}]"


def test_clamp_joints_clips_out_of_range():
    raw = np.array([10.0, 5.0, -5.0, 2.0, -4.0, 3.0])
    q = clamp_joints(raw)
    for (lo, hi), v in zip(JOINT_RANGE, q):
        assert lo <= v <= hi


def test_fsm_one_cycle_transitions():
    planner = PickPlacePlanner()
    transitions, _ = drive_planner(planner, cycles=1)
    assert transitions == [0, 1, 2, 3, 4, 0], transitions


def test_fsm_two_cycles_and_reset_once_per_cycle():
    planner = PickPlacePlanner()
    transitions, cmds = drive_planner(planner, cycles=2)
    assert transitions == [0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0], transitions
    # 每个周期只应请求一次方块复位（评审关注的“第 2 周期抓不到”回归点）
    assert sum(1 for c in cmds if c[0] == "reset") == 2
    grasp_on = [c for c in cmds if c[0] == "grasp" and c[1] is True]
    grasp_off = [c for c in cmds if c[0] == "grasp" and c[1] is False]
    assert len(grasp_on) == 2 and len(grasp_off) == 2


def test_planner_does_not_import_mujoco():
    """权重：状态机节点只依赖 planner，因此 planner 绝不能把 mujoco 拉进来。"""
    scripts = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    scripts = os.path.join(scripts, "scripts")
    code = (
        "import sys; sys.path.insert(0, r'%s');"
        "import pickplace_planner;"
        "print('mujoco' in sys.modules)" % scripts
    )
    out = subprocess.check_output([sys.executable, "-c", code], text=True).strip()
    assert out == "False", f"pickplace_planner 间接导入了 mujoco：{out}"
