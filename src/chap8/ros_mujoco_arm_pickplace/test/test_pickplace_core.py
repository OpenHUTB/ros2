#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物理层测试：模型加载 / IK 与模型 FK 对账 / 方块复位 / 抓取距离校验 / 整周期无 NaN。

需要 pip3 install mujoco；未安装时本文件整体跳过（决策层测试仍可跑）。
"""
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from pickplace_core import (  # noqa: E402  (importorskip 之后才能 import)
    GRASP_CUBE_INIT_POS, STATE_GRASP, STATE_LIFT_PLACE, STATE_RESET,
    WPT_APPROACH, WPT_GRASP, WPT_HOME, WPT_LIFT, WPT_PLACE, WPT_PLACE_A,
    PickPlaceCore,
)

WAYPOINTS = [
    ("HOME", WPT_HOME),
    ("APPROACH", WPT_APPROACH),
    ("GRASP", WPT_GRASP),
    ("LIFT", WPT_LIFT),
    ("PLACE_A", WPT_PLACE_A),
    ("PLACE", WPT_PLACE),
]


@pytest.fixture(scope="module")
def core():
    return PickPlaceCore(verbose=False)


def test_model_structure(core):
    """模型自带 freejoint 方块、ee_site 参考点与 6 路 position 伺服。"""
    assert core.model.nq == 13          # 6 臂关节 + freejoint(7)
    assert core.model.nv == 12          # 6 臂关节 + freejoint(6)
    assert core.model.nu == 6
    assert core.cube_free_id >= 0
    assert core.ee_site >= 0
    assert core.nj == 6


@pytest.mark.parametrize("name,xyz", WAYPOINTS, ids=[w[0] for w in WAYPOINTS])
def test_ik_agrees_with_model_fk(core, name, xyz):
    """解析 IK 的解用 MuJoCo 自己的 FK 复核（独立于解析 FK）。"""
    err, q = core.verify_ik(xyz)
    assert err < 3e-3, f"{name} 用模型 FK 复核误差 {err*1000:.2f} mm 超限"
    assert np.all(np.abs(q) <= np.array([3.14, 1.6, 2.8, 1.2, 3.14, 1.2]) + 1e-9)


def test_ik_does_not_pollute_simulation_state(core):
    """IK 是纯计算：调用前后 qpos 必须逐字节相同（旧版靠 try/finally 兜底）。"""
    before = core.data.qpos.copy()
    for _, xyz in WAYPOINTS:
        core.ik(xyz)
    assert np.array_equal(before, core.data.qpos)


def test_reset_cube_puts_it_back_and_it_settles(core):
    core.reset_cube()
    pos, _ = core.read_cube_pose()
    assert np.allclose(pos, GRASP_CUBE_INIT_POS, atol=1e-9)
    for _ in range(300):        # 自由下落并静置
        core.step(10)
    pos, _ = core.read_cube_pose()
    assert pos[2] < 0.06        # 落到地面（初始中心 0.05，允许小抖动）
    assert abs(pos[0] - GRASP_CUBE_INIT_POS[0]) < 0.01
    assert np.all(np.isfinite(core.data.qacc))


def test_grasp_rejected_when_end_effector_far(core):
    """末端远离方块时不允许 attach（抓取距离校验）。"""
    for a in core.qpos_adr:
        core.data.qpos[a] = 0.0
    core.data.qvel[:] = 0.0
    core.reset_cube()
    mujoco.mj_forward(core.model, core.data)
    core.attached = False
    core.grasp(True)
    core.apply_weld_request()
    assert core.attached is False
    assert core._pending_weld is None


def test_one_cycle_moves_cube_without_nan(core):
    """完整 1 周期：状态链 0→1→2→3→4→0，无 NaN，方块真的从 +0.4 搬到 -0.4。"""
    res = core.run_fsm(max_cycles=1)
    assert res["cycles"] == 1
    assert res["nan_count"] == 0
    trans = res["transitions"]
    assert trans[:6] == [STATE_RESET, 1, 2, STATE_GRASP, STATE_LIFT_PLACE, STATE_RESET]
    xmin, xmax = res["cube_x_range"]
    assert xmax > 0.30, f"没抓到 +x 侧的方块：xmax={xmax}"
    assert xmin < -0.30, f"方块没被搬到 -x 侧：xmin={xmin}"
    assert res["cube_final_z"] < 0.10
    for name, err, _q in res["ee_errors"]:
        assert err < 3e-3, f"{name} IK 误差 {err*1000:.2f} mm"
