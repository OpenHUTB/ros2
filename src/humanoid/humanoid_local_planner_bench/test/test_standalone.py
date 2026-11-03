#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""humanoid_local_planner_bench 自检。

两种跑法：
    python3 test/test_standalone.py      # 独立运行，失败退出码 1
    python3 -m pytest test/ -v           # pytest / colcon 收集

所有检查都用 assert 或抛异常，**失败必须让运行器失败**，
不能只打印 FAIL 就正常返回。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import humanoid_local_planner_bench.config as config          # noqa: E402
from humanoid_local_planner_bench.sim_core import (         # noqa: E402
    MIN_MUJOCO, CorridorSim, check_mujoco_version)
from humanoid_local_planner_bench.planners import PLANNERS    # noqa: E402

SIM = None


def sim():
    global SIM
    if SIM is None:
        SIM = CorridorSim()
    return SIM


try:
    import pytest

    @pytest.fixture(name="sim")
    def _sim_fixture():
        """pytest 用：各测试的第一参数 sim 由 fixture 提供。"""
        return CorridorSim()

except ImportError:                       # 未安装 pytest（直接跑脚本）时不需要 fixture
    pass


# ----------------------------------------------------------------------
# 一、场景与几何
# ----------------------------------------------------------------------
def test_scene_loads(sim):
    """模型能加载，几何数量与关节数符合预期。"""
    assert sim.model.nq >= 7, "自由关节至少 7 个 qpos"
    assert sim.model.ngeom >= 10, "场景几何体数量异常"
    names = {sim.mujoco.mj_id2name(sim.model, sim.mujoco.mjtObj.mjOBJ_GEOM, g)
             for g in range(sim.model.ngeom)}
    for need in ("footprint", "floor", "obs_a", "obs_b", "door_jamb_s", "door_jamb_n"):
        assert need in names, "缺少几何体 {}".format(need)


def test_lidar_returns_typed_scan(sim):
    """激光束数与量程正确。"""
    sim.set_pose(0.0, 0.0, 0.0)
    scan = sim.scan()
    assert len(scan) == config.N_RAYS, "激光束数应为 {}".format(config.N_RAYS)
    for ang, dist in scan:
        assert 0.0 <= dist <= config.RAY_RANGE + 1e-6, "测距超出量程"


def test_lidar_wall_distance(sim):
    """起点朝向墙时，最近测距约等于墙内表面到机器人中心的距离。"""
    sim.set_pose(0.0, 0.0, 0.0)
    scan = sim.scan()
    nearest = min(d for _, d in scan)
    # 墙内表面在 y=±1.54，机器人中心在 y=0
    assert abs(nearest - 1.54) < 0.06, "最近测距 {} 偏离 1.54 过多".format(nearest)


def test_door_clearance(sim):
    """门洞净宽 0.90 m、足迹半径 0.28 m → 表面间距应为 0.17 m。"""
    sim.set_pose(4.0, 0.0, 0.0)
    clr = sim.clearance()
    assert abs(clr - (0.90 / 2 - 0.28)) < 0.02, "门洞净空 {} 不符合 0.17".format(clr)


def test_collision_detection(sim):
    """贴住门柱时必须判定为碰撞，远离时必须不碰撞。"""
    sim.set_pose(1.5, 0.0, 0.0)
    assert not sim.collided(), "空旷处不应判定碰撞"
    sim.set_pose(4.0, 1.10, 0.0)
    assert sim.collided(), "压住门柱应判定碰撞"


def test_gait_does_not_affect_clearance(sim):
    """腿部是纯视觉（contype=0），摆动不应影响碰撞与间距。"""
    sim.set_pose(1.5, 0.0, 0.0, phase=0.0)
    c0 = sim.clearance()
    hit0 = sim.collided()
    sim.set_pose(1.5, 0.0, 0.0, phase=math.pi / 2)
    c1 = sim.clearance()
    hit1 = sim.collided()
    assert abs(c0 - c1) < 1e-6, "步态改变了间距：{} -> {}".format(c0, c1)
    assert hit0 == hit1, "步态改变了碰撞判定"


# ----------------------------------------------------------------------
# 二、规划器
# ----------------------------------------------------------------------
def _run(name, time_limit=60.0):
    from humanoid_local_planner_bench.metrics import RunRecorder
    s = CorridorSim()
    s.reset(*config.START)
    planner = PLANNERS[name]()
    planner.reset()
    rec = RunRecorder()
    dt = 1.0 / config.CONTROL_HZ
    rec.add(0.0, s.pose(), 0.0, 0.0, s.clearance(), s.collided())
    t, phase = 0.0, 0.0
    while t < time_limit:
        pose = s.pose()
        v, w = planner.step(pose, s.scan(), config.GOAL, dt)
        x = pose[0] + v * math.cos(pose[2]) * dt
        y = pose[1] + v * math.sin(pose[2]) * dt
        yaw = pose[2] + w * dt
        phase += (v * dt) / 0.7 * 2 * math.pi
        s.set_pose(x, y, yaw, phase)
        t += dt
        rec.add(t, s.pose(), v, w, s.clearance(), s.collided())
        if math.hypot(x - config.GOAL[0], y - config.GOAL[1]) <= config.GOAL_TOL:
            break
    return rec, rec.summary(config.GOAL, config.GOAL_TOL)


def test_planners_reach_goal_without_collision():
    """三种规划器都必须到达终点且全程零碰撞。"""
    for name in PLANNERS:
        _rec, m = _run(name)
        assert m["reached"], "{} 未到达终点（最终距离 {:.2f} m）".format(name, m["final_dist"])
        assert m["collision_steps"] == 0, "{} 发生 {} 步碰撞".format(name, m["collision_steps"])
        assert m["min_clearance"] > 0.10, "{} 最小间距过小：{:.3f}".format(name, m["min_clearance"])


def test_first_sample_is_initial_state():
    """第一条样本必须是 t=0 的初始状态，不能是"走完第一步之后"的位置。"""
    rec, _m = _run("dwa", time_limit=3.0)
    s0 = rec.samples[0]
    assert s0["t"] == 0.0, "首条样本时间应为 0，实际 {}".format(s0["t"])
    assert abs(s0["x"] - config.START[0]) < 1e-9, "首条样本 x 不是起点：{}".format(s0["x"])
    assert abs(s0["y"] - config.START[1]) < 1e-9, "首条样本 y 不是起点：{}".format(s0["y"])


def test_footprint_matters():
    """足迹半径 0.28 m 时代价更高：同一条直线路径在门洞处不可通行。

    这条检查保证「人形宽度」确实被纳入判定，而不是被当成质点。
    """
    s = CorridorSim()
    # 直线 y=0 穿过门洞：中心线净空 0.17*2 = 0.34 m，但若按质点则必然"通过"
    s.set_pose(4.0, 0.0, 0.0)
    assert s.clearance() < 0.20, "门洞处净空应小于 0.20 m，说明足迹半径生效"
    assert s.clearance() > 0.0, "门洞中线不应直接碰撞"


class _FakeMujoco:
    def __init__(self, version):
        self.__version__ = version


def test_mujoco_version_guard():
    """低于 3.3 必须拒绝启动，3.3 及以上放行。"""
    check_mujoco_version(_FakeMujoco("%d.%d.%d" % MIN_MUJOCO))
    check_mujoco_version(_FakeMujoco("3.16.0"))
    for bad in ("3.2.3", "3.2.7", "3.0.0", "2.9.9"):
        try:
            check_mujoco_version(_FakeMujoco(bad))
        except RuntimeError:
            continue
        raise AssertionError("%s 应被拒绝" % bad)


def test_empty_summary_has_all_keys(sim):
    """一帧都没跑时 summary() 也要返回完整键，不能 KeyError。"""
    from humanoid_local_planner_bench.metrics import RunRecorder
    m = RunRecorder().summary(config.GOAL)
    for key in ("reached", "time", "path_len", "min_clearance",
                "collision_steps", "mean_w", "final_dist"):
        assert key in m, "空记录缺少键 {}".format(key)
    assert m["reached"] is False


# ----------------------------------------------------------------------
def main():
    print("=" * 62)
    print("  humanoid_local_planner_bench 自检")
    print("=" * 62)
    checks = [
        ("场景加载", lambda: test_scene_loads(CorridorSim())),
        ("激光束数与量程", lambda: test_lidar_returns_typed_scan(CorridorSim())),
        ("激光测墙距离", lambda: test_lidar_wall_distance(CorridorSim())),
        ("门洞净空 0.17 m", lambda: test_door_clearance(CorridorSim())),
        ("碰撞判定", lambda: test_collision_detection(CorridorSim())),
        ("步态不影响判定", lambda: test_gait_does_not_affect_clearance(CorridorSim())),
        ("规划器到达且零碰撞", test_planners_reach_goal_without_collision),
        ("足迹宽度生效", test_footprint_matters),
        ("首条样本为初始状态", test_first_sample_is_initial_state),
        ("空记录不抛异常", lambda: test_empty_summary_has_all_keys(None)),
        ("MuJoCo 版本下界", test_mujoco_version_guard),
    ]
    failed = 0
    for title, fn in checks:
        try:
            fn()
            print("  [PASS] {}".format(title))
        except AssertionError as exc:
            failed += 1
            print("  [FAIL] {} —— {}".format(title, exc))
        except Exception as exc:                              # noqa: BLE001
            failed += 1
            print("  [ERROR] {} —— {}: {}".format(title, type(exc).__name__, exc))
    print("=" * 62)
    if failed:
        print("  {} 项未通过".format(failed))
        return 1
    print("  全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
