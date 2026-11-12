#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""避障仿真测试：纯逻辑单测 + 闭环轨迹绕障断言。

用法（在包目录下执行）::

    python3 test/test_obstacle_avoidance.py

纯逻辑部分不加载 MuJoCo 模型；闭环部分加载 ``models/obstacle_avoidance.xml``，
复用 ``main.py`` 的同一套常量与控制律，断言小车确实绕开障碍并到达目标。
"""
import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main as m  # noqa: E402


def test_wrap_angle():
    assert abs(m.wrap_angle(0.0)) < 1e-12
    assert abs(m.wrap_angle(math.pi) - math.pi) < 1e-12
    assert abs(m.wrap_angle(3 * math.pi / 2) + math.pi / 2) < 1e-12
    print("[ok] wrap_angle 归一化到 (-pi, pi]")


def test_quat_to_yaw():
    # 绕 z 轴转 90°：四元数 [qw,qx,qy,qz] = [cos45, 0, 0, sin45]
    q = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
    yaw = m.quat_to_yaw(q)
    assert abs(yaw - math.pi / 2) < 1e-12, f"yaw={yaw}"
    # 回归原 bug：qpos[5] 是 qy，绕 z 转时恒为 0，不能当 yaw
    assert abs(q[2]) < 1e-12
    assert abs(m.quat_to_yaw([1.0, 0.0, 0.0, 0.0])) < 1e-12
    print("[ok] quat_to_yaw 从四元数解算偏航角（qpos[5] 不是 yaw）")


def test_diff_drive_vels():
    # 直行 omega=0：左右轮等速
    wl, wr = m.diff_drive_vels(0.6, 0.0)
    assert abs(wl - wr) < 1e-12
    assert abs(wl - 0.6 / m.WHEEL_RADIUS) < 1e-12
    # v=0 纯转动：左右轮等大反号
    wl, wr = m.diff_drive_vels(0.0, 1.0)
    assert abs(wl + wr) < 1e-12
    print("[ok] diff_drive_vels 差速逆解（轮角速度 rad/s）")


def test_arrive():
    state = {"mode": "cruise", "cruise_mode": "drive"}
    left, right, state, arrived, dist = m.control_step(
        m.GOAL_X, m.GOAL_Y, 0.0, state, m.RAY_MAX_RANGE)
    assert arrived is True
    assert left == 0.0 and right == 0.0
    assert dist < m.ARRIVE_DIST
    print("[ok] control_step 到达判据：距目标小于阈值即停车")


def test_state_machine_avoidance_transition():
    # 巡航中前方射线近障 → 进入左转绕行
    state = {"mode": "cruise", "cruise_mode": "drive"}
    left, right, state, arrived, _ = m.control_step(
        0.0, 0.0, 0.0, state, front_dist=0.3)
    assert state["mode"] == "turn_left"
    assert arrived is False

    # 左转到位（yaw 已接近 +y）→ 进入横向绕行段
    state["mode"] = "turn_left"
    left, right, state, _, _ = m.control_step(
        0.4, 0.0, math.pi / 2 - 0.05, state, front_dist=0.3)
    assert state["mode"] == "side"

    # 横向抬到 SIDE_Y → 进入回正段
    state["mode"] = "side"
    left, right, state, _, _ = m.control_step(
        0.6, m.SIDE_Y, math.pi / 2, state, front_dist=0.3)
    assert state["mode"] == "turn_right"

    # 回正到位 → 回到巡航
    state["mode"] = "turn_right"
    left, right, state, _, _ = m.control_step(
        0.8, m.SIDE_Y, 0.05, state, front_dist=m.RAY_MAX_RANGE)
    assert state["mode"] == "cruise"
    print("[ok] 避障状态机：巡航 → 左转 → 横向绕行 → 回正 → 巡航")


def test_closed_loop_avoids_and_reaches():
    # 闭环加载模型，复用主程序常量；断言到达且轨迹绕开障碍
    import mujoco
    model = m.load_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    res = m.run(model, data, viewer=None)

    assert res["arrived"] is True, f"未到达目标：{res}"
    assert res["final_dist"] < m.ARRIVE_DIST
    assert res["avoid_triggered"] is True, "前向射线未触发避障"
    # 车半宽 0.1 m + 障碍半径 0.1 m，中心距需大于 0.2 m 才不接触；
    # 留 0.05 m 余量，断言最小中心距 > 0.25 m。
    assert res["min_obs_dist"] > 0.25, f"轨迹过于贴近障碍：{res['min_obs_dist']:.3f} m"
    print(f"[ok] 闭环到达：{res['steps']} 步，距目标 {res['final_dist']:.3f} m，"
          f"到障碍最小中心距 {res['min_obs_dist']:.3f} m")


def main():
    test_wrap_angle()
    test_quat_to_yaw()
    test_diff_drive_vels()
    test_arrive()
    test_state_machine_avoidance_transition()
    test_closed_loop_avoids_and_reaches()
    print("\n全部测试通过。")


if __name__ == "__main__":
    main()
