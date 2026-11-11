#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""走迷宫仿真测试：纯逻辑单测 + 闭环轨迹到达 / 撞墙断言。

用法（在包目录下执行）::

    python3 test/test_diff_drive_maze.py

纯逻辑部分不加载 MuJoCo 模型；闭环部分加载 ``models/diff_drive_maze.xml``，
复用 ``main.py`` 的同一套常量与控制律，断言小车确实绕过 L 型隔墙到达终点、
未接触墙壁。
"""
import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import mujoco  # noqa: F401
    _MUJOCO_OK = True
except Exception as exc:  # 环境未装 MuJoCo 时给出明确提示，而不是裸 ImportError
    _MUJOCO_OK = False
    _MUJOCO_ERR = exc

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
    wl, wr = m.diff_drive_vels(0.5, 0.0)
    assert abs(wl - wr) < 1e-12
    assert abs(wl - 0.5 / m.WHEEL_RADIUS) < 1e-12
    # 原地转 v=0：左右轮等大反号
    wl, wr = m.diff_drive_vels(0.0, 1.0)
    assert abs(wl + wr) < 1e-12
    print("[ok] diff_drive_vels 差速逆解（轮角速度 rad/s）")


def test_arrive():
    state = {"wp": 0, "mode": "turn"}
    left, right, state, arrived, dist = m.control_step(
        m.GOAL_X, m.GOAL_Y, 0.0, state)
    assert arrived is True
    assert left == 0.0 and right == 0.0
    assert dist < m.ARRIVE_DIST
    print("[ok] control_step 到达判据：距终点小于阈值即停车")


def test_waypoint_transition():
    # 已接近第 0 段航点 (2.0, -2.0) → 切换到第 1 段航点 (2.0, 2.0)，并重新进入转向
    state = {"wp": 0, "mode": "drive"}
    left, right, state, arrived, _ = m.control_step(
        2.0, -2.0, 0.0, state)
    assert arrived is False
    assert state["wp"] == 1
    assert state["mode"] == "turn"
    print("[ok] 航点状态机：到达航点后切换下一段并重新对准航向")


def test_initial_pose():
    if not _MUJOCO_OK:
        print(f"[skip] 未安装 MuJoCo，跳过初始位姿测试：{_MUJOCO_ERR}")
        return
    model = m.load_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert abs(data.qpos[0] - m.START_X) < 1e-3
    assert abs(data.qpos[1] - m.START_Y) < 1e-3
    print(f"[ok] 初始位姿 ({data.qpos[0]:.2f}, {data.qpos[1]:.2f}) m")


def test_closed_loop_reaches_goal():
    if not _MUJOCO_OK:
        print(f"[skip] 未安装 MuJoCo，跳过闭环轨迹测试：{_MUJOCO_ERR}")
        return
    model = m.load_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    res = m.run(model, data, viewer=None)

    assert res["arrived"] is True, f"未到达终点：{res}"
    assert res["collided"] is False, f"闭环过程中撞墙：{res}"
    assert res["final_dist"] < m.ARRIVE_DIST
    print(f"[ok] 闭环到达：{res['steps']} 步，终止位置 "
          f"({res['final_x']:.3f}, {res['final_y']:.3f}) m，"
          f"距终点 {res['final_dist']:.3f} m，未接触墙壁")


def test_wall_contact_detected():
    if not _MUJOCO_OK:
        print(f"[skip] 未安装 MuJoCo，跳过撞墙判定测试：{_MUJOCO_ERR}")
        return
    model = m.load_model()
    data = mujoco.MjData(model)
    # 把底盘直接放到横墙 (y=0) 位置上，强制产生墙体接触
    data.qpos[0] = -0.75
    data.qpos[1] = 0.0
    mujoco.mj_forward(model, data)
    assert m._touch_wall(model, data) is True
    print("[ok] 撞墙判定：接触墙壁几何即识别为撞墙")


def main():
    test_wrap_angle()
    test_quat_to_yaw()
    test_diff_drive_vels()
    test_arrive()
    test_waypoint_transition()
    test_initial_pose()
    test_closed_loop_reaches_goal()
    test_wall_contact_detected()
    print("\n全部测试通过。")


if __name__ == "__main__":
    main()
