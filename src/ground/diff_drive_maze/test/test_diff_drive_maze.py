#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""走迷宫仿真测试：纯逻辑单测 + 闭环轨迹到达 / 撞墙 / 车体支撑断言。

用法（在包目录下执行）::

    python3 test/test_diff_drive_maze.py
    python3 -O test/test_diff_drive_maze.py   # 断言不会被剔除

纯逻辑部分不加载 MuJoCo 模型；闭环部分加载 ``models/diff_drive_maze.xml``，
复用 ``main.py`` 的同一套常量与控制律，断言小车确实绕过 L 型隔墙到达终点、
未接触墙壁。测试统一用 ``unittest.TestCase`` 的 ``self.assert*``，而不是裸
``assert``——后者在 ``python3 -O`` 下会被整体剔除，让失败永远“通过”。
"""
import os
import sys
import math
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import mujoco  # noqa: F401
    _MUJOCO_OK = True
except Exception as exc:  # 环境未装 MuJoCo 时给出明确提示，而不是裸 ImportError
    _MUJOCO_OK = False
    _MUJOCO_ERR = exc

import main as m  # noqa: E402


class TestDiffDriveMaze(unittest.TestCase):
    """走迷宫作业的单元测试。"""

    # ---- 纯逻辑部分（不需要 MuJoCo）--------------------------------------

    def test_wrap_angle(self):
        self.assertAlmostEqual(m.wrap_angle(0.0), 0.0, places=12)
        self.assertAlmostEqual(m.wrap_angle(math.pi), math.pi, places=12)
        self.assertAlmostEqual(m.wrap_angle(3 * math.pi / 2), -math.pi / 2, places=12)

    def test_quat_to_yaw(self):
        # 绕 z 轴转 90°：四元数 [qw,qx,qy,qz] = [cos45, 0, 0, sin45]
        q = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
        self.assertAlmostEqual(m.quat_to_yaw(q), math.pi / 2, places=12)
        # 回归原 bug：qpos[5] 是 qy，绕 z 转时恒为 0，不能当 yaw
        self.assertAlmostEqual(q[2], 0.0, places=12)
        self.assertAlmostEqual(m.quat_to_yaw([1.0, 0.0, 0.0, 0.0]), 0.0, places=12)

    def test_diff_drive_vels(self):
        # 直行 omega=0：左右轮等速
        wl, wr = m.diff_drive_vels(0.5, 0.0)
        self.assertAlmostEqual(wl, wr, places=12)
        self.assertAlmostEqual(wl, 0.5 / m.WHEEL_RADIUS, places=12)
        # 原地转 v=0：左右轮等大反号
        wl, wr = m.diff_drive_vels(0.0, 1.0)
        self.assertAlmostEqual(wl + wr, 0.0, places=12)

    def test_arrive(self):
        state = {"wp": 0, "mode": "turn"}
        left, right, state, arrived, dist = m.control_step(
            m.GOAL_X, m.GOAL_Y, 0.0, state)
        self.assertIs(arrived, True)
        self.assertEqual((left, right), (0.0, 0.0))
        self.assertLess(dist, m.ARRIVE_DIST)

    def test_waypoint_transition(self):
        # 已接近第 0 段航点 (2.0, -2.0) → 切换到第 1 段航点 (2.0, 2.0)，并重新进入转向
        state = {"wp": 0, "mode": "drive"}
        left, right, state, arrived, _ = m.control_step(2.0, -2.0, 0.0, state)
        self.assertIs(arrived, False)
        self.assertEqual(state["wp"], 1)
        self.assertEqual(state["mode"], "turn")

    # ---- 需要 MuJoCo 的部分 ---------------------------------------------

    @unittest.skipUnless(_MUJOCO_OK, "未安装 MuJoCo")
    def test_initial_pose(self):
        model = m.load_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.qpos[0], m.START_X, delta=1e-3)
        self.assertAlmostEqual(data.qpos[1], m.START_Y, delta=1e-3)

    @unittest.skipUnless(_MUJOCO_OK, "未安装 MuJoCo")
    def test_closed_loop_reaches_goal(self):
        model = m.load_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        res = m.run(model, data, viewer=None)

        self.assertIs(res["arrived"], True, f"未到达终点：{res}")
        self.assertIs(res["collided"], False, f"闭环过程中撞墙：{res}")
        self.assertLess(res["final_dist"], m.ARRIVE_DIST)

    @unittest.skipUnless(_MUJOCO_OK, "未安装 MuJoCo")
    def test_wall_contact_detected(self):
        model = m.load_model()
        data = mujoco.MjData(model)
        # 把底盘直接放到横墙 (y=0) 位置上，强制产生墙体接触
        data.qpos[0] = -0.75
        data.qpos[1] = 0.0
        mujoco.mj_forward(model, data)
        self.assertIs(m._touch_wall(model, data), True)

    @unittest.skipUnless(_MUJOCO_OK, "未安装 MuJoCo")
    def test_chassis_supported_by_wheels(self):
        """车体必须由四轮支撑：底盘不与地面接触，俯仰角不超限。"""
        model = m.load_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        state = {"wp": 0, "mode": "turn"}
        chassis_floor = 0
        max_pitch = 0.0
        step = 0
        arrived = False
        while step < m.MAX_STEPS:
            x, y = data.qpos[0], data.qpos[1]
            yaw = m.quat_to_yaw(data.qpos[3:7])
            left, right, state, arrived, _ = m.control_step(x, y, yaw, state)
            data.ctrl[0], data.ctrl[1] = left, right
            mujoco.mj_step(model, data)
            step += 1

            qw, qx, qy, qz = data.qpos[3:7]
            pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
            max_pitch = max(max_pitch, abs(pitch))
            for c in range(data.ncon):
                g1 = model.geom(data.contact[c].geom1).name
                g2 = model.geom(data.contact[c].geom2).name
                if {g1, g2} == {"chassis", "floor"}:
                    chassis_floor += 1

            if m._touch_wall(model, data) or arrived:
                break

        self.assertIs(arrived, True, "闭环未到达终点，无法统计支撑情况")
        self.assertEqual(chassis_floor, 0, "底盘与地面发生接触，说明车体没被轮子支撑")
        self.assertLess(math.degrees(max_pitch), 5.0,
                        f"俯仰角过大：{math.degrees(max_pitch):.2f}°")


if __name__ == "__main__":
    unittest.main(verbosity=2)
