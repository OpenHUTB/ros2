#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""沿墙巡线仿真测试：纯逻辑单测 + 闭环绕回起点 / 撞墙断言。

用法（在包目录下执行）::

    python3 test/test_wall_following.py
    python3 -m unittest discover -s test -p "test_*.py"

用 ``unittest.TestCase`` 组织，断言走 ``self.assert*`` 方法而不是裸 ``assert``
语句：

- 可被 ``unittest`` / pytest / nose 自动收集；
- 断言是方法调用，``python3 -O`` 优化模式下不会被编译期剔除，仍会真正校验。

纯逻辑部分不加载 MuJoCo 模型；闭环部分加载 ``models/wall_following.xml``，
复用 ``main.py`` 的同一套常量与控制律，断言小车确实绕矩形一圈回到起点、
未接触墙壁。
"""
import os
import sys
import math
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import mujoco  # noqa: F401
    _MUJOCO_OK = True
except Exception as exc:  # 环境未装 MuJoCo 时跳过闭环测试，而不是裸 ImportError
    _MUJOCO_OK = False
    _MUJOCO_ERR = exc

import main as m  # noqa: E402


def _fresh_state():
    return {"wp": 0, "mode": "turn", "side_filt": m.RAY_MAX_RANGE}


class TestWallFollowing(unittest.TestCase):

    def test_wrap_angle(self):
        self.assertAlmostEqual(m.wrap_angle(0.0), 0.0, places=12)
        self.assertAlmostEqual(m.wrap_angle(math.pi), math.pi, places=12)
        # 超过 pi 折回 (-pi, pi]
        self.assertAlmostEqual(m.wrap_angle(3 * math.pi / 2), -math.pi / 2, places=12)
        self.assertAlmostEqual(m.wrap_angle(-3 * math.pi / 2), math.pi / 2, places=12)

    def test_quat_to_yaw(self):
        # 绕 z 轴转 90°：四元数 [qw,qx,qy,qz] = [cos45, 0, 0, sin45]
        q = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
        yaw = m.quat_to_yaw(q)
        self.assertAlmostEqual(yaw, math.pi / 2, places=12, msg=f"yaw={yaw}")
        # 回归原 bug：qpos[5] 是 qy，绕 z 转时恒为 0，不能当 yaw
        self.assertAlmostEqual(q[2], 0.0, places=12)
        # 单位四元数 → yaw 0
        self.assertAlmostEqual(m.quat_to_yaw([1.0, 0.0, 0.0, 0.0]), 0.0, places=12)

    def test_diff_drive_vels(self):
        # 直行 omega=0：左右轮等速
        wl, wr = m.diff_drive_vels(0.45, 0.0)
        self.assertAlmostEqual(wl, wr, places=12)
        self.assertAlmostEqual(wl, 0.45 / m.WHEEL_RADIUS, places=12)
        # 转向 v=0：左右轮等大反号
        wl, wr = m.diff_drive_vels(0.0, 1.0)
        self.assertAlmostEqual(wl + wr, 0.0, places=12)
        self.assertAlmostEqual(wr - wl, 1.0 * m.WHEEL_BASE / m.WHEEL_RADIUS, places=12)

    def test_arrive(self):
        # 末航点就在起点 → 到达、两轮速度 0
        state = {"wp": len(m.WAYPOINTS) - 1, "mode": "turn",
                 "side_filt": m.RAY_MAX_RANGE}
        left, right, _, arrived, dist = m.control_step(
            m.START_X, m.START_Y, 0.0, state,
            m.RAY_MAX_RANGE, m.RAY_MAX_RANGE)
        self.assertTrue(arrived)
        self.assertEqual(left, 0.0)
        self.assertEqual(right, 0.0)
        self.assertLess(dist, m.END_ARRIVE_DIST)

    def test_waypoint_transition(self):
        # 已接近第 0 段航点 (1.45, -1.45) → 切换到第 1 段航点，重新进入转向
        state = _fresh_state()
        state["mode"] = "drive"
        left, right, state, arrived, _ = m.control_step(
            1.45, -1.45, 0.0, state,
            m.RAY_MAX_RANGE, m.RAY_MAX_RANGE)
        self.assertFalse(arrived)
        self.assertEqual(state["wp"], 1)
        self.assertEqual(state["mode"], "turn")

    def test_side_pd_participates(self):
        # 直行段：离右墙偏近（侧距小于目标）时，侧距 PD 项把角速度往远离墙一侧推
        state = _fresh_state()
        state["mode"] = "drive"
        state["side_filt"] = m.D_DES  # 滤波已预热到目标侧距
        # 位置 (0.5,-0.5)、车头朝 -pi/4，正对第 0 航点，航向误差 0；侧距明显偏小
        yaw = -math.pi / 4
        left_close, right_close, _, _, _ = m.control_step(
            0.5, -0.5, yaw, state, m.RAY_MAX_RANGE, 0.15)
        omega_close = (right_close - left_close) * m.WHEEL_RADIUS / m.WHEEL_BASE
        self.assertGreater(omega_close, 0.0)

    def test_front_slowdown(self):
        # 前向距墙过近时直行限速到 SAFE_FRONT 以下的低速
        state = _fresh_state()
        state["mode"] = "drive"
        left, right, _, _, _ = m.control_step(
            0.5, -0.5, -math.pi / 4, state, 0.20, m.RAY_MAX_RANGE)
        v = (left + right) * m.WHEEL_RADIUS / 2.0
        self.assertLessEqual(v, 0.15 + 1e-9)

    def test_initial_pose(self):
        if not _MUJOCO_OK:
            self.skipTest(f"未安装 MuJoCo，跳过初始位姿测试：{_MUJOCO_ERR}")
        model = m.load_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(data.qpos[0], m.START_X, places=3)
        self.assertAlmostEqual(data.qpos[1], m.START_Y, places=3)

    def test_closed_loop_returns_start(self):
        if not _MUJOCO_OK:
            self.skipTest(f"未安装 MuJoCo，跳过闭环轨迹测试：{_MUJOCO_ERR}")
        model = m.load_model()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        res = m.run(model, data, viewer=None)

        self.assertTrue(res["arrived"], f"未绕回起点：{res}")
        self.assertFalse(res["collided"], f"闭环过程中撞墙：{res}")
        self.assertLess(res["dist_start"], m.END_ARRIVE_DIST)

    def test_wall_contact_detected(self):
        if not _MUJOCO_OK:
            self.skipTest(f"未安装 MuJoCo，跳过撞墙判定测试：{_MUJOCO_ERR}")
        model = m.load_model()
        data = mujoco.MjData(model)
        # 把底盘直接放到南墙 (y=-1.95) 位置上，强制产生墙体接触
        data.qpos[0] = 0.0
        data.qpos[1] = -1.95
        mujoco.mj_forward(model, data)
        self.assertTrue(m._touch_wall(model, data))


if __name__ == "__main__":
    unittest.main()
