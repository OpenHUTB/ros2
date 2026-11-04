#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AEB 模块单元测试：纯逻辑 + 仿真集成（无 mujoco 时自动跳过）。

运行：

    python3 test/test_aeb_logic.py

覆盖：
  * 控制律（TTC 公式、比例制动、限幅、低速保护）
  * 指标定义（首次接触、制动介入时刻、制动距离、汇总）
  * 仿真集成（基线必须刹住、关闭 AEB 必须追尾——这两个是互为对照的回归）
  * 模型/入口约定（关节命名、下标不再硬编码、main.* 与 launch 齐全）
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_SCRIPTS = os.path.join(_PKG, "scripts")
for _p in (_PKG, _SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402

import aeb_metrics as mt  # noqa: E402
from aeb_controller import AEBController  # noqa: E402
from aeb_env import mujoco_available  # noqa: E402
from runner import AEBConfig, run_scenario  # noqa: E402

HAS_MUJOCO = mujoco_available()


# ---------------------------------------------------------------------------
# 控制律
# ---------------------------------------------------------------------------
class TestController(unittest.TestCase):
    def test_ttc_formula_static_lead(self):
        """前车静止时 TTC = gap / v_ego。"""
        self.assertAlmostEqual(AEBController.compute_ttc(10.0, 5.0, 0.0), 2.0)

    def test_ttc_formula_moving_lead(self):
        """前车同向行驶时用相对速度：10 m 间距、相对速度 2 m/s → 5 s。"""
        self.assertAlmostEqual(AEBController.compute_ttc(10.0, 7.0, 5.0), 5.0)

    def test_ttc_infinite_when_not_closing(self):
        """相对速度 <= 0 时不可能追上，TTC 必须是"无穷"而不是负数/0。"""
        self.assertEqual(AEBController.compute_ttc(10.0, 3.0, 3.0), AEBController.INF_TTC)
        self.assertEqual(AEBController.compute_ttc(10.0, 2.0, 4.0), AEBController.INF_TTC)

    def test_ttc_infinite_when_stopped(self):
        """自车停住时即使间距很小也不该报出很小的 TTC。"""
        self.assertEqual(AEBController.compute_ttc(0.5, 0.0, 0.0), AEBController.INF_TTC)

    def test_brake_increases_as_ttc_drops(self):
        """TTC 越小制动力越大（纯比例律，关闭保持后与公式一致）。"""
        c = AEBController(ttc_threshold=5.0, gain=1.0, latch=False)
        b_far = c.get_brake(4.5, 5.0)
        b_near = c.get_brake(2.5, 5.0)
        self.assertLess(b_far, b_near)
        self.assertAlmostEqual(b_far, 1.0 * (1 - 4.5 / 5.0))
        self.assertAlmostEqual(b_near, 1.0 * (1 - 2.5 / 5.0))

    def test_latch_keeps_minimum_brake(self):
        """触发后即使 TTC 回升（未超解除阈值），制动力也不低于保持下限。"""
        c = AEBController(ttc_threshold=5.0, gain=1.4, latch=True, latch_floor=0.35)
        c.get_brake(2.0, 5.0)                      # 先触发
        b = c.get_brake(4.9, 5.0)                  # TTC 回升但仍在阈值内
        self.assertGreaterEqual(b, 0.35 - 1e-9)

    def test_latch_not_released_while_still_closing(self):
        """低速段 TTC 被放大时**不能**解除保持——否则车会一直蹭着走。"""
        c = AEBController(ttc_threshold=5.0, gain=1.4, latch_floor=0.35)
        c.get_brake(2.0, 5.0)
        b = c.get_brake(8.0, 0.4, closing_speed=0.4)   # TTC 很大，但仍在接近
        self.assertGreaterEqual(b, 0.35 - 1e-9)
        self.assertTrue(c.triggered)

    def test_latch_released_when_lead_pulls_away(self):
        """前车加速拉开距离（相对速度 <= 0）且 TTC 明显恢复时解除保持。"""
        c = AEBController(ttc_threshold=5.0, gain=1.4, latch_floor=0.35)
        c.get_brake(2.0, 5.0)
        b = c.get_brake(9.0, 3.0, closing_speed=-2.0)
        self.assertEqual(b, 0.0)
        self.assertFalse(c.triggered)

    def test_gain_zero_never_triggers(self):
        """gain=0 等价于关掉 AEB：不能触发、更不能被保持下限补出一个制动力。

        回归：早期实现里 `triggered` 只要 TTC 进阈值就置位，于是 gain=0 时
        保持下限仍然生效，基线场景与"无 AEB"场景指标完全相同——对照组直接失效。
        """
        c = AEBController(ttc_threshold=5.0, gain=0.0, latch=True)
        self.assertEqual(c.get_brake(2.0, 5.0), 0.0)
        self.assertFalse(c.triggered)

    def test_brake_zero_above_threshold(self):
        c = AEBController(ttc_threshold=5.0)
        self.assertEqual(c.get_brake(5.0, 5.0), 0.0)
        self.assertEqual(c.get_brake(99.0, 5.0), 0.0)

    def test_brake_clipped_to_unit_interval(self):
        """TTC 极小时增益再大也必须限幅到 1。"""
        c = AEBController(ttc_threshold=5.0, gain=50.0)
        self.assertEqual(c.get_brake(0.001, 5.0), 1.0)
        self.assertGreaterEqual(c.get_brake(4.9, 5.0), 0.0)

    def test_low_speed_guard(self):
        """低速保护：已停住时不输出制动（否则原地抖动）。"""
        c = AEBController(ttc_threshold=5.0, v_stop=0.1)
        self.assertEqual(c.get_brake(0.5, 0.05), 0.0)

    def test_triggered_flag(self):
        c = AEBController(ttc_threshold=5.0, gain=1.4)
        self.assertFalse(c.triggered)
        c.get_brake(2.0, 5.0)
        self.assertTrue(c.triggered)
        c.reset()
        self.assertFalse(c.triggered)


# ---------------------------------------------------------------------------
# 指标
# ---------------------------------------------------------------------------
class TestMetrics(unittest.TestCase):
    def test_first_contact(self):
        self.assertEqual(mt.first_contact([3.0, 1.0, -0.2, 0.5]), 2)
        self.assertEqual(mt.first_contact([3.0, 1.0]), -1)

    def test_brake_onset(self):
        t = [0.0, 0.1, 0.2, 0.3]
        self.assertAlmostEqual(mt.brake_onset(t, [0.0, 0.0, 0.5, 0.7]), 0.2)
        self.assertTrue(math.isnan(mt.brake_onset(t, [0.0, 0.0, 0.0, 0.0])))

    def test_stop_distance(self):
        x = [0.0, 1.0, 2.5, 3.5, 4.0]
        v = [5.0, 4.0, 2.0, 0.0, 0.0]
        self.assertAlmostEqual(mt.stop_distance(x, v, v_stop=0.1), 3.5)

    def test_stop_distance_nan_when_never_stops(self):
        self.assertTrue(math.isnan(mt.stop_distance([0, 1, 2], [5, 4, 3])))

    def test_summarize_collision(self):
        t = [0, 1, 2]
        gap = [10.0, 2.0, -0.5]
        v = [5.0, 4.0, 3.0]
        ttc = [2.0, 0.5, 0.1]
        brake = [0.0, 1.0, 1.0]
        m = mt.summarize(t, gap, v, ttc, brake)
        self.assertTrue(m["collided"])
        self.assertAlmostEqual(m["impact_speed"], 3.0)
        self.assertAlmostEqual(m["min_gap"], -0.5)
        self.assertAlmostEqual(m["brake_onset"], 1.0)
        self.assertFalse(m["stopped"])

    def test_summarize_no_collision(self):
        t = [0, 1, 2]
        gap = [10.0, 5.0, 2.0]
        v = [5.0, 2.0, 0.0]
        ttc = [2.0, 2.5, 9.9]
        brake = [0.0, 0.5, 0.0]
        m = mt.summarize(t, gap, v, ttc, brake)
        self.assertFalse(m["collided"])
        self.assertTrue(math.isnan(m["impact_speed"]))
        self.assertTrue(m["stopped"])


# ---------------------------------------------------------------------------
# 仿真集成（无 mujoco 时跳过）
# ---------------------------------------------------------------------------
@unittest.skipUnless(HAS_MUJOCO, "本机没有 mujoco，跳过仿真集成测试")
class TestSimulation(unittest.TestCase):
    def test_env_exposes_named_joint_addresses(self):
        """回归：原实现用 qpos[11]/qvel[10] 读前车，而模型只有 11 个 qpos → IndexError。

        现在按关节名取地址，且前车有独立滑移关节，两个状态都必须可读。
        """
        from aeb_env import AEBEnv
        env = AEBEnv()
        info = env.describe()
        self.assertGreaterEqual(info["nq"], 12)
        env.reset(ego_x=0.0, ego_v=5.0, lead_x=35.0, lead_v=0.0)
        self.assertAlmostEqual(env.ego_x, 0.0, places=6)
        self.assertAlmostEqual(env.lead_x, 35.0, places=6)
        self.assertAlmostEqual(env.ego_v, 5.0, places=6)
        # 净间距 = 中心距 - 一个车身长
        self.assertAlmostEqual(env.gap, 35.0 - env.BODY_LENGTH, places=6)

    def test_brake_slows_the_car(self):
        from aeb_env import AEBEnv
        env = AEBEnv()
        env.reset(ego_x=0.0, ego_v=5.0, lead_x=100.0, lead_v=0.0)
        for _ in range(200):                 # 0.2 s 全力制动
            env.apply_brake(1.0)
            env.step()
        self.assertLess(env.ego_v, 5.0)

    def test_baseline_does_not_collide(self):
        """基线工况（5 m/s、静止前车、TTC 阈值 5 s）必须停住且不追尾。"""
        _track, m = run_scenario(AEBConfig(ego_v=5.0, gain=1.4, ttc_threshold=5.0))
        self.assertFalse(m["collided"], m)
        self.assertTrue(m["stopped"], m)
        self.assertGreater(m["min_gap"], 0.5)

    def test_aeb_off_collides(self):
        """对照组：gain=0 相当于关掉 AEB，同工况必须追尾。

        没有这一条，"刹住了"可能只是因为这车本来就要停——对照才能说明 AEB 起了作用。
        """
        _track, m = run_scenario(AEBConfig(ego_v=5.0, gain=0.0, ttc_threshold=5.0))
        self.assertTrue(m["collided"], m)

    def test_higher_speed_needs_stronger_braking(self):
        """初速度越高，制动距离越长（单调性的粗略检查）。"""
        _t1, m1 = run_scenario(AEBConfig(ego_v=5.0), "v5")
        _t2, m2 = run_scenario(AEBConfig(ego_v=8.0), "v8")
        self.assertGreater(m2["stop_distance"], m1["stop_distance"])


# ---------------------------------------------------------------------------
# 仓库约定
# ---------------------------------------------------------------------------
class TestRepoConventions(unittest.TestCase):
    def test_required_files_exist(self):
        for rel in ("main.py", "main.sh" if os.name != "nt" else "main.bat",
                    "package.xml", "CMakeLists.txt", "requirements.txt",
                    "README.md", "launch/main.launch", "models/aeb_car.xml"):
            self.assertTrue(os.path.isfile(os.path.join(_PKG, rel)), f"缺少 {rel}")

    def test_launch_declares_args_and_uses_them(self):
        """launch 里声明的参数必须真的出现在节点 args 中。"""
        import re
        text = open(os.path.join(_PKG, "launch", "main.launch"), encoding="utf-8").read()
        declared = set(re.findall(r'<arg\s+name="([^"]+)"', text))
        used = set(re.findall(r"\$\(arg\s+([^)]+)\)", text)) - {"gui"}
        self.assertTrue(declared)
        self.assertTrue(declared - {"gui"} <= used,
                        f"声明了但没传给节点：{sorted(declared - used)}")

    def test_model_has_named_joints(self):
        """模型必须给自车/前车关节命名，否则又只能靠下标猜。"""
        text = open(os.path.join(_PKG, "models", "aeb_car.xml"), encoding="utf-8").read()
        self.assertIn('name="ego_free"', text)
        self.assertIn('name="lead_slide"', text)

    def test_no_personal_email_in_package_xml(self):
        """package.xml 里不该出现个人邮箱（提交记录里已有作者信息）。"""
        text = open(os.path.join(_PKG, "package.xml"), encoding="utf-8").read()
        import re
        emails = re.findall(r"[\w.+-]+@[\w-]+\.[\w.]+", text)
        self.assertTrue(all(e.endswith("example.com") for e in emails), emails)


if __name__ == "__main__":
    unittest.main(verbosity=2)
