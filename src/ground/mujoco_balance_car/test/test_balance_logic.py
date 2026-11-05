#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单元测试：纯逻辑（PID/指标/控制器）+ 仿真集成（无 mujoco 时自动跳过）。

运行：

    python3 test/test_balance_logic.py

约定：**没有 mujoco 的机器也要能跑**——逻辑部分的用例不导入 mujoco，
仿真集成的用例显式 `skipUnless(mujoco_available())`，避免"缺依赖"被误报成
"代码有 bug"。
"""

import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from mujoco_balance_car import metrics                      # noqa: E402
from mujoco_balance_car.config import BalanceConfig, ScenarioConfig  # noqa: E402
from mujoco_balance_car.controller import BalanceController  # noqa: E402
from mujoco_balance_car.env import mujoco_available          # noqa: E402
from mujoco_balance_car.pid import PIDController             # noqa: E402

HAS_MUJOCO = mujoco_available()


# ---------------------------------------------------------------------------
# PID
# ---------------------------------------------------------------------------
class TestPID(unittest.TestCase):
    def test_output_is_clamped(self):
        """输出必须被限幅：电机 ctrlrange 是 ±5，超了会被 MuJoCo 静默截断。"""
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, output_min=-2.0, output_max=2.0)
        out = pid.compute(1.0, 0.0, 0.002)
        self.assertAlmostEqual(out, 2.0)

    def test_integral_is_limited(self):
        """积分限幅：长时间饱和时积分项不能无限涨（否则退出饱和会大幅超调）。"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_min=-1.0, output_max=1.0)
        for _ in range(5000):
            pid.compute(1.0, 0.0, 0.002)
        # ki=1、输出限幅 1 → 积分上限应为 1
        self.assertLessEqual(abs(pid.integral), 1.0 + 1e-9)

    def test_reset_clears_state(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0)
        pid.compute(1.0, 0.0, 0.002)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertIsNone(pid.prev_measurement)

    def test_derivative_kick_on_step(self):
        """目标阶跃时：'对误差求导'会有尖峰，'对测量值求导'不会——这正是位置外环
        默认用后者的原因。"""
        kick = PIDController(kp=0.0, ki=0.0, kd=1.0, output_min=-1e9, output_max=1e9)
        kick.compute(0.0, 0.0, 0.002)          # 先建立上一拍状态
        out_with_kick = kick.compute(1.0, 0.0, 0.002)   # 目标突跳 → 误差突跳

        smooth = PIDController(kp=0.0, ki=0.0, kd=1.0, output_min=-1e9, output_max=1e9,
                               derivative_on_measurement=True)
        smooth.compute(0.0, 0.0, 0.002)
        out_smooth = smooth.compute(1.0, 0.0, 0.002)    # 测量值没变 → 微分为 0
        self.assertGreater(out_with_kick, 100.0)
        self.assertAlmostEqual(out_smooth, 0.0)

    def test_invalid_dt_raises(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        with self.assertRaises(ValueError):
            pid.compute(1.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# 指标
# ---------------------------------------------------------------------------
class TestMetrics(unittest.TestCase):
    def test_settle_time_basic(self):
        t = [0, 1, 2, 3, 4]
        y = [5.0, 2.0, 0.5, 0.2, 0.1]
        self.assertAlmostEqual(metrics.settle_time(t, y, 0.0, 1.0), 2.0)

    def test_settle_time_nan_when_never_settles(self):
        t = [0, 1, 2]
        y = [5.0, 4.0, 3.0]
        self.assertTrue(math.isnan(metrics.settle_time(t, y, 0.0, 0.5)))

    def test_settle_time_requires_holding_band(self):
        """中途出带再次进入时，稳定时间应取**最后一次**进带的时刻。"""
        t = [0, 1, 2, 3]
        y = [0.1, 0.1, 5.0, 0.1]
        self.assertAlmostEqual(metrics.settle_time(t, y, 0.0, 0.5), 3.0)

    def test_overshoot(self):
        self.assertAlmostEqual(metrics.peak_overshoot([0.0, 1.0, 1.2, 1.0], 1.0, 0.0), 20.0)
        # 反向行程
        self.assertAlmostEqual(metrics.peak_overshoot([0.0, -1.0, -1.1, -1.0], -1.0, 0.0), 10.0)
        # 行程为 0 时不应出现 inf
        self.assertEqual(metrics.peak_overshoot([1.0, 1.0], 1.0), 0.0)

    def test_steady_state_error(self):
        t = list(range(10))
        y = [1.0] * 9 + [1.5]
        self.assertAlmostEqual(metrics.steady_state_error(t, y, 1.0, frac=0.1), 0.5)


# ---------------------------------------------------------------------------
# 控制器
# ---------------------------------------------------------------------------
class TestController(unittest.TestCase):
    def test_target_lean_is_limited(self):
        ctrl = BalanceController(BalanceConfig())
        ctrl.set_target(100.0)          # 远到超出外环能力
        _torque, target_lean = ctrl.compute(x=0.0, angle=0.0, dt=0.002)
        cfg = ctrl.cfg
        self.assertLessEqual(abs(target_lean), max(abs(cfg.lean_max), abs(cfg.lean_min)) + 1e-12)

    def test_positive_position_error_leans_forward(self):
        """目标在前方（x 误差为正）时，外环给出的目标倾角应为正（向前倾）。"""
        ctrl = BalanceController(BalanceConfig())
        ctrl.set_target(1.0)
        _t, target_lean = ctrl.compute(x=0.0, angle=0.0, dt=0.002)
        self.assertGreater(target_lean, 0.0)

    def test_torque_is_limited(self):
        ctrl = BalanceController(BalanceConfig())
        torque, _ = ctrl.compute(x=0.0, angle=0.5, dt=0.002)   # 大倾角
        self.assertLessEqual(abs(torque), max(abs(ctrl.cfg.torque_max),
                                             abs(ctrl.cfg.torque_min)) + 1e-12)

    def test_reset_clears_both_loops(self):
        ctrl = BalanceController(BalanceConfig())
        ctrl.compute(x=0.5, angle=0.2, dt=0.002)
        ctrl.reset()
        self.assertEqual(ctrl.angle_pid.integral, 0.0)
        self.assertEqual(ctrl.pos_pid.integral, 0.0)


# ---------------------------------------------------------------------------
# 仿真集成（无 mujoco 时跳过）
# ---------------------------------------------------------------------------
@unittest.skipUnless(HAS_MUJOCO, "本机没有 mujoco，跳过仿真集成测试")
class TestSimulation(unittest.TestCase):
    def test_env_shapes_and_state(self):
        from mujoco_balance_car.env import BalanceEnv
        env = BalanceEnv()
        info = env.describe()
        self.assertEqual(info["nu"], 2)          # 两个电机
        self.assertAlmostEqual(env.timestep, 0.002, places=6)

        env.reset(lean=0.1, x=0.3)
        self.assertAlmostEqual(env.angle, 0.1, places=6)
        self.assertAlmostEqual(env.x, 0.3, places=6)

    def test_disturb_changes_velocity(self):
        from mujoco_balance_car.env import BalanceEnv
        env = BalanceEnv()
        env.reset(lean=0.0, x=0.0, velocity=0.0)
        env.disturb(1.5)
        self.assertAlmostEqual(env.vx, 1.5, places=3)

    def test_balance_scenario_converges(self):
        """默认 PID 参数必须能从 0.05 rad 恢复：稳定时间有限且末态直立。

        这是本模块最关键的一条集成测试——调参把增益改坏、或马达符号写反，
        都会在这里红。
        """
        from mujoco_balance_car.runner import run_balance
        res = run_balance(BalanceConfig(duration=6.0, initial_lean=0.05))
        self.assertTrue(res.metrics["upright"], f"末态未直立: {res.metrics}")
        self.assertFalse(math.isnan(res.metrics["angle_settle"]), f"未稳定: {res.metrics}")
        self.assertLess(res.metrics["angle_settle"], 6.0)
        self.assertLess(abs(res.metrics["final_angle"]), 0.02)

    def test_position_step_reaches_target(self):
        from mujoco_balance_car.runner import run_position_step
        cfg = BalanceConfig(duration=8.0)
        sc = ScenarioConfig(settle_window=8.0, step_position=1.0)
        res = run_position_step(cfg, sc)
        self.assertTrue(res.metrics["reached"], f"未到达目标: {res.metrics}")
        self.assertLess(abs(res.metrics["final_pos"] - 1.0), 0.1)

    def test_metrics_are_finite_for_disturbance(self):
        from mujoco_balance_car.runner import run_disturbance
        cfg = BalanceConfig(duration=8.0)
        sc = ScenarioConfig(settle_window=8.0)
        res = run_disturbance(cfg, sc)
        self.assertTrue(math.isfinite(res.metrics["pos_max_dev"]))
        self.assertGreater(res.metrics["pos_max_dev"], 0.01)   # 确实被推动了


# ---------------------------------------------------------------------------
# 仓库约定
# ---------------------------------------------------------------------------
class TestRepoConventions(unittest.TestCase):
    def test_entry_and_launch_exist(self):
        for rel in ("main.py", "main.sh", "main.bat", "package.xml", "setup.py",
                    "launch/main.launch.py", "balance_car.xml", "README.md"):
            path = os.path.join(_PKG_ROOT, rel)
            self.assertTrue(os.path.isfile(path), f"缺少约定文件：{rel}")

    def test_launch_args_are_declared_and_used(self):
        """launch 里 `DeclareLaunchArgument` 声明的参数必须真的被传给进程。

        ROS 2 对未声明的 launch 参数不报错、直接忽略；反过来"声明了却没用"
        会让使用者以为配置生效了，实际还是默认值。
        """
        path = os.path.join(_PKG_ROOT, "launch", "main.launch.py")
        with open(path, encoding="utf-8") as f:
            text = f.read()
        import re
        declared = set(re.findall(r"DeclareLaunchArgument\(\s*\n?\s*['\"]([^'\"]+)", text))
        used = set(re.findall(r"LaunchConfiguration\(\s*['\"]([^'\"]+)", text))
        self.assertTrue(declared, "launch 里没有声明任何参数")
        self.assertTrue(declared <= used, f"声明了但没使用：{sorted(declared - used)}")

    def test_pid_shim_still_importable(self):
        """老的 `import pid_controller` 用法不能被重构弄坏。"""
        sys.path.insert(0, _PKG_ROOT)
        import pid_controller
        self.assertTrue(hasattr(pid_controller, "PIDController"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
