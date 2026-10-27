# -*- coding: utf-8 -*-
"""用"桩环境"端到端验证 runner / metrics / CSV 导出（不需要 MuJoCo）。

桩环境实现与 CarEnv 相同的接口（duck typing），内部是一个理想速度跟踪的一维小车。
它验证的是闭环节点之间的接线是否正确（属性名、状态比较、指标字段、导出路径），
MuJoCo 的接触/摩擦/射线求交仍由 test_scene_and_sensors.py 覆盖。
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

import math

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_avoidance import config                       # noqa: E402
from mujoco_car_avoidance.car_env import CarState             # noqa: E402
from mujoco_car_avoidance.decision import DecisionParams      # noqa: E402
from mujoco_car_avoidance.runner import AvoidanceRunner       # noqa: E402

CENTER = (config.LIDAR_ANGLES_DEG[0] + config.LIDAR_ANGLES_DEG[-1]) / 2.0


class StubLidar:
    """与 sensors.LidarFan 接口一致的最小实现。"""

    def __init__(self):
        self.max_range = config.LIDAR_MAX_RANGE_M
        self._value = None

    def min_distance(self, ranges=None):
        return self._value


class StubEnv:
    """理想速度跟踪的两轮小车（无物理引擎）。"""

    def __init__(self, initial_distance=7.5, control_hz=config.CONTROL_HZ):
        self.timestep = config.TIMESTEP
        self.control_hz = float(control_hz)
        self.steps_per_control = max(1, int(round(1.0 / (self.control_hz * self.timestep))))
        self.lidar = StubLidar()
        self.initial_distance = float(initial_distance)
        self.reset()

    # ------------------------------------------------------------------ 接口
    @property
    def time(self):
        return self._t

    def reset(self):
        self._t = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._ctrl = (0.0, 0.0)
        self.dropout_frames = 0          #: 模拟传感器连续丢帧的帧数
        self.lidar._value = self.initial_distance

    def scan(self):
        # 丢帧期间本车道无有效测距（min_distance 返回 None）
        if self.dropout_frames > 0:
            self.dropout_frames -= 1
            self.lidar._value = None
            return np.array([self.lidar.max_range], dtype=np.float64)

        value = max(self.initial_distance - self._x, 0.0)
        self.lidar._value = value
        return np.array([value], dtype=np.float64)

    def sensed_distance(self):
        return max(self.initial_distance - self._x, 0.0)

    def set_wheel_speeds(self, left_omega, right_omega):
        self._ctrl = (float(left_omega), float(right_omega))
        # 理想速度跟踪：轮速指令立刻生效（真实环境由速度伺服跟踪）
        self._v = 0.5 * (self._ctrl[0] + self._ctrl[1]) * config.WHEEL_RADIUS_M

    def advance_control_period(self):
        dt = self.steps_per_control * self.timestep
        self._x += self._v * dt
        self._t += dt
        self.lidar._value = max(self.initial_distance - self._x, 0.0)

    def step(self, count=1):
        """推进 count 个物理步长（与 CarEnv.step 语义一致，供 runner.settle() 使用）。"""
        dt = int(count) * self.timestep
        self._x += self._v * dt
        self._t += dt
        self.lidar._value = max(self.initial_distance - self._x, 0.0)

    def state(self):
        return CarState(t=self._t, x=self._x, y=self._y, yaw=self._yaw,
                        v=self._v, yaw_rate=0.0,
                        wheel_left=self._ctrl[0], wheel_right=self._ctrl[1])

    def truth_clearance(self):
        return max(self.initial_distance - self._x, 0.0)

    def has_obstacle_contact(self):
        return (self.initial_distance - self._x) <= 0.0


class TestRunnerWithStub(unittest.TestCase):

    def setUp(self):
        self.env = StubEnv()
        self.runner = AvoidanceRunner(self.env, params=DecisionParams())

    def test_run_completes_and_stops_safely(self):
        recorder = self.runner.run(30.0)
        summary = recorder.summary()
        self.assertFalse(summary["是否碰撞"])
        self.assertIsNotNone(summary["c_停车(m)"])
        self.assertGreater(summary["c_停车(m)"], 0.0)
        self.assertAlmostEqual(summary["c_停车(m)"], DecisionParams.stop_target_m, delta=0.4)

    def test_stop_metrics_ignore_initial_standstill(self):
        """停车指标必须来自"真正起步之后的停稳"，而不是起步瞬间的静止。

        回归背景：停车判定原本只要求 |v| < 0.02 且 t > 0.5，而车静止起步时
        t=0.62 s 的速度就是 0.005 m/s，于是立刻被判成"已停车"：
        c_停车 被记成初始间距 7.48 m、t_停车 记成 0.62 s（真机实测踩过这个坑）。
        """
        recorder = self.runner.run(30.0)
        summary = recorder.summary()

        self.assertIsNotNone(summary["t_停车(s)"])
        self.assertGreater(summary["t_停车(s)"], 3.0,
                           "停车时刻 {:.2f}s 过早：疑似把起步静止段当成了停车".format(
                               summary["t_停车(s)"]))
        self.assertLess(summary["c_停车(m)"], 3.0,
                        "停车间距 {:.2f} m 过大：疑似记成了初始间距".format(
                            summary["c_停车(m)"]))
        # 停车间距应等于"最终停住的位置"，而不是接近段某次瞬时低速的位置
        self.assertAlmostEqual(
            summary["c_停车(m)"], summary["c_最小(m)"], delta=0.3,
            msg="停车间距 {:.2f} m 与最小间距 {:.2f} m 相差过大："
                "疑似被接近段的瞬时停顿误触发".format(
                    summary["c_停车(m)"], summary["c_最小(m)"]))

    def test_metrics_are_populated(self):
        recorder = self.runner.run(30.0)
        summary = recorder.summary()
        self.assertGreater(summary["采样帧数"], 10)
        self.assertAlmostEqual(summary["v_最大(m/s)"], DecisionParams.cruise_speed, places=2)
        self.assertIsNotNone(summary["d_触发减速(m)"])
        self.assertGreater(summary["a_最大制动(m/s^2)"], 0.0)

    def test_realtime_factor_is_positive(self):
        self.runner.run(5.0)
        self.assertGreater(self.runner.realtime_factor, 0.0)

    def test_csv_and_plot_export(self):
        recorder = self.runner.run(10.0)
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "run_log.csv"
            png_path = Path(tmp) / "run_curves.png"
            recorder.write_csv(csv_path)
            self.assertTrue(csv_path.is_file())
            self.assertGreater(csv_path.stat().st_size, 100)
            plotted = recorder.plot(png_path)          # matplotlib 缺失时返回 False，不算失败
            if plotted:
                self.assertTrue(png_path.is_file())

    def test_reset_clears_state(self):
        self.runner.run(5.0)
        self.runner.reset()
        self.assertEqual(len(self.runner.recorder.samples), 0)
        self.assertAlmostEqual(self.env.time, 0.0, places=9)

    def test_manual_command_is_capped_by_avoidance(self):
        """手动请求 5 m/s 时，减速区应把速度压到 target_speed 以下。"""
        self.env = StubEnv(initial_distance=3.0)
        runner = AvoidanceRunner(self.env, params=DecisionParams())
        info = None
        for _ in range(200):
            info = runner.step_once(manual_cmd=(5.0, 0.0))
        self.assertIsNotNone(info)
        self.assertLessEqual(info.v_cmd, DecisionParams.cruise_speed + 1e-6)
        self.assertFalse(info.collision)

    def test_emergency_stop_on_immediate_obstacle(self):
        self.env = StubEnv(initial_distance=0.5)
        runner = AvoidanceRunner(self.env, params=DecisionParams())
        info = runner.step_once()
        self.assertEqual(info.state.value, "FULL_BRAKE")
        self.assertAlmostEqual(info.v_cmd, 0.0, places=9)

    def test_single_missing_measurement_does_not_brake(self):
        """单帧测距丢失不应立刻急刹。

        回归背景：早期实现里"拿不到测距但还没超时"直接返回 FULL_BRAKE，
        轮速被瞬间拍死，反作用力矩让车头俯仰，进而使测距更不可靠，形成恶性循环。
        """
        runner = AvoidanceRunner(self.env, params=DecisionParams())
        for _ in range(25):                      # 先正常行驶 0.5 s
            runner.step_once()

        self.env.dropout_frames = 3              # 连续丢 3 帧（0.06 s < 超时 0.5 s）
        info = None
        for _ in range(3):
            info = runner.step_once()

        self.assertNotEqual(info.state.value, "STALE")
        self.assertGreater(info.v_cmd, 0.0, "丢帧期间不应把速度指令清零")
        self.assertTrue(info.held, "应标记为沿用了上一次有效测距")

    def test_long_dropout_eventually_goes_stale(self):
        """持续丢失超过超时窗口后应进入 STALE 并刹停。"""
        runner = AvoidanceRunner(self.env, params=DecisionParams())
        for _ in range(25):
            runner.step_once()

        self.env.dropout_frames = 60             # 1.2 s > stale_timeout 0.5 s
        info = None
        for _ in range(40):
            info = runner.step_once()

        self.assertEqual(info.state.value, "STALE")
        self.assertAlmostEqual(info.v_cmd, 0.0, places=9)

    def test_settle_keeps_car_still_then_drives_normally(self):
        """静置阶段不应移动车辆，静置后第一帧应能正常起步（不受"无测距"影响）。"""
        self.runner.settle(0.6)
        self.assertAlmostEqual(self.env.time, 0.6, places=6)
        self.assertAlmostEqual(self.env.state().x, 0.0, places=9)

        info = self.runner.step_once()
        self.assertEqual(info.state.value, "CRUISE")
        self.assertGreater(info.v_cmd, 0.0)

    def test_heading_correction_reduces_yaw(self):
        """航向保持的符号必须让偏航减小。

        回归背景：符号写反时它会变成正反馈——车头越偏、修正量越大且方向相同，
        实测把偏航从 10° 推到了 29°，进而让路侧干扰物进入本车道并触发了提前刹停。
        约定：differential_drive 中正角速度 = 左转，因此车头左偏时必须输出负角速度。
        """
        runner = AvoidanceRunner(self.env)          # 参考航向 = 0
        yaw_left = math.radians(5.0)
        yaw_right = math.radians(-5.0)

        self.assertLess(runner._heading_correction(yaw_left), 0.0,
                        "车头左偏时应输出负角速度（向右修正）")
        self.assertGreater(runner._heading_correction(yaw_right), 0.0,
                           "车头右偏时应输出正角速度（向左修正）")

        # 修正量与偏差成正比，且被限幅
        self.assertAlmostEqual(runner._heading_correction(yaw_left),
                               -runner.heading_hold_gain * yaw_left, places=9)
        self.assertLessEqual(abs(runner._heading_correction(math.radians(90.0))),
                             runner.heading_hold_limit + 1e-9)

    def test_heading_correction_can_be_disabled(self):
        runner = AvoidanceRunner(self.env, heading_hold_gain=0.0)
        self.assertEqual(runner._heading_correction(math.radians(10.0)), 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
