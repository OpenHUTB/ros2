#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``speed_controller`` ROS 节点的单元测试。

在没有 ROS 的机器上通过桩模块（stub）替代 rospy / std_msgs 等，
以便在普通开发机上也能验证节点逻辑：

* 参数加载与默认值
* 曲率话题超时后退化为直道（而不是沿用旧曲率）
* 限速取"巡航速度"与"曲率限速"的较小值
* 发布的消息字段与顺序
* 非有限曲率的容错
* 未接里程计时的速度估计与告警

注意：这些测试验证的是**节点逻辑**，不能替代真实 ROS 环境下的集成测试。
"""

from __future__ import annotations

import math
import os
import sys
import types
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.abspath(os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, SCRIPTS)


# ---------------------------------------------------------------------------
# ROS 桩模块
# ---------------------------------------------------------------------------


class _Time:
    """时间戳。支撑节点用到的减法与 to_sec()。"""

    def __init__(self, secs=0.0):
        self._secs = float(secs)

    def to_sec(self):
        return self._secs

    def __sub__(self, other):
        return _Time(self._secs - other._secs)

    def __add__(self, other):
        if isinstance(other, _Time):
            return _Time(self._secs + other._secs)
        return _Time(self._secs + float(other))


class _TimeFactory:
    """模拟 ``rospy.Time``。

    真实 ROS 中 ``rospy.Time`` 是一个**类**，用法有两种：

    * ``rospy.Time(secs)``  —— 构造时间戳
    * ``rospy.Time.now()``  —— 取当前时间（类方法）

    因此桩必须同时可调用、又带有 ``now()`` 静态方法。
    早期版本把它写成普通方法，导致 ``rospy.Time.now()`` 抛
    ``AttributeError: 'function' object has no attribute 'now'``。
    """

    _clock = {"now": 0.0}

    def __call__(self, secs=0.0):
        return _Time(secs)

    @classmethod
    def now(cls):  # noqa: N802  与 ROS 接口同名
        return _Time(cls._clock["now"])

    @classmethod
    def set_now(cls, secs):
        cls._clock["now"] = float(secs)

    @classmethod
    def advance(cls, dt):
        cls._clock["now"] += float(dt)


class _FakeRosPy(types.ModuleType):
    def __init__(self):
        super().__init__("rospy")
        self.params = {}
        self.published = []      # [(topic, msg)]
        self.subscribers = {}    # topic -> callback
        self.logged = []
        self._now = 0.0
        self.shutdown = False

    # -- 参数 --
    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def set_param(self, name, value):
        self.params[name] = value

    # -- 时间 --
    Time = _TimeFactory  # noqa: N815  与 ROS 接口同名

    def now(self):
        return _Time(self._now)

    def advance(self, dt):
        self._now += float(dt)
        _TimeFactory.set_now(self._now)

    # -- 话题 --
    def Publisher(self, topic, msg_type, queue_size=1):  # noqa: N802
        rospy_ref = self

        class _Pub:
            def __init__(self, t):
                self.topic = t

            def publish(self, msg):
                rospy_ref.published.append((self.topic, msg))

        return _Pub(topic)

    def Subscriber(self, topic, msg_type, callback, queue_size=1):  # noqa: N802
        self.subscribers[topic] = callback
        return callback

    # -- 日志 --
    def loginfo(self, msg, *a):
        self.logged.append(("info", msg % a if a else msg))

    def logwarn(self, msg, *a):
        self.logged.append(("warn", msg % a if a else msg))

    def logwarn_throttle(self, period, msg, *a):
        self.logged.append(("warn_throttle", msg % a if a else msg))

    def logerr(self, msg, *a):
        self.logged.append(("err", msg % a if a else msg))

    def init_node(self, *a, **kw):
        pass

    def is_shutdown(self):
        return self.shutdown

    def Rate(self, hz):  # noqa: N802
        class _Rate:
            def sleep(self):
                pass

        return _Rate()


class _Msg(types.SimpleNamespace):
    """通用消息桩。"""


class _Float32(_Msg):
    def __init__(self, data=0.0):
        super().__init__(data=float(data))


class _Float32MultiArray(_Msg):
    def __init__(self, data=None):
        super().__init__(data=list(data) if data is not None else [])


class _Twist(_Msg):
    def __init__(self):
        super().__init__(
            linear=_Msg(x=0.0, y=0.0, z=0.0),
            angular=_Msg(x=0.0, y=0.0, z=0.0),
        )


def install_stubs():
    """安装 ROS 桩模块，返回 fake rospy 实例。"""
    fake = _FakeRosPy()
    sys.modules["rospy"] = fake

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.Float32 = _Float32
    std_msgs.Float32MultiArray = _Float32MultiArray
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    std_msgs.msg.Float32 = _Float32
    std_msgs.msg.Float32MultiArray = _Float32MultiArray
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs.msg

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg.Twist = _Twist
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs.msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs.msg = types.ModuleType("nav_msgs.msg")
    nav_msgs.msg.Odometry = _Msg
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs.msg

    return fake


ROSPY = install_stubs()

# 桩必须在 import 节点之前装好
import speed_controller  # noqa: E402


class NodeTestCase(unittest.TestCase):
    """所有节点测试的公共准备：清空参数与发布记录。"""

    def setUp(self):
        ROSPY.params.clear()
        ROSPY.published.clear()
        ROSPY.subscribers.clear()
        ROSPY.logged.clear()
        ROSPY._now = 0.0
        _TimeFactory.set_now(0.0)
        ROSPY.shutdown = False

    def make_node(self, **params):
        for k, v in params.items():
            ROSPY.params["~" + k] = v
        return speed_controller.SpeedControllerNode()

    def last_msg(self, topic):
        for t, m in reversed(ROSPY.published):
            if t == topic:
                return m
        return None


class TestParameterLoading(NodeTestCase):
    """参数加载。"""

    def test_defaults_when_no_params(self):
        node = self.make_node()
        self.assertAlmostEqual(node.target_speed, speed_controller.DEFAULTS["target_speed"])
        self.assertAlmostEqual(node.config.limits.max_lateral_accel, 3.0)
        self.assertAlmostEqual(node.config.limits.max_jerk, 4.0)

    def test_params_override_defaults(self):
        node = self.make_node(
            target_speed=8.0, max_lateral_accel=2.5, max_accel=2.0, max_jerk=3.0
        )
        self.assertAlmostEqual(node.target_speed, 8.0)
        self.assertAlmostEqual(node.config.limits.max_lateral_accel, 2.5)
        self.assertAlmostEqual(node.config.limits.max_accel, 2.0)
        self.assertAlmostEqual(node.config.limits.max_jerk, 3.0)

    def test_invalid_params_raise(self):
        with self.assertRaises(ValueError):
            # max_accel 必须为正
            self.make_node(max_accel=0.0)


class TestCurvatureHandling(NodeTestCase):
    """曲率收取与超时处理。"""

    def test_curvature_updates_limit(self):
        node = self.make_node(target_speed=12.0, max_lateral_accel=3.0)
        node.curvature_stamp = ROSPY.now()  # 标记为"刚收到"
        node.curvature = 0.025             # 半径 40 m
        node.step(0.05)
        v_limit = self.last_msg("speed/limit").data
        expected = 12.0 ** 0  # 占位，下面用公式算
        from speed_control import speed_limit_from_curvature

        expected = speed_limit_from_curvature(0.025, 3.0, 1.0)
        self.assertAlmostEqual(v_limit, expected, places=6)

    def test_curvature_timeout_falls_back_to_straight(self):
        """曲率超时后必须按直道处理，不能沿用旧曲率。

        这是关键行为：若在直道上沿用弯道限速，车辆会无缘无故慢速行驶。
        """
        node = self.make_node(target_speed=12.0)
        node.curvature = 0.05            # 很急的弯
        node.curvature_stamp = ROSPY.now()
        # 时间推进超过超时阈值
        ROSPY.advance(node.curvature_timeout + 0.1)
        node.step(0.05)
        v_limit = self.last_msg("speed/limit").data
        # 应恢复为巡航速度（直道不构成限速）
        self.assertAlmostEqual(v_limit, 12.0, places=6)

    def test_curvature_just_before_timeout_still_active(self):
        node = self.make_node(target_speed=12.0, curvature_timeout=1.0)
        node.curvature = 0.025
        node.curvature_stamp = ROSPY.now()
        ROSPY.advance(0.9)               # 未超时
        node.step(0.05)
        v_limit = self.last_msg("speed/limit").data
        self.assertLess(v_limit, 12.0)

    def test_non_finite_curvature_treated_as_straight(self):
        node = self.make_node(target_speed=12.0)
        node.on_curvature(_Float32(float("nan")))
        self.assertAlmostEqual(node.curvature, 0.0)
        node.on_curvature(_Float32(float("inf")))
        self.assertAlmostEqual(node.curvature, 0.0)

    def test_target_speed_ignores_non_positive(self):
        node = self.make_node(target_speed=12.0)
        node.on_target(_Float32(-5.0))
        self.assertAlmostEqual(node.target_speed, 12.0)
        node.on_target(_Float32(8.0))
        self.assertAlmostEqual(node.target_speed, 8.0)


class TestLimitIsMinOfConstraints(NodeTestCase):
    """限速取巡航速度与曲率限速的较小值。"""

    def test_straight_uses_cruise(self):
        node = self.make_node(target_speed=10.0)
        node.curvature = 0.0
        node.curvature_stamp = ROSPY.now()
        node.step(0.05)
        self.assertAlmostEqual(self.last_msg("speed/limit").data, 10.0, places=6)

    def test_sharp_curve_uses_curvature_limit(self):
        node = self.make_node(target_speed=30.0, max_lateral_accel=3.0)
        node.curvature = 0.025
        node.curvature_stamp = ROSPY.now()
        node.step(0.05)
        v = self.last_msg("speed/limit").data
        self.assertLess(v, 30.0)
        self.assertAlmostEqual(v, math.sqrt(3.0 / 0.025) * 0.995, places=4)

    def test_cruise_wins_when_curve_is_gentle(self):
        node = self.make_node(target_speed=6.0, max_lateral_accel=3.0)
        node.curvature = 0.001           # 很缓的弯，限速远高于 6
        node.curvature_stamp = ROSPY.now()
        node.step(0.05)
        self.assertAlmostEqual(self.last_msg("speed/limit").data, 6.0, places=6)


class TestPublishedMessages(NodeTestCase):
    """发布的消息与字段。"""

    def test_publishes_all_topics(self):
        node = self.make_node()
        node.curvature_stamp = ROSPY.now()
        node.step(0.05)
        topics = {t for t, _ in ROSPY.published}
        for expected in ("speed/limit", "speed/command", "speed/control", "speed/diagnostics"):
            self.assertIn(expected, topics)

    def test_diagnostics_field_order(self):
        node = self.make_node(target_speed=12.0)
        node.curvature = 0.025
        node.curvature_stamp = ROSPY.now()
        node.current_speed = 5.0
        node.has_odom = True
        node.step(0.05)
        d = self.last_msg("speed/diagnostics").data
        self.assertEqual(len(d), 5)
        self.assertAlmostEqual(d[0], 5.0, places=6)             # 当前速度
        self.assertLess(d[1], 12.0)                             # 限速受曲率限制
        self.assertGreater(d[2], 0.0)                           # 应加速
        self.assertGreater(d[3], 0.0)                           # 横向加速度
        self.assertAlmostEqual(d[4], 1.0, places=6)             # 限速生效标志

    def test_no_limit_flag_on_straight(self):
        node = self.make_node(target_speed=12.0)
        node.curvature = 0.0
        node.curvature_stamp = ROSPY.now()
        node.current_speed = 12.0
        node.has_odom = True
        node.step(0.05)
        self.assertAlmostEqual(self.last_msg("speed/diagnostics").data[4], 0.0, places=6)

    def test_twist_carries_current_speed(self):
        node = self.make_node()
        node.current_speed = 7.5
        node.has_odom = True
        node.step(0.05)
        self.assertAlmostEqual(self.last_msg("speed/control").linear.x, 7.5, places=6)

    def test_invert_command_flips_sign(self):
        node = self.make_node(invert_command=True, target_speed=20.0)
        node.current_speed = 0.0
        node.has_odom = True
        node.curvature_stamp = ROSPY.now()
        node.step(0.05)
        self.assertLess(self.last_msg("speed/command").data, 0.0)


class TestSafetyLayers(NodeTestCase):
    """安全层与限幅。"""

    def test_acceleration_within_limits(self):
        node = self.make_node(max_accel=3.0, max_decel=6.0, max_jerk=4.0, target_speed=50.0)
        node.current_speed = 0.0
        node.has_odom = True
        node.curvature_stamp = ROSPY.now()
        for _ in range(20):
            node.step(0.05)
            a = self.last_msg("speed/command").data
            self.assertLessEqual(a, 3.0 + 1e-6)
            self.assertGreaterEqual(a, -6.0 - 1e-6)

    def test_jerk_is_limited_across_steps(self):
        """相邻两拍的加速度变化不得超过 max_jerk·dt。"""
        node = self.make_node(max_jerk=4.0, target_speed=50.0, kp=50.0, ki=0.0, kd=0.0)
        node.current_speed = 0.0
        node.has_odom = True
        node.curvature_stamp = ROSPY.now()
        prev = None
        for _ in range(30):
            node.step(0.05)
            a = self.last_msg("speed/command").data
            if prev is not None:
                self.assertLessEqual(abs(a - prev), 4.0 * 0.05 + 1e-6)
            prev = a

    def test_overspeed_triggers_braking(self):
        node = self.make_node(target_speed=5.0, overspeed_tolerance=0.2)
        node.curvature_stamp = ROSPY.now()
        node.curvature = 0.0
        node.current_speed = 20.0        # 远超限速
        node.has_odom = True
        node.step(0.05)
        self.assertLess(self.last_msg("speed/command").data, 0.0)

    def test_no_odom_estimates_speed_and_warns(self):
        node = self.make_node(target_speed=12.0)
        node.curvature_stamp = ROSPY.now()
        node.step(0.05)
        # 速度应由指令积分得到，且大于 0
        self.assertGreater(node.current_speed, 0.0)
        warnings = [m for lvl, m in ROSPY.logged if lvl == "warn_throttle"]
        self.assertTrue(any("里程计" in w for w in warnings), "未给出缺里程计的告警")


class TestConfigLoader(NodeTestCase):
    """配置加载函数的独立测试。"""

    def test_load_config_uses_defaults(self):
        cfg = speed_controller.load_config_from_params()
        self.assertAlmostEqual(cfg.limits.max_lateral_accel, 3.0)
        self.assertAlmostEqual(cfg.kp, 1.0)

    def test_load_config_reads_params(self):
        ROSPY.params["~kp"] = 2.5
        ROSPY.params["~max_lateral_accel"] = 4.0
        cfg = speed_controller.load_config_from_params()
        self.assertAlmostEqual(cfg.kp, 2.5)
        self.assertAlmostEqual(cfg.limits.max_lateral_accel, 4.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
