#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_teleop_keyboard.py -- 用 mock 的 rospy 验证键盘节点核心逻辑

不依赖 ROS 环境即可运行：
    python3 test_teleop_keyboard.py

覆盖：
  * 高度 <-> orientation.z 归一化比例的双向换算（对应 learning_planning.cpp:198）
  * W/S/A/D/Q/E/I/K/R/G 各按键对游标状态的影响
  * 边界裁剪与高度裁剪
  * 两段式 Goal 语义（第 1 次 G 起点，第 2 次 G 终点）
"""

import math
import os
import sys
import types
import unittest

# ---------------------------------------------------------------------------
# 构造 mock 的 ROS 模块，注入 sys.modules 后再导入被测模块
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))


def _find_teleop_dir():
    """定位 teleop_keyboard.py 所在目录，兼容两种存放布局：

    1. 本模块的平铺布局：test_teleop_keyboard.py 与 teleop_keyboard.py 同级
       （ros2 仓库 src/air/allocnet_teleop/）；
    2. AllocNet 工作区布局：被测脚本位于 ../scripts/
       （AllocNet 仓库 src/planner/test/ 与 src/planner/scripts/）。
    """
    if os.path.exists(os.path.join(HERE, "teleop_keyboard.py")):
        return HERE
    sibling = os.path.join(os.path.dirname(HERE), "scripts")
    if os.path.exists(os.path.join(sibling, "teleop_keyboard.py")):
        return sibling
    return HERE


sys.path.insert(0, _find_teleop_dir())


class _Msg(object):
    """通用消息容器。

    真实 rospy 消息在访问未赋值字段时会自动创建嵌套消息
    （例如 marker.header.frame_id 无需显式初始化），这里用 __getattr__
    模拟该行为，使被测节点可以像在 ROS 下一样写代码。
    """

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        val = _Msg()
        setattr(self, name, val)
        return val


class _Marker(_Msg):
    ADD = 0
    MODIFY = 1
    DELETE = 2
    DELETEALL = 3
    ARROW = 0
    CUBE = 1
    SPHERE = 2
    LINE_STRIP = 4
    LINE_LIST = 5
    POINTS = 8
    TEXT_VIEW_FACING = 9

    def __init__(self, **kw):
        # points 必须是真正的 list（节点会 append），其余字段按需自动创建
        super(_Marker, self).__init__(points=[], markers=[], **kw)


class _Publisher(object):
    def __init__(self, topic, msg_type, queue_size=10, latch=False):
        self.topic = topic
        self.msg_type = msg_type
        self.latch = latch
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Timer(object):
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback


def _install_mocks():
    rospy = types.ModuleType("rospy")

    class _Time(object):
        _t = 1000.0

        @staticmethod
        def now():
            return _Time

        @staticmethod
        def to_sec():
            return _Time._t

    class _Duration(object):
        def __init__(self, sec):
            self.sec = sec

    rospy.Time = _Time
    rospy.Duration = _Duration
    rospy.Publisher = _Publisher
    rospy.Timer = _Timer
    rospy.Subscriber = lambda *a, **k: None
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.on_shutdown = lambda *a, **k: None
    _logs = []
    rospy.loginfo = lambda *a, **k: _logs.append(("info", a))
    rospy.logwarn = lambda *a, **k: _logs.append(("warn", a))
    rospy.logerr = lambda *a, **k: _logs.append(("err", a))
    rospy._logs = _logs

    # get_param 直接返回默认值，模拟“参数都已按 launch 设定”
    rospy.get_param = lambda name, default=None: default

    sys.modules["rospy"] = rospy

    geo = types.ModuleType("geometry_msgs.msg")
    std = types.ModuleType("std_msgs.msg")
    vis = types.ModuleType("visualization_msgs.msg")

    class PoseStamped(_Msg):
        def __init__(self):
            self.header = _Msg(stamp=None, frame_id="")
            self.pose = _Msg(position=_Msg(x=0, y=0, z=0),
                             orientation=_Msg(x=0, y=0, z=0, w=1))

    class ColorRGBA(_Msg):
        """真实 rospy 的 ColorRGBA 支持位置参数 ColorRGBA(r, g, b, a)。"""

        def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0):
            super(ColorRGBA, self).__init__(r=r, g=g, b=b, a=a)

    class String(_Msg):
        """真实 rospy 的 String 支持位置参数 String(data)。"""

        def __init__(self, data=""):
            super(String, self).__init__(data=data)

    class Float64(_Msg):
        def __init__(self, data=0.0):
            super(Float64, self).__init__(data=data)

    class MarkerArray(_Msg):
        def __init__(self):
            super(MarkerArray, self).__init__(markers=[])

    geo.PoseStamped = PoseStamped
    std.Float64 = Float64
    std.ColorRGBA = ColorRGBA
    std.String = String
    vis.Marker = _Marker
    vis.MarkerArray = MarkerArray

    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geo
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std
    sys.modules["visualization_msgs"] = types.ModuleType("visualization_msgs")
    sys.modules["visualization_msgs.msg"] = vis

    # termios / tty 是 Unix-only 模块。节点本身只在 Linux(ROS) 上运行，
    # 在模块顶层导入它们是正确的；这里仅为在 Windows 上跑单元测试而打桩。
    try:
        import termios    # noqa: F401
        import tty        # noqa: F401
    except ImportError:
        termios = types.ModuleType("termios")
        termios.tcgetattr = lambda fd: []
        termios.tcsetattr = lambda fd, when, attrs: None
        termios.TCSADRAIN = 1
        sys.modules["termios"] = termios

        tty = types.ModuleType("tty")
        tty.setcbreak = lambda fd: None
        sys.modules["tty"] = tty

    return rospy


_install_mocks()
import teleop_keyboard as tk       # noqa: E402


# ---------------------------------------------------------------------------
class TeleopTests(unittest.TestCase):

    def setUp(self):
        self.node = tk.TeleopKeyboard()
        # 静音终端 banner，否则每个按键都会刷屏，淹没测试结果
        self.node.render = lambda: None

    # ---- 高度换算 --------------------------------------------------------
    def test_altitude_ratio_bounds(self):
        """ratio 应为 [0,1]，下限对应 z_origin+dilate，上限对应 z_origin+z_size-dilate。"""
        n = self.node
        lo = n.map_z_origin + n.dilate                 # 0.2
        hi = n.map_z_origin + n.map_z_size - n.dilate  # 4.8

        self.assertAlmostEqual(n.altitude_to_orientation_z(lo), 0.0, places=6)
        self.assertAlmostEqual(n.altitude_to_orientation_z(hi), 1.0, places=6)

    def test_altitude_ratio_out_of_range_is_clamped(self):
        n = self.node
        self.assertEqual(n.altitude_to_orientation_z(-100.0), 0.0)
        self.assertEqual(n.altitude_to_orientation_z(999.0), 1.0)

    def test_altitude_roundtrip_matches_cpp_formula(self):
        """反解出的 ratio 代回 C++ 公式应还原原高度。"""
        n = self.node
        span = n.map_z_size - 2 * n.dilate
        for z in (0.2, 1.0, 1.5, 2.5, 4.0, 4.8):
            ratio = n.altitude_to_orientation_z(z)
            # 对应 learning_planning.cpp:198
            z_goal = (n.map_z_origin + n.dilate) + abs(ratio) * span
            self.assertAlmostEqual(z_goal, z, places=6,
                                   msg="z=%s 还原为 %s" % (z, z_goal))

    # ---- 按键行为 --------------------------------------------------------
    def test_forward_then_back_returns_to_origin(self):
        n = self.node
        n.on_key("w")
        n.on_key("s")
        self.assertAlmostEqual(n.x, 0.0, places=6)
        self.assertAlmostEqual(n.y, 0.0, places=6)

    def test_right_then_left_returns_to_origin(self):
        n = self.node
        n.on_key("d")
        n.on_key("a")
        self.assertAlmostEqual(n.x, 0.0, places=6)
        self.assertAlmostEqual(n.y, 0.0, places=6)

    def test_up_down_returns_to_origin(self):
        n = self.node
        n.on_key("i")
        n.on_key("k")
        self.assertAlmostEqual(n.z, 1.0, places=6)

    def test_forward_uses_yaw_direction(self):
        """偏航 90 度后按 W 应主要改变 y。"""
        n = self.node
        n.yaw = math.pi / 2
        n.on_key("w")
        self.assertAlmostEqual(n.x, 0.0, places=6)
        self.assertAlmostEqual(n.y, 0.5, places=6)

    def test_reset_clears_state(self):
        n = self.node
        n.on_key("w")
        n.on_key("i")
        n.send_goal()
        self.assertTrue(n.sent_waypoints)
        n.on_key("r")
        self.assertEqual(n.sent_waypoints, [])
        self.assertEqual(n.goal_seq, 0)
        self.assertAlmostEqual(n.x, 0.0)
        self.assertAlmostEqual(n.z, 1.0)

    # ---- 边界与高度裁剪 --------------------------------------------------
    def test_xy_clamped_to_map(self):
        n = self.node
        for _ in range(200):
            n.on_key("w")
        self.assertLessEqual(n.x, n.map_x_origin + n.map_x_size)

        n.on_key("r")
        for _ in range(200):
            n.on_key("s")
        self.assertGreaterEqual(n.x, n.map_x_origin)

    def test_z_clamped_to_map(self):
        n = self.node
        for _ in range(100):
            n.on_key("i")
        self.assertLessEqual(n.z, n.map_z_origin + n.map_z_size - n.dilate)

        n.on_key("r")
        for _ in range(100):
            n.on_key("k")
        self.assertGreaterEqual(n.z, n.map_z_origin + n.dilate)

    # ---- 两段式 Goal -----------------------------------------------------
    def test_goal_is_two_stage(self):
        n = self.node
        self.assertEqual(n.goal_seq, 0)

        n.on_key("g")
        self.assertEqual(n.goal_seq, 1)
        self.assertEqual(len(n.sent_waypoints), 1)
        self.assertIn("起点", n.phase)

        n.on_key("w")
        n.on_key("g")
        self.assertEqual(n.goal_seq, 2)
        self.assertEqual(len(n.sent_waypoints), 2)
        self.assertIn("规划", n.phase)

    def test_goal_message_encodes_height_in_orientation_z(self):
        """position.z 只是参考值，真正生效的是 orientation.z。"""
        n = self.node
        n.z = 2.5
        n.on_key("g")
        msg = n.goal_pub.sent[-1]
        expected = n.altitude_to_orientation_z(2.5)
        self.assertAlmostEqual(msg.pose.orientation.z, expected, places=9)
        self.assertAlmostEqual(msg.pose.position.z, 2.5, places=9)
        self.assertEqual(msg.header.frame_id, n.frame_id)

    def test_goal_publishes_on_configured_topic(self):
        n = self.node
        self.assertEqual(n.goal_pub.topic, "/move_base_simple/goal")

    # ---- 步长调整 --------------------------------------------------------
    def test_step_size_adjustment_respects_bounds(self):
        n = self.node
        for _ in range(100):
            n.on_key("+")
        self.assertLessEqual(n.step, n.step_max)
        for _ in range(200):
            n.on_key("-")
        self.assertGreaterEqual(n.step, n.step_min)

    # ---- 未知按键不改变状态 ----------------------------------------------
    def test_unknown_key_is_noop(self):
        n = self.node
        before = (n.x, n.y, n.z, n.yaw)
        n.on_key("z")
        n.on_key("%")
        self.assertEqual((n.x, n.y, n.z, n.yaw), before)

    def test_ctrl_c_requests_exit(self):
        n = self.node
        self.assertFalse(n.on_key("\x03"))

    # ---- 随机游走不越界 --------------------------------------------------
    def test_random_walk_stays_in_bounds(self):
        import random
        rng = random.Random(7)
        n = self.node
        keys = list("wsadikqe") + ["i"]
        for _ in range(3000):
            n.on_key(rng.choice(keys))
            self.assertGreaterEqual(n.x, n.map_x_origin - 1e-9)
            self.assertLessEqual(n.x, n.map_x_origin + n.map_x_size + 1e-9)
            self.assertGreaterEqual(n.y, n.map_y_origin - 1e-9)
            self.assertLessEqual(n.y, n.map_y_origin + n.map_y_size + 1e-9)
            self.assertGreaterEqual(n.z, n.map_z_origin + n.dilate - 1e-9)
            self.assertLessEqual(n.z,
                                 n.map_z_origin + n.map_z_size - n.dilate + 1e-9)

    # ---- 航点发布 --------------------------------------------------------
    def test_waypoint_marker_array_grows(self):
        n = self.node
        n.on_key("g")
        n.on_key("w")
        n.on_key("g")
        arr = n.path_pub.sent[-1]
        # 2 个航点球 + 1 条折线
        self.assertEqual(len(arr.markers), 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
