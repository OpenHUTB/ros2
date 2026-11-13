#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""APF 节点的 launch 参数与话题发布测试（与 rclpy 接口一致的桩）。

评审 #1400 第 1 点：文档声称"节点用桩验证过话题与参数"，但仓库里没有
桩测试。本文件补上：用桩 Node 模拟 ROS 参数覆盖（launch / --ros-args
语义），验证 ①节点真的声明并读取参数；②cmd_vel / planning/path 两个
话题真的被创建并在 _tick 里发布；③非默认参数（trap:=true）切换场景
后参考轨迹随之改变。

无 ROS 2 环境即可运行：

    python3 -m unittest discover -s tests -t .
"""

import math
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "..", "scripts"))

OVERRIDES = {}          # 模拟 launch / --ros-args 传入的 ROS 参数


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class _Time:
    nanoseconds = 0

    def to_msg(self):
        return 0


class _Clock:
    def now(self):
        return _Time()


class _Pub:
    """发布器桩：记录话题与已发布消息。"""

    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Sub:
    def __init__(self, topic, cb):
        self.topic = topic
        self.cb = cb


class StubNode:
    """最小 Node 桩：只实现 plan_control_node.py 用到的接口。"""

    def __init__(self, name):
        self.node_name = name
        self._params = {}
        self.pubs = {}
        self.subs = {}
        self.timer_period = None

    def declare_parameter(self, name, value):
        # rclpy 语义：已有外部覆盖（launch / --ros-args）时取覆盖值
        self._params[name] = OVERRIDES.get(name, value)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        p = _Pub(topic)
        self.pubs[topic] = p
        return p

    def create_subscription(self, msg_type, topic, cb, qos):
        s = _Sub(topic, cb)
        self.subs[topic] = s
        return s

    def create_timer(self, period, cb):
        self.timer_period = period
        return None

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return _Clock()


class _Any:
    """自动补全属性链的消息桩；append 供 msg.poses.append(ps) 使用。"""

    def __getattr__(self, name):
        v = _Any()
        object.__setattr__(self, name, v)
        return v

    def append(self, item):
        if "_items" not in self.__dict__:
            object.__setattr__(self, "_items", [])
        self._items.append(item)

    def __len__(self):
        return len(getattr(self, "_items", []))


class _Vec:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Quat:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _Odom:
    """数值化的 Odometry 桩：_on_odom 的四元数运算需要真实数字。"""

    def __init__(self, x, y, yaw):
        self.pose = types.SimpleNamespace()
        self.pose.pose = types.SimpleNamespace()
        self.pose.pose.position = _Vec()
        self.pose.pose.position.x = float(x)
        self.pose.pose.position.y = float(y)
        self.pose.pose.orientation = _Quat()
        q = self.pose.pose.orientation
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    mod("rclpy")
    mod("rclpy.node", Node=StubNode)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=_Any, Path=_Any)
    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=_Any, Twist=_Any)


_install_stubs()
import plan_control_node  # noqa: E402  （此时 HAS_ROS 为 True，走节点分支）


def _path_length(path):
    return sum(math.hypot(path[i + 1][0] - path[i][0],
                          path[i + 1][1] - path[i][1])
               for i in range(len(path) - 1))


class TestAPFRosParams(unittest.TestCase):
    def tearDown(self):
        OVERRIDES.clear()

    def test_topics_created(self):
        # 评审点：cmd_vel / planning/path 两个话题真的被创建
        node = plan_control_node.APFControlNode()
        self.assertIn("cmd_vel", node.pubs)
        self.assertIn("planning/path", node.pubs)
        self.assertIn("odom", node.subs)
        # 控制定时器周期 = 1/rate（默认 20 Hz）
        self.assertAlmostEqual(node.timer_period, 0.05)

    def test_v_override_takes_effect(self):
        # launch 里 v:=1.0 必须真的生效（声明 + 读回）
        OVERRIDES.update({"v": 1.0})
        node = plan_control_node.APFControlNode()
        self.assertAlmostEqual(node.v, 1.0)

    def test_trap_override_switches_scenario(self):
        # 非默认参数验证：trap:=true 切到凹形陷阱场景（带逃逸 72 m）
        OVERRIDES.update({"trap": True, "escape": True})
        node = plan_control_node.APFControlNode()
        self.assertAlmostEqual(_path_length(node.path), 72.0, delta=0.5)
        self.assertEqual(node.goal, plan_control_node.apf.TRAP_GOAL)

    def test_tick_publishes_path_and_cmd_vel(self):
        # 评审点：_tick 真的往 planning/path 与 cmd_vel 发布消息
        node = plan_control_node.APFControlNode()
        node._on_odom(_Odom(plan_control_node.apf.START[0],
                            plan_control_node.apf.START[1], 0.0))
        node._tick()
        self.assertGreaterEqual(len(node.pubs["planning/path"].msgs), 1)
        path_msg = node.pubs["planning/path"].msgs[0]
        self.assertEqual(len(path_msg.poses), len(node.path))
        self.assertGreaterEqual(len(node.pubs["cmd_vel"].msgs), 1)
        cmd = node.pubs["cmd_vel"].msgs[0]
        self.assertGreater(cmd.linear.x, 0.0)   # 未到终点，应有前进速度

    def test_tick_without_odom_publishes_nothing_but_path(self):
        # 没有 odom 反馈时不该发 cmd_vel（安全：无反馈不盲动）
        node = plan_control_node.APFControlNode()
        node._tick()
        self.assertEqual(len(node.pubs["cmd_vel"].msgs), 0)

    def test_stuck_no_reference_keeps_still_with_odom(self):
        # 评审 #1400 复核阻塞 1：trap + escape=false 卡死时，即使有 odom
        # 反馈也不许发 cmd_vel——旧版把止于卡点的残缺路径交给跟踪器，
        # 120 s 回放车走了 126 m 还在绕路径末端打转。现在不可达即清空
        # 路径：无参考轨迹可跟踪，机器人保持静止。
        OVERRIDES.update({"trap": True, "escape": False})
        node = plan_control_node.APFControlNode()
        self.assertEqual(node.path, [])
        node._on_odom(_Odom(plan_control_node.apf.TRAP_START[0],
                            plan_control_node.apf.TRAP_START[1], 1.5708))
        node._tick()
        self.assertEqual(len(node.pubs["cmd_vel"].msgs), 0)
        self.assertEqual(len(node.pubs["planning/path"].msgs), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
