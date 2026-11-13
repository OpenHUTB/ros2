"""idm_car_following 的 launch 参数生效性测试（与 rclpy 接口一致的桩）。

评审 #998：launch 传 ``rate``（ROS 参数）时节点此前完全不读取，控制频率
永远是默认值。修复后节点显式声明并读取参数；本文件用桩 Node 模拟 ROS
参数覆盖，验证评审点：``rate:=20.0`` 时定时器周期实际变为 0.05 s，
且闭环仍收敛到与前车速度/平衡间距一致。

无 ROS 2 环境即可运行：

    python3 -m unittest discover -s tests -t .
"""

import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

OVERRIDES = {}          # 模拟 launch / --ros-args 传入的 ROS 参数


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass


class _Time:
    nanoseconds = 0

    def to_msg(self):
        return 0


class _Clock:
    def now(self):
        return _Time()


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class StubNode:
    """最小 Node 桩：只实现 ros_node.py 用到的接口。"""

    def __init__(self, name):
        self.node_name = name
        self._params = {}
        self.timer_period = None

    def declare_parameter(self, name, value):
        self._params[name] = OVERRIDES.get(name, value)

    def get_parameter(self, name):
        return _Param(self._params[name])

    def create_publisher(self, *a, **k):
        return _Pub()

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, period, cb):
        self.timer_period = period
        return None

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return _Clock()


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Any:
        """自动补全属性链的消息桩。"""

        def __getattr__(self, name):
            v = _Any()
            object.__setattr__(self, name, v)
            return v

    mod("rclpy")
    mod("rclpy.node", Node=StubNode)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=_Any)
    mod("std_msgs")
    mod("std_msgs.msg", Float64=_Any)


_install_stubs()
import ros_node  # noqa: E402  （此时 HAS_ROS 为 True，走节点分支）


class TestIDMLaunchParams(unittest.TestCase):
    def tearDown(self):
        OVERRIDES.clear()

    def test_rate_lead_x0_take_effect(self):
        # 评审点 1：rate:=20.0（外加 lead_x0:=50.0）必须真的生效
        OVERRIDES.update({"rate": 20.0, "lead_x0": 50.0})
        node = ros_node.IDMNode()
        self.assertAlmostEqual(node.timer_period, 0.05)
        self.assertAlmostEqual(node.period, 0.05)
        self.assertAlmostEqual(node.lead_x, 50.0)
        # n_sub = period/dt = 0.05/0.01 = 5 个积分子步
        self.assertEqual(node.n_sub, 5)

    def test_default_params_unchanged(self):
        node = ros_node.IDMNode()
        self.assertAlmostEqual(node.timer_period, 1.0 / 50.0)
        self.assertAlmostEqual(node.lead_x, 30.0)

    def test_nondefault_rate_converges_to_equilibrium(self):
        # 评审点 2：rate:=20.0 回放 120 s —— 前车里程计话题以 15 m/s 匀速刷新
        # （真实系统里由 /idm/lead_odom 发布方推进），自车速度收敛到 15 m/s；
        # IDM 平衡间距解自 a=0：s* = s0 + v*T = 24.5，
        #   gap_eq = s*/sqrt(1-(v/v0)^4) = 24.5/0.827 = 29.6 m
        OVERRIDES.update({"rate": 20.0})
        node = ros_node.IDMNode()
        for _ in range(20 * 120):           # 20 Hz x 120 s
            node.lead_x += 15.0 * node.period
            node._tick()
        self.assertAlmostEqual(node.ego_v, 15.0, delta=0.5)
        gap = node.lead_x - node.ego_x - ros_node.VEH_LEN
        s_star = node.model.s0 + node.ego_v * node.model.T
        gap_eq = s_star / (1.0 - (node.ego_v / node.model.v0) ** 4) ** 0.5
        self.assertAlmostEqual(gap, gap_eq, delta=1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
