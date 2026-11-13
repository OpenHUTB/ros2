"""cartpole_lqr 的 launch 参数生效性测试（与 rclpy 接口一致的桩）。

``use_lqr`` 此前已正确声明（评审 #998 未点名本包）；本次把 theta0 / rate /
dt 一并声明读取，与另两个包统一参数模式。这里验证：launch 传
``use_lqr:=false rate:=20.0`` 时定时器周期与控制行为实际改变。

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
    mod("sensor_msgs")
    mod("sensor_msgs.msg", JointState=_Any)
    mod("std_msgs")
    mod("std_msgs.msg", Float64=_Any)


_install_stubs()
import ros_node  # noqa: E402  （此时 HAS_ROS 为 True，走节点分支）


class TestCartPoleLaunchParams(unittest.TestCase):
    def tearDown(self):
        OVERRIDES.clear()

    def test_use_lqr_false_rate_take_effect(self):
        # launch 传 use_lqr:=false rate:=20.0：开关与定时器周期都生效
        OVERRIDES.update({"use_lqr": False, "rate": 20.0})
        node = ros_node.CartPoleNode()
        self.assertFalse(node.use_lqr)
        self.assertAlmostEqual(node.timer_period, 0.05)
        self.assertEqual(node.n_sub, 20)     # 0.05 / 0.0025

    def test_pole_falls_without_lqr(self):
        # use_lqr:=false 回放 10 s：自由摆持续倒下（对照组真的无控）
        OVERRIDES.update({"use_lqr": False, "rate": 20.0})
        node = ros_node.CartPoleNode()
        for _ in range(20 * 10):
            node._tick()
        self.assertGreater(abs(node.state[2]), 0.5)

    def test_pole_balanced_with_lqr_and_custom_theta0(self):
        # theta0:=0.2 回放 10 s：LQR 把更歪的摆角拉回平衡附近
        OVERRIDES.update({"theta0": 0.2})
        node = ros_node.CartPoleNode()
        self.assertAlmostEqual(node.state[2], 0.2, delta=1e-9)  # 初值确实变了
        for _ in range(100 * 10):
            node._tick()
        self.assertLess(abs(node.state[2]), 0.05)


if __name__ == "__main__":
    unittest.main(verbosity=2)
