"""quad_eso_hover 的 launch 参数生效性测试（与 rclpy 接口一致的桩）。

评审 #998：launch 传 ``w0`` / ``ref``（ROS 参数）时节点此前完全不读取，
改参数不生效。修复后节点显式声明并读取参数；本文件用桩 Node 模拟 ROS
参数覆盖，验证评审点：``ref:=2.0 w0:=10.0`` 时目标高度、观测器增益、
定时器周期实际改变，且闭环仍收敛到新的目标高度。

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
        # rclpy 语义：已有外部覆盖（launch / --ros-args）时取覆盖值
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
        """自动补全属性链的消息桩：odom.pose.pose.position.z 之类都能赋值。"""

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


class TestQuadLaunchParams(unittest.TestCase):
    def tearDown(self):
        OVERRIDES.clear()

    def test_ref_w0_rate_take_effect(self):
        # 评审点 1：ref:=2.0 w0:=10.0（外加 rate:=20.0）必须真的生效
        OVERRIDES.update({"ref": 2.0, "w0": 10.0, "rate": 20.0})
        node = ros_node.QuadESONode()
        self.assertAlmostEqual(node.ref, 2.0)
        # ESO 增益由 w0 生成：beta1 = 3*w0
        self.assertAlmostEqual(node.beta1, 30.0)
        self.assertAlmostEqual(node.beta2, 300.0)
        # 定时器周期真的变成 1/20 s
        self.assertAlmostEqual(node.timer_period, 0.05)
        self.assertAlmostEqual(node.period, 0.05)

    def test_default_params_unchanged(self):
        # 不传覆盖时仍是构造默认值
        node = ros_node.QuadESONode()
        self.assertAlmostEqual(node.ref, 1.0)
        self.assertAlmostEqual(node.beta1, 45.0)
        self.assertAlmostEqual(node.timer_period, 1.0 / 200.0)

    def test_nondefault_params_still_converge(self):
        # 评审点 2：ref:=2.0 w0:=10.0 回放 12 s，闭环收敛到新目标高度 2 m；
        # 4 s 起扰动 4 m/s^2，总扰动估计收敛到 -(g - d) = -5.81 m/s^2
        OVERRIDES.update({"ref": 2.0, "w0": 10.0, "rate": 20.0})
        node = ros_node.QuadESONode()
        for _ in range(20 * 12):            # 20 Hz x 12 s
            node._tick()
        self.assertAlmostEqual(node.z, 2.0, delta=1e-2)
        self.assertAlmostEqual(node.z3, -(9.81 - 4.0), delta=0.3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
