"""Spin-loop integration tests: real KeyboardController, fake terminal + fake clock.

不需要 ROS / 终端 / 仿真器：`rospy` 与 `geometry_msgs` 打桩，用假时钟驱动 30 Hz 主循环，
假终端按帧喂按键，然后检查**真正发到 `/uav/cmd_vel` 的指令序列**——覆盖
"按住加速 -> 松开减速 -> 退出补零"整条链路，而不只是 `step_once` 单点。

覆盖：
  * 按下 W：首帧 0 起步、按 accel 逐帧爬升、约 0.5 s 到 2 m/s
  * 松开：hold_timeout 后目标归零，速度单调回落到 0，不会突跳
  * W+A：两轴同时出力（旧版一帧只能一个轴）
  * 方向键序列不产生运动指令、也不退出
  * 退出时补发零速，且 30 Hz 下每帧恰好一条指令
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/keyboard_control.py'
HZ = 30.0
DT = 1.0 / HZ

PARAMS = {
    "control/horiz_speed": 2.0, "control/vert_speed": 1.0, "control/yaw_rate": 1.0,
    "control/key_hold_timeout": 0.15,
    "control/accel_horiz": 4.0, "control/accel_vert": 2.0, "control/accel_yaw": 4.0,
    "topic/cmd_vel": "/uav/cmd_vel", "rate/publish_hz": HZ,
}


class _Vec3(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class _Twist(object):
    def __init__(self):
        self.linear = _Vec3()
        self.angular = _Vec3()


class FakeClock(object):
    """单调时钟；每次 rate.sleep() 推进一帧."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def tick(self):
        self.now += DT


class FakeReader(object):
    """按帧返回预设按键；脚本用完后一直返回 None（没按键）."""

    def __init__(self, keys):
        self.keys = list(keys)
        self.calls = 0

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def get_key(self, timeout=0.05):
        self.calls += 1
        return self.keys.pop(0) if self.keys else None


class SpinLoopTests(unittest.TestCase):
    def setUp(self):
        ros = types.ModuleType('rospy')
        ros.is_shutdown = Mock(return_value=False)
        ros.get_param = lambda name, default=None: PARAMS.get(name, default)
        ros.Publisher = lambda name, msg_type, queue_size=1: Mock()
        ros.Rate = lambda hz: Mock()
        ros.Subscriber = lambda *a, **k: None
        ros.logwarn = Mock()
        ros.init_node = Mock()

        geometry = types.ModuleType('geometry_msgs')
        messages = types.ModuleType('geometry_msgs.msg')
        messages.Twist = _Twist
        nav = types.ModuleType('nav_msgs')
        nav_msg = types.ModuleType('nav_msgs.msg')
        nav_msg.Odometry = type('Odometry', (object,), {})

        with patch.dict(sys.modules, {'rospy': ros, 'geometry_msgs': geometry,
                                      'geometry_msgs.msg': messages,
                                      'nav_msgs': nav, 'nav_msgs.msg': nav_msg}):
            spec = importlib.util.spec_from_file_location('keyboard_spin_under_test', SCRIPT)
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)

        self.published = []
        self.clock = FakeClock()
        self.node = self.module.KeyboardController()
        self.node.pub = Mock()
        self.node.pub.publish.side_effect = self.published.append
        self.node.rate = Mock()
        self.node.rate.sleep.side_effect = self.clock.tick

    # ------------------------------------------------------------------ 工具
    def spin(self, keys):
        reader = FakeReader(keys)
        with patch.object(self.module, 'RawKeyReader', return_value=reader), \
                patch.object(self.module.time, 'monotonic', self.clock), \
                patch('builtins.print'):
            self.node.spin()
        return reader

    def vx(self, i):
        return self.published[i].linear.x

    def vy(self, i):
        return self.published[i].linear.y

    # ------------------------------------------------------------------ 用例
    def test_hold_release_profile(self):
        frames = 20
        self.spin(['w'] * frames + [None] * 25 + ['\x03'])

        self.assertEqual(len(self.published), frames + 25 + 1,
                         '30 Hz 下每帧恰好一条指令，退出再补一条零速')
        self.assertAlmostEqual(self.vx(0), 0.0, places=9, msg='首帧 dt=0，从 0 起步')
        self.assertAlmostEqual(self.vx(1), 4.0 * DT, places=3, msg='第二帧按 accel 起步')
        self.assertAlmostEqual(self.vx(2), 2 * 4.0 * DT, places=3, msg='逐帧线性爬升')
        self.assertAlmostEqual(max(self.vx(i) for i in range(frames)), 2.0, places=6,
                               msg='按住期间爬到 horiz_speed=2.0 后不再超调')

        zero_vx = self.vx(frames + 24)
        self.assertAlmostEqual(zero_vx, 0.0, places=9, msg='松开足够久后完全停下')
        self.assertAlmostEqual(self.published[-1].linear.x, 0.0, places=9,
                               msg='退出时补发的零速')

    def test_release_is_monotonic(self):
        self.spin(['w'] * 20 + [None] * 30 + ['\x03'])
        seq = [self.vx(i) for i in range(20, 20 + 30)]
        drops = [a - b for a, b in zip(seq, seq[1:])]
        self.assertTrue(all(d >= -1e-9 for d in drops), '松开后速度只减不增（无突跳/反冲）')
        self.assertTrue(seq[0] > 0.0 and seq[-1] == 0.0, '从非零平滑降到 0')

    def test_combo_two_axes(self):
        self.spin(['w', 'a'] + [None] * 20 + ['\x03'])
        both = [(self.vx(i), self.vy(i)) for i in range(len(self.published) - 1)]
        self.assertTrue(any(vx > 0.0 and vy > 0.0 for vx, vy in both),
                        'W+A 会同时给出 vx、vy（旧版一帧只有一个轴）')
        self.assertLessEqual(max(vy for _, vy in both), 2.0)

    def test_arrow_sequence_does_not_move(self):
        reader = self.spin(['\x1b', '[', 'A', '\x03'])
        self.assertEqual(reader.calls, 4, '方向键整段序列被吃掉')
        self.assertEqual(len(self.published), 1, '只有退出时的一条零速')
        self.assertAlmostEqual(self.published[0].linear.x, 0.0, places=9)

    def test_yaw_component(self):
        self.spin(['q'] * 10 + ['\x03'])
        wz = [self.published[i].angular.z for i in range(10)]
        self.assertTrue(wz[0] == 0.0 and wz[1] > 0.0 and max(wz) <= 1.0,
                        'Q 走 yaw_rate 斜坡，稳态不超过 1.0 rad/s')


if __name__ == '__main__':
    unittest.main()
