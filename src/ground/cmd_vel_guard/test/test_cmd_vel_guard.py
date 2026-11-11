"""cmd_vel_guard 的单元测试。"""

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from std_msgs.msg import Bool

from cmd_vel_guard.cmd_vel_guard_node import CmdVelGuard, _clamp


class _RecordingPublisher:
    """记录发布内容，用于替代真实 publisher。"""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    def last(self):
        return self.messages[-1]


@pytest.fixture
def guard():
    rclpy.init()
    node = CmdVelGuard()
    node.cmd_pub = _RecordingPublisher()
    node.status_pub = _RecordingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _make_cmd(linear_x=0.0, angular_z=0.0):
    cmd = Twist()
    cmd.linear.x = linear_x
    cmd.angular.z = angular_z
    return cmd


def test_clamp_normal():
    assert _clamp(0.2, 0.3) == 0.2
    assert _clamp(-0.2, 0.3) == -0.2


def test_clamp_boundary():
    assert _clamp(0.3, 0.3) == 0.3
    assert _clamp(-0.3, 0.3) == -0.3
    assert _clamp(0.8, 0.3) == 0.3
    assert _clamp(-0.8, 0.3) == -0.3


@pytest.mark.parametrize(
    'value', [float('nan'), float('inf'), float('-inf')])
def test_clamp_not_finite_returns_zero(value):
    assert _clamp(value, 0.3) == 0.0


def test_estop_has_priority_over_timeout(guard):
    guard.last_cmd = _make_cmd(linear_x=0.2)
    guard.last_cmd_time = (
        guard.get_clock().now() - Duration(seconds=10.0))
    guard.estop = True

    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == 0.0
    assert guard.cmd_pub.last().angular.z == 0.0
    assert guard.status_pub.last().data == 'estop'


def test_timeout_outputs_zero(guard):
    guard.last_cmd = _make_cmd(linear_x=0.2)
    guard.last_cmd_time = (
        guard.get_clock().now() - Duration(seconds=10.0))

    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == 0.0
    assert guard.status_pub.last().data == 'timeout'


def test_estop_clears_cache_and_waits_for_new_command(guard):
    """急停清除旧指令，解除后不发送新指令则保持零速。"""
    guard.last_cmd = _make_cmd(linear_x=0.2)
    guard.last_cmd_time = guard.get_clock().now()

    guard.estop_callback(Bool(data=True))

    assert guard.last_cmd is None
    assert guard.last_cmd_time is None

    guard.on_timer()
    assert guard.cmd_pub.last().linear.x == 0.0
    assert guard.status_pub.last().data == 'estop'

    guard.estop_callback(Bool(data=False))
    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == 0.0
    assert guard.status_pub.last().data == 'waiting'

    guard.cmd_callback(_make_cmd(linear_x=0.2))
    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == pytest.approx(0.2)
    assert guard.status_pub.last().data == 'ok'


def test_command_received_during_estop_is_discarded(guard):
    """急停期间收到的指令不缓存，解除后仍保持零速。"""
    guard.estop_callback(Bool(data=True))
    guard.cmd_callback(_make_cmd(linear_x=0.2))

    assert guard.last_cmd is None

    guard.estop_callback(Bool(data=False))
    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == 0.0
    assert guard.status_pub.last().data == 'waiting'


def test_nan_command_outputs_zero(guard):
    guard.last_cmd = _make_cmd(linear_x=float('nan'))
    guard.last_cmd_time = guard.get_clock().now()

    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == 0.0
    assert guard.status_pub.last().data == 'invalid'


def test_normal_command_passes_through(guard):
    guard.last_cmd = _make_cmd(linear_x=0.2, angular_z=0.5)
    guard.last_cmd_time = guard.get_clock().now()

    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == 0.2
    assert guard.cmd_pub.last().angular.z == 0.5
    assert guard.status_pub.last().data == 'ok'


def test_overspeed_command_is_clamped(guard):
    guard.last_cmd = _make_cmd(linear_x=0.8, angular_z=2.0)
    guard.last_cmd_time = guard.get_clock().now()

    guard.on_timer()

    assert guard.cmd_pub.last().linear.x == pytest.approx(0.3)
    assert guard.cmd_pub.last().angular.z == pytest.approx(0.8)
    assert guard.status_pub.last().data == 'clamped'


@pytest.mark.parametrize(
    'name, value',
    [('max_linear_x', '-1.0'),
     ('max_angular_z', '-0.5'),
     ('cmd_timeout', '0.0'),
     ('publish_rate', '0.0')])
def test_invalid_parameter_rejected(name, value):
    rclpy.init(args=['--ros-args', '-p', '%s:=%s' % (name, value)])
    try:
        with pytest.raises(ValueError):
            CmdVelGuard()
    finally:
        rclpy.shutdown()
