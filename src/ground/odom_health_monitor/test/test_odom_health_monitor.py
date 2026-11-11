"""odom_health_monitor 的单元测试。"""

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration

from odom_health_monitor.odom_health_monitor_node import (
    OdomHealthMonitor,
    _is_reverse,
    _planar_speed,
    _speed_ratio,
)


class _RecordingPublisher:
    """记录发布内容，用于替代真实 publisher。"""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    def last(self):
        return self.messages[-1]


@pytest.fixture
def monitor():
    rclpy.init()
    node = OdomHealthMonitor()
    node.health_pub = _RecordingPublisher()
    node.ratio_pub = _RecordingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _cmd(x=0.0, y=0.0):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    return twist


def _odom(x=0.0, y=0.0):
    msg = Odometry()
    msg.twist.twist.linear.x = x
    msg.twist.twist.linear.y = y
    return msg


def _feed(monitor, cmd, odom, cmd_age=0.0, odom_age=0.0):
    monitor.last_cmd = cmd
    monitor.last_cmd_time = (
        monitor.get_clock().now() - Duration(seconds=cmd_age))
    monitor.last_odom = odom
    monitor.last_odom_time = (
        monitor.get_clock().now() - Duration(seconds=odom_age))


def test_planar_speed_uses_both_axes():
    assert _planar_speed(_cmd(3.0, 4.0)) == pytest.approx(5.0)


def test_speed_ratio_normal():
    assert _speed_ratio(0.2, 0.2, 0.02) == pytest.approx(1.0)


def test_speed_ratio_returns_none_inside_deadband():
    assert _speed_ratio(0.1, 0.01, 0.02) is None


def test_is_reverse_detects_opposite_direction():
    assert _is_reverse(_cmd(1.0), _odom(-1.0).twist.twist, 0.02) is True


def test_is_reverse_false_for_same_direction():
    assert _is_reverse(_cmd(1.0), _odom(1.0).twist.twist, 0.02) is False


def test_is_reverse_false_inside_deadband():
    assert _is_reverse(_cmd(0.01), _odom(-0.5).twist.twist, 0.02) is False


def test_missing_odom_reports_no_odom(monitor):
    monitor.on_timer()

    assert monitor.health_pub.last().data == 'no_odom'
    assert monitor.ratio_pub.last().data == pytest.approx(-1.0)


def test_stale_odom_reports_no_odom(monitor):
    _feed(monitor, _cmd(0.2), _odom(0.2), odom_age=5.0)

    monitor.on_timer()

    assert monitor.health_pub.last().data == 'no_odom'


def test_missing_cmd_reports_no_cmd(monitor):
    monitor.last_odom = _odom(0.2)
    monitor.last_odom_time = monitor.get_clock().now()

    monitor.on_timer()

    assert monitor.health_pub.last().data == 'no_cmd'


def test_stale_cmd_reports_no_cmd(monitor):
    _feed(monitor, _cmd(0.2), _odom(0.2), cmd_age=5.0)

    monitor.on_timer()

    assert monitor.health_pub.last().data == 'no_cmd'


def test_idle_when_command_inside_deadband(monitor):
    _feed(monitor, _cmd(0.01), _odom(0.0))

    monitor.on_timer()

    assert monitor.health_pub.last().data == 'idle'
    assert monitor.ratio_pub.last().data == pytest.approx(-1.0)


def test_normal_motion_reports_ok(monitor):
    _feed(monitor, _cmd(0.2), _odom(0.2))

    monitor.on_timer()

    assert monitor.health_pub.last().data == 'ok'
    assert monitor.ratio_pub.last().data == pytest.approx(1.0)


def test_overspeed_reports_slip(monitor):
    _feed(monitor, _cmd(0.2), _odom(0.5))

    monitor.on_timer()

    assert monitor.health_pub.last().data == 'slip'
    assert monitor.ratio_pub.last().data == pytest.approx(2.5)


def test_zero_measured_speed_needs_duration_before_stuck(monitor):
    _feed(monitor, _cmd(0.2), _odom(0.0))

    monitor.on_timer()
    assert monitor.health_pub.last().data == 'ok'
    assert monitor.stuck_since is not None

    monitor.stuck_since = (
        monitor.get_clock().now() - Duration(seconds=1.0))
    monitor.on_timer()

    assert monitor.health_pub.last().data == 'stuck'
    assert monitor.ratio_pub.last().data == pytest.approx(0.0)


def test_opposite_motion_reports_reverse(monitor):
    _feed(monitor, _cmd(0.2), _odom(-0.2))

    monitor.on_timer()

    assert monitor.health_pub.last().data == 'reverse'


def test_ok_resets_stuck_timer(monitor):
    _feed(monitor, _cmd(0.2), _odom(0.0))
    monitor.on_timer()
    assert monitor.stuck_since is not None

    _feed(monitor, _cmd(0.2), _odom(0.2))
    monitor.on_timer()

    assert monitor.health_pub.last().data == 'ok'
    assert monitor.stuck_since is None


@pytest.mark.parametrize(
    'name, value',
    [('velocity_deadband', '-0.1'),
     ('stuck_ratio', '1.0'),
     ('slip_ratio', '1.0'),
     ('stuck_duration', '0.0'),
     ('cmd_timeout', '0.0'),
     ('odom_timeout', '0.0'),
     ('publish_rate', '0.0')])
def test_invalid_parameter_rejected(name, value):
    rclpy.init(args=['--ros-args', '-p', '%s:=%s' % (name, value)])
    try:
        with pytest.raises(ValueError):
            OdomHealthMonitor()
    finally:
        rclpy.shutdown()
