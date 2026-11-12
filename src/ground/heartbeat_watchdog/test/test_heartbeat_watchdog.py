"""heartbeat_watchdog 的单元测试。"""

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from heartbeat_watchdog.heartbeat_watchdog_node import (
    HeartbeatWatchdog,
    _split_nodes,
    _stale_nodes,
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
def watchdog():
    rclpy.init()
    node = HeartbeatWatchdog()
    node.status_pub = _RecordingPublisher()
    node.estop_pub = _RecordingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_stale_nodes_empty_when_all_fresh():
    last_seen = {'a': 10.0, 'b': 9.5}
    assert _stale_nodes(last_seen, 10.2, 1.0, ['a', 'b']) == []


def test_stale_nodes_detects_timeout():
    last_seen = {'a': 10.0, 'b': 8.0}
    assert _stale_nodes(last_seen, 10.2, 1.0, ['a', 'b']) == ['b']


def test_stale_nodes_boundary_is_not_stale():
    last_seen = {'a': 10.0}
    assert _stale_nodes(last_seen, 11.0, 1.0, ['a']) == []


def test_stale_nodes_never_seen_is_stale():
    assert _stale_nodes({}, 10.0, 1.0, ['a']) == ['a']


def test_stale_nodes_ignores_unwatched_sources():
    last_seen = {'a': 10.0, 'other': 1.0}
    assert _stale_nodes(last_seen, 10.1, 1.0, ['a']) == []


def test_waiting_before_any_heartbeat(watchdog):
    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'waiting'
    assert watchdog.estop_pub.last().data is True


def test_unrelated_heartbeat_keeps_waiting(watchdog):
    """不在名单里的心跳不应把 waiting 变成 stale。"""
    watchdog.heartbeat_callback(String(data='other_node'))

    assert watchdog.last_seen == {}

    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'waiting'


def test_partial_report_keeps_waiting(watchdog):
    watchdog.watched_nodes = ['navigator', 'controller']
    watchdog.heartbeat_callback(String(data='navigator'))

    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'waiting'


def test_ok_after_heartbeat(watchdog):
    watchdog.heartbeat_callback(String(data='navigator'))

    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'ok'
    assert watchdog.estop_pub.last().data is False


def test_stale_after_timeout(watchdog):
    watchdog.heartbeat_callback(String(data='navigator'))
    watchdog.last_seen['navigator'] = watchdog._now_seconds() - 5.0

    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'stale: navigator'
    assert watchdog.estop_pub.last().data is True


def test_recovery_returns_to_ok(watchdog):
    watchdog.heartbeat_callback(String(data='navigator'))
    watchdog.last_seen['navigator'] = watchdog._now_seconds() - 5.0
    watchdog.on_timer()
    assert watchdog.status_pub.last().data == 'stale: navigator'

    watchdog.heartbeat_callback(String(data='navigator'))
    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'ok'
    assert watchdog.estop_pub.last().data is False


def test_multiple_nodes_reports_only_stale_one(watchdog):
    watchdog.watched_nodes = ['navigator', 'controller']
    now = watchdog._now_seconds()
    watchdog.last_seen = {'navigator': now, 'controller': now - 5.0}
    watchdog.heartbeat_timeout = 1.0

    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'stale: controller'


def test_blank_heartbeat_is_ignored(watchdog):
    watchdog.heartbeat_callback(String(data='   '))

    assert watchdog.last_seen == {}


def test_publish_estop_disabled_sends_no_estop(watchdog):
    watchdog.publish_estop = False

    watchdog.on_timer()

    assert watchdog.status_pub.last().data == 'waiting'
    assert watchdog.estop_pub.messages == []


@pytest.mark.parametrize(
    'name, value',
    [('heartbeat_timeout', '0.0'),
     ('heartbeat_timeout', '-1.0'),
     ('publish_rate', '0.0')])
def test_invalid_parameter_rejected(name, value):
    rclpy.init(args=['--ros-args', '-p', '%s:=%s' % (name, value)])
    try:
        with pytest.raises(ValueError):
            HeartbeatWatchdog()
    finally:
        rclpy.shutdown()


def test_split_nodes_drops_blank_entries():
    assert _split_nodes('navigator, ,controller') == [
        'navigator', 'controller']
    assert _split_nodes('') == []
    assert _split_nodes(' , ') == []


def test_empty_watched_nodes_rejected():
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            HeartbeatWatchdog(parameter_overrides=[
                Parameter(name='watched_nodes', value=' , ')])
    finally:
        rclpy.shutdown()


def test_integer_timeout_is_accepted():
    """CLI 传整数不应触发参数类型异常。"""
    rclpy.init(args=['--ros-args', '-p', 'heartbeat_timeout:=2'])
    try:
        node = HeartbeatWatchdog()
        assert node.heartbeat_timeout == pytest.approx(2.0)
        node.destroy_node()
    finally:
        rclpy.shutdown()
