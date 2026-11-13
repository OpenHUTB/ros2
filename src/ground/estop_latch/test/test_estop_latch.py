"""estop_latch 的单元测试。"""

import pytest
import rclpy
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from estop_latch.estop_latch_node import (
    EstopLatch,
    evaluate_release,
    input_stale,
    main,
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
def latch():
    rclpy.init()
    node = EstopLatch(parameter_overrides=[
        Parameter(name='start_latched', value=True),
        Parameter(name='release_hold_time', value=0.0),
        Parameter(name='input_timeout', value=0.0),
    ])
    node.estop_pub = _RecordingPublisher()
    node.status_pub = _RecordingPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _reset(node):
    return node.handle_reset(Trigger.Request(), Trigger.Response())


def test_evaluate_release_without_input():
    """从未收到输入时，人工复位应当被允许（上游可能是事件驱动）。"""
    assert evaluate_release(10.0, None, 0.0, False, 1.0, 2.0) == (True, '')


def test_evaluate_release_rejects_active_input():
    ok, reason = evaluate_release(10.0, 10.0, 10.0, True, 1.0, 0.0)
    assert ok is False
    assert '仍处于触发状态' in reason


def test_evaluate_release_rejects_stale_link():
    ok, reason = evaluate_release(10.0, 5.0, 0.0, False, 1.0, 2.0)
    assert ok is False
    assert '输入链路' in reason


def test_evaluate_release_rejects_short_hold():
    ok, reason = evaluate_release(10.0, 9.9, 9.0, False, 1.0, 2.0)
    assert ok is False
    assert '消失不足' in reason


def test_evaluate_release_accepts_after_hold():
    assert evaluate_release(10.0, 9.9, 7.0, False, 1.0, 2.0) == (True, '')


def test_evaluate_release_zero_hold_accepts_immediately():
    assert evaluate_release(10.0, 10.0, 10.0, False, 1.0, 0.0) == (True, '')


def test_evaluate_release_zero_timeout_skips_link_check():
    assert evaluate_release(100.0, 1.0, 0.0, False, 0.0, 2.0) == (True, '')


def test_input_stale_when_never_received():
    """没收到过输入不等于链路失效，否则会一启动就自锁。"""
    assert input_stale(10.0, None, 1.0) is False


def test_input_stale_within_timeout():
    assert input_stale(10.5, 10.0, 1.0) is False


def test_input_stale_after_timeout():
    assert input_stale(12.0, 10.0, 1.0) is True


def test_input_stale_disabled():
    assert input_stale(100.0, None, 0.0) is False


def test_start_latched_by_default(latch):
    latch.on_timer()

    assert latch.estop_pub.last().data is True
    assert latch.status_pub.last().data == 'latched: start'


def test_reset_allowed_without_any_input(latch):
    """默认参数下不应自锁死：没有输入也能人工复位。"""
    response = _reset(latch)
    latch.on_timer()

    assert response.success is True
    assert latch.estop_pub.last().data is False
    assert latch.status_pub.last().data == 'clear'


def test_latch_publishes_immediately():
    """收到 true 后立刻发布急停，不等 1 Hz 定时器。"""
    rclpy.init()
    try:
        node = EstopLatch(parameter_overrides=[
            Parameter(name='start_latched', value=False),
            Parameter(name='input_timeout', value=0.0),
        ])
        node.estop_pub = _RecordingPublisher()
        node.status_pub = _RecordingPublisher()

        node.estop_callback(Bool(data=True))

        assert node.estop_pub.last().data is True
        assert node.status_pub.last().data == 'latched: input'
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_reset_clears_latch(latch):
    latch.estop_callback(Bool(data=False))

    response = _reset(latch)
    latch.on_timer()

    assert response.success is True
    assert latch.latched is False
    assert latch.estop_pub.last().data is False
    assert latch.status_pub.last().data == 'clear'


def test_reset_when_already_clear_returns_notice(latch):
    latch.estop_callback(Bool(data=False))
    _reset(latch)

    response = _reset(latch)

    assert response.success is False
    assert '未闩锁' in response.message


def test_stale_link_blocks_reset():
    rclpy.init()
    try:
        node = EstopLatch(parameter_overrides=[
            Parameter(name='start_latched', value=False),
            Parameter(name='input_timeout', value=1.0),
        ])
        node.estop_pub = _RecordingPublisher()
        node.status_pub = _RecordingPublisher()
        node.estop_callback(Bool(data=False))
        node.last_input_time = node._now_seconds() - 5.0
        node.on_timer()
        assert node.status_pub.last().data == 'latched: input_timeout'

        response = _reset(node)

        assert response.success is False
        assert '输入链路' in response.message
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_reset_succeeds_after_new_message():
    """链路恢复（收到新消息）后可以正常复位。"""
    rclpy.init()
    try:
        node = EstopLatch(parameter_overrides=[
            Parameter(name='start_latched', value=False),
            Parameter(name='input_timeout', value=1.0),
            Parameter(name='release_hold_time', value=0.0),
        ])
        node.estop_pub = _RecordingPublisher()
        node.status_pub = _RecordingPublisher()
        node.estop_callback(Bool(data=False))
        node.last_input_time = node._now_seconds() - 5.0
        node.on_timer()
        assert _reset(node).success is False

        node.estop_callback(Bool(data=False))
        response = _reset(node)

        assert response.success is True
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_estop_publisher_keeps_last_value():
    rclpy.init()
    try:
        node = EstopLatch()
        assert (node.estop_pub.qos_profile.durability
                == DurabilityPolicy.TRANSIENT_LOCAL)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_main_exits_cleanly_on_bad_bool_param():
    """参数类型错误应以退出码 1 结束，而不是带 traceback 崩溃。"""
    with pytest.raises(SystemExit) as info:
        main(['--ros-args', '-p', 'auto_release:=abc'])

    assert info.value.code == 1


def test_estop_input_relatches_after_reset(latch):
    latch.estop_callback(Bool(data=False))
    _reset(latch)

    latch.estop_callback(Bool(data=True))
    latch.on_timer()

    assert latch.latched is True
    assert latch.status_pub.last().data == 'latched: input'
    assert latch.estop_pub.last().data is True


def test_reset_rejected_while_input_active(latch):
    latch.estop_callback(Bool(data=True))

    response = _reset(latch)

    assert response.success is False
    assert '仍处于触发状态' in response.message


def test_input_timeout_latches_after_input_stops():
    """收到过输入之后再中断，才按失效安全闩锁。"""
    rclpy.init()
    try:
        node = EstopLatch(parameter_overrides=[
            Parameter(name='start_latched', value=False),
            Parameter(name='input_timeout', value=1.0),
        ])
        node.estop_pub = _RecordingPublisher()
        node.status_pub = _RecordingPublisher()
        node.estop_callback(Bool(data=False))
        node.last_input_time = node._now_seconds() - 5.0

        node.on_timer()

        assert node.latched is True
        assert node.status_pub.last().data == 'latched: input_timeout'
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_no_input_does_not_self_latch():
    """事件驱动的上游一直不发消息时，不应因超时自己锁死。"""
    rclpy.init()
    try:
        node = EstopLatch(parameter_overrides=[
            Parameter(name='start_latched', value=False),
            Parameter(name='input_timeout', value=1.0),
        ])
        node.estop_pub = _RecordingPublisher()
        node.status_pub = _RecordingPublisher()

        node.on_timer()

        assert node.latched is False
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_auto_release_clears_latch():
    rclpy.init()
    try:
        node = EstopLatch(parameter_overrides=[
            Parameter(name='auto_release', value=True),
            Parameter(name='release_hold_time', value=0.0),
            Parameter(name='input_timeout', value=0.0),
        ])
        node.estop_pub = _RecordingPublisher()
        node.status_pub = _RecordingPublisher()
        node.last_input_time = node._now_seconds()
        node.last_active_time = node._now_seconds() - 10.0

        node.on_timer()

        assert node.latched is False
        assert node.estop_pub.last().data is False
        node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize(
    'name, value',
    [('input_timeout', '-1.0'),
     ('release_hold_time', '-1.0'),
     ('status_rate', '0.0')])
def test_invalid_parameter_rejected(name, value):
    rclpy.init(args=['--ros-args', '-p', '%s:=%s' % (name, value)])
    try:
        with pytest.raises(ValueError):
            EstopLatch()
    finally:
        rclpy.shutdown()


def test_integer_hold_time_is_accepted():
    """支持 dynamic_typing 的发行版上，CLI 传整数不应触发类型异常。"""
    rclpy.init(args=['--ros-args', '-p', 'release_hold_time:=2'])
    try:
        try:
            node = EstopLatch()
        except InvalidParameterTypeException:
            pytest.skip('当前发行版的参数声明不支持整数覆盖 double')
        assert node.release_hold_time == pytest.approx(2.0)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_float_hold_time_is_accepted():
    """Foxy 与 Humble 都支持的写法。"""
    rclpy.init(args=['--ros-args', '-p', 'release_hold_time:=2.5'])
    try:
        node = EstopLatch()
        assert node.release_hold_time == pytest.approx(2.5)
        node.destroy_node()
    finally:
        rclpy.shutdown()
