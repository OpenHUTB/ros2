"""均值滤波器的单元测试与 ROS 节点集成测试."""

import time

from mean_filter_pkg.mean_filter_node import MeanFilterNode
from mean_filter_pkg.mean_filter_node import MovingAverageFilter
import pytest
import rclpy
from std_msgs.msg import Float64


@pytest.mark.parametrize(
    'window_size', [0, -1, 1.5, '5', None, True])
def test_rejects_invalid_window_size(window_size):
    """窗口长度不是正整数时应抛出 ValueError."""
    with pytest.raises(ValueError, match='window_size'):
        MovingAverageFilter(window_size)


def test_step_response_matches_moving_average():
    """阶跃输入下输出应与滑动平均的期望值一致."""
    filter_ = MovingAverageFilter(5)
    outputs = [filter_.update(value)
               for value in (10.0, 10.0, 20.0, 20.0, 20.0, 20.0, 20.0)]
    expected = [10.0, 10.0, 40.0 / 3.0, 15.0, 16.0, 18.0, 20.0]
    assert outputs == pytest.approx(expected, rel=1e-6)


def test_window_keeps_only_latest_samples():
    """窗口填满后应丢弃最早的样本."""
    filter_ = MovingAverageFilter(3)
    outputs = [filter_.update(value)
               for value in (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)]
    assert outputs == pytest.approx(
        [10.0, 15.0, 20.0, 30.0, 40.0, 50.0], rel=1e-6)


def test_reset_clears_window():
    """Reset 后应重新从空窗口开始累计."""
    filter_ = MovingAverageFilter(2)
    filter_.update(10.0)
    filter_.update(20.0)
    filter_.reset()
    assert filter_.update(5.0) == pytest.approx(5.0)


def spin_until(node, condition, timeout):
    """在超时前持续 spin，直到条件满足."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        if condition():
            return True
    return False


def test_ros_node_rejects_zero_window_size():
    """通过 ROS 参数传入 0 时节点构造应失败."""
    rclpy.init(args=['--ros-args', '-p', 'window_size:=0'])
    try:
        with pytest.raises(ValueError, match='window_size'):
            MeanFilterNode()
    finally:
        rclpy.shutdown()


def test_ros_step_response():
    """ROS 节点发布的话题数据应符合滑动平均结果."""
    rclpy.init(args=[
        '--ros-args',
        '-p', 'input_topic:=/test/mean_filter/raw',
        '-p', 'output_topic:=/test/mean_filter/filtered',
        '-p', 'window_size:=3',
    ])
    node = MeanFilterNode()
    received = []
    node.create_subscription(
        Float64, '/test/mean_filter/filtered',
        lambda msg: received.append(msg.data), 10)
    publisher = node.create_publisher(Float64, '/test/mean_filter/raw', 10)
    try:
        # 等待发布者与订阅者完成发现，避免第一条数据丢失
        spin_until(node, lambda: False, 0.5)

        expected = [10.0, 10.0, 40.0 / 3.0, 20.0, 30.0]
        for value, want in zip((10.0, 10.0, 20.0, 30.0, 40.0), expected):
            before = len(received)
            publisher.publish(Float64(data=value))
            assert spin_until(
                node, lambda: len(received) > before, 2.0), \
                f'没有收到输入 {value} 对应的输出'
            assert received[-1] == pytest.approx(want, rel=1e-4)
    finally:
        node.destroy_node()
        rclpy.shutdown()
