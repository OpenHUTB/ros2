"""Functional tests for the mean filter node."""

import time

import pytest
import rclpy
from std_msgs.msg import Float64

from mean_filter_pkg.mean_filter_node import MeanFilterNode


def spin_until(node, condition, timeout):
    """Spin until the condition is met or the timeout elapses."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        if condition():
            return True
    return False


def test_rejects_non_positive_window_size():
    """window_size below 1 should raise ValueError during construction."""
    rclpy.init(args=['--ros-args', '-p', 'window_size:=0'])
    try:
        with pytest.raises(ValueError, match='window_size'):
            MeanFilterNode()
    finally:
        rclpy.shutdown()


def test_step_response_matches_moving_average():
    """Output should equal the moving average over the sliding window."""
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
            assert spin_until(node, lambda: len(received) > before, 2.0), \
                f'没有收到输入 {value} 对应的输出'
            assert received[-1] == pytest.approx(want, rel=1e-4)
    finally:
        node.destroy_node()
        rclpy.shutdown()
