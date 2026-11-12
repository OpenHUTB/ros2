"""ROS adapter regressions for every Twist component and stale measurements."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Temperature

from mujoco_motor_thermal_guard.node import MotorThermalGuardNode


@pytest.fixture
def node():
    rclpy.init()
    instance = MotorThermalGuardNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def command():
    message = Twist()
    message.linear.x, message.linear.y, message.linear.z = 1.0, 2.0, 3.0
    message.angular.x, message.angular.y, message.angular.z = 0.5, 1.0, 1.5
    return message


def components(message):
    return [getattr(getattr(message, vector), axis)
            for vector in ("linear", "angular") for axis in ("x", "y", "z")]


def test_hot_temperature_scales_every_twist_component(node):
    published = []
    node.command_pub = SimpleNamespace(publish=published.append)
    temperature = Temperature()
    temperature.temperature = 70.0
    node._temperature(temperature)
    message = command()
    node._command(message)
    assert components(published[-1]) == pytest.approx([value * 0.7 for value in components(message)])


def test_missing_or_stale_temperature_stops_every_twist_component(node):
    published = []
    node.command_pub = SimpleNamespace(publish=published.append)
    message = command()
    node._command(message)
    assert components(published[-1]) == [0.0] * 6

    timeout = float(node.get_parameter("temperature_timeout_s").value)
    node.last_temperature_received_s = node.get_clock().now().nanoseconds * 1e-9 - timeout - 1.0
    node._command(message)
    assert components(published[-1]) == [0.0] * 6
