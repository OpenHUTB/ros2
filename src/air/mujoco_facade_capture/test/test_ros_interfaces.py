"""ROS 2 消息接口及节点发布订阅声明检查。"""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from facade.ros_node import FacadeMissionNode


def test_standard_message_schemas_are_available():
    assert set(Image.get_fields_and_field_types()) == {
        "header",
        "height",
        "width",
        "encoding",
        "is_bigendian",
        "step",
        "data",
    }
    assert set(Float32MultiArray.get_fields_and_field_types()) == {"layout", "data"}
    assert set(Twist.get_fields_and_field_types()) == {"linear", "angular"}


def test_mission_node_declares_sensor_command_and_override_topics():
    rclpy.init(args=None)
    node = FacadeMissionNode()
    try:
        assert node.image_pub.msg_type is Image
        assert node.range_pub.msg_type is Float32MultiArray
        assert node.command_pub.msg_type is Twist
        assert node.override_sub.msg_type is Twist
        assert node.image_pub.topic_name == "/facade/image"
        assert node.range_pub.topic_name == "/facade/range"
        assert node.command_pub.topic_name == "/facade/cmd_vel"
        assert node.override_sub.topic_name == "/facade/cmd_vel_override"
        initial_position = node.sim.position.copy()
        override = Twist()
        override.linear.x = 0.8
        node._on_override(override)
        node._tick()
        assert node.sim.velocity[0] > 0.0
        assert (node.sim.position != initial_position).any()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_twist_topic_override_changes_physical_motion():
    rclpy.init(args=None)
    mission = FacadeMissionNode()
    probe = rclpy.create_node("facade_override_probe")
    publisher = probe.create_publisher(Twist, "/facade/cmd_vel_override", 10)
    commands = []
    probe.create_subscription(Twist, "/facade/cmd_vel", commands.append, 10)
    override = Twist()
    override.linear.x = 0.7
    initial_position = mission.sim.position.copy()
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            publisher.publish(override)
            rclpy.spin_once(mission, timeout_sec=0.01)
            rclpy.spin_once(probe, timeout_sec=0.01)
            if (
                any(np.isclose(command.linear.x, 0.7) for command in commands)
                and mission.sim.position[0] > initial_position[0] + 0.01
            ):
                break
        assert any(np.isclose(command.linear.x, 0.7) for command in commands)
        assert mission.sim.position[0] > initial_position[0] + 0.01
        assert mission.sim.velocity[0] > 0.0
    finally:
        probe.destroy_node()
        mission.destroy_node()
        rclpy.shutdown()
