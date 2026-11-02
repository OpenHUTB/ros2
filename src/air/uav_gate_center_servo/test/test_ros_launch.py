import os

import pytest


def test_node_imports():
    pytest.importorskip("rclpy")
    import uav_gate_center_servo.ros.gate_servo_node  # noqa: F401


def test_launch_file_contract():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, "launch", "gate_servo.launch.py")
    text = open(path, encoding="utf-8").read()
    assert "gate_servo_node" in text
    assert "with_sim" in text
    assert "headless" in text  # 注释里明确说明不能传 true
