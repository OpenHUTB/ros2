"""ROS context and KeyboardInterrupt cleanup regression tests."""

import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from ros2_uav_controller import main


@pytest.mark.parametrize("exit_mode", ["normal", "keyboard_interrupt", "already_shutdown", "shutdown_then_interrupt"])
def test_controller_exit_closes_node_and_context_once(monkeypatch, exit_mode):
    """A signal may close the ROS context before spin raises KeyboardInterrupt."""
    observed = {"active": False, "shutdown_calls": 0, "events": []}

    class Node:
        def __init__(self, name):
            self.parameters = {}
            observed["node"] = self

        def declare_parameter(self, name, default):
            self.parameters[name] = default

        def get_parameter(self, name):
            return NS(value=self.parameters[name])

        def create_publisher(self, *args, **kwargs):
            return NS(publish=lambda message: None)

        def create_subscription(self, *args, **kwargs):
            return object()

        def create_timer(self, *args, **kwargs):
            return object()

        def destroy_node(self):
            observed["events"].append("destroy_node")

    def init(*, args):
        assert args == []
        observed["active"] = True
        observed["events"].append("init")

    def shutdown():
        assert observed["active"], "ROS context must not be shut down twice"
        observed["shutdown_calls"] += 1
        observed["active"] = False
        observed["events"].append("shutdown")

    def spin(node):
        assert node is observed["node"]
        observed["events"].append("spin")
        if exit_mode in ("already_shutdown", "shutdown_then_interrupt"):
            shutdown()
        if exit_mode in ("keyboard_interrupt", "shutdown_then_interrupt"):
            raise KeyboardInterrupt()

    rclpy = ModuleType("rclpy")
    rclpy.init, rclpy.spin, rclpy.shutdown = init, spin, shutdown
    rclpy.ok = lambda: observed["active"]
    modules = {"rclpy": rclpy}
    for name, values in {
        "rclpy.node": {"Node": Node}, "rclpy.qos": {"qos_profile_sensor_data": object()},
        "geometry_msgs.msg": {"Twist": type("Twist", (), {}), "PoseStamped": type("PoseStamped", (), {})},
        "nav_msgs.msg": {"Path": type("RoutePath", (), {})},
        "sensor_msgs.msg": {"Image": type("Image", (), {})},
        "std_msgs.msg": {"String": type("String", (), {})},
    }.items():
        module = ModuleType(name)
        for key, value in values.items():
            setattr(module, key, value)
        modules[name] = module
        modules.setdefault(name.split(".")[0], ModuleType(name.split(".")[0]))
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    try:
        result = main([])
    except KeyboardInterrupt:
        pytest.fail("main must handle KeyboardInterrupt and finish ROS cleanup")
    assert result == 0
    assert observed["shutdown_calls"] == 1
    assert observed["events"].count("destroy_node") == 1
    assert not observed["active"]
    if exit_mode in ("already_shutdown", "shutdown_then_interrupt"):
        assert observed["events"] == ["init", "spin", "shutdown", "destroy_node"]
    else:
        assert observed["events"] == ["init", "spin", "destroy_node", "shutdown"]
