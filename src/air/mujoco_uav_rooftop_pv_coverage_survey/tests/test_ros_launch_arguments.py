"""ROS launch argument and topic configuration tests / ROS 启动参数与话题配置测试。"""

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest
import yaml

from ros2_uav_controller import main


@pytest.mark.parametrize("use_process_arguments", [False, True])
def test_live_launch_accepts_ros_arguments_and_uses_configured_topic(
    tmp_path: Path, monkeypatch, use_process_arguments: bool
) -> None:
    params_file = tmp_path / "controller.yaml"
    params_file.write_text(
        "rooftop_pv_controller:\n"
        "  ros__parameters:\n"
        "    command_topic: /survey/velocity\n"
        "    image_topic: /survey/rgb\n",
        encoding="utf-8",
    )
    ros_args = ["--ros-args", "--params-file", str(params_file)]
    observed = {}

    class Twist:
        pass

    class Node:
        def __init__(self, name):
            observed["node_name"] = name
            self.parameters = {}

        def declare_parameter(self, name, default):
            self.parameters[name] = observed["overrides"].get(name, default)

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, message_type, topic, queue_size):
            observed["topic"] = topic
            observed["message_type"] = message_type
            return SimpleNamespace(publish=lambda message: observed.setdefault("messages", []).append(message))

        def create_timer(self, period, callback):
            observed["period"] = period
            callback()

        def destroy_node(self):
            observed["destroyed"] = True

    def init(*, args):
        observed["ros_args"] = args
        config_path = Path(args[args.index("--params-file") + 1])
        observed["overrides"] = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
            "rooftop_pv_controller"
        ]["ros__parameters"]

    rclpy = ModuleType("rclpy")
    rclpy.init = init
    rclpy.spin = lambda node: observed.update(parameters=node.parameters)
    rclpy.shutdown = lambda: observed.update(shutdown=True)
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    geometry_msgs = ModuleType("geometry_msgs")
    messages = ModuleType("geometry_msgs.msg")
    messages.Twist = Twist
    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": messages,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    if use_process_arguments:
        monkeypatch.setattr(sys, "argv", ["rooftop_pv_controller", *ros_args])
        result = main()
    else:
        result = main(ros_args)

    assert result == 0
    assert observed["ros_args"] == ros_args
    assert observed["node_name"] == "rooftop_pv_controller"
    assert observed["parameters"] == {
        "command_topic": "/survey/velocity",
        "image_topic": "/survey/rgb",
        "pose_topic": "/uav/pose",
        "route_topic": "/uav/route",
    }
    assert observed["topic"] == "/survey/velocity"
    assert observed["message_type"] is Twist
    assert observed["period"] == 0.1
    assert len(observed["messages"]) == 1
    assert isinstance(observed["messages"][0], Twist)
    assert observed["destroyed"] and observed["shutdown"]


def test_replay_rejects_unknown_arguments(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as exc:
        main(["--replay", str(tmp_path / "replay.jsonl"), "--unknown"])
    assert exc.value.code == 2
