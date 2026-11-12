"""ROS launch arguments, subscriptions and configured command publication tests."""

import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest
import yaml

from ros2_uav_controller import main


@pytest.mark.parametrize("use_process_arguments", [False, True])
def test_live_launch_consumes_inputs_and_respects_config(tmp_path: Path, monkeypatch, use_process_arguments):
    params_file = tmp_path / "controller.yaml"
    params_file.write_text(
        "rooftop_pv_controller:\n  ros__parameters:\n"
        "    command_topic: /survey/velocity\n    image_topic: /survey/rgb\n",
        encoding="utf-8",
    )
    ros_args = ["--ros-args", "--params-file", str(params_file)]
    observed = {"subscriptions": {}, "messages": {}, "overrides": {}}

    class Twist:
        def __init__(self):
            self.linear = NS(x=0.0, y=0.0, z=0.0)
            self.angular = NS(x=0.0, y=0.0, z=0.0)

    class String:
        def __init__(self):
            self.data = ""

    class PoseStamped:
        pass

    class RoutePath:
        pass

    class Image:
        pass

    class Node:
        def __init__(self, name):
            observed["node_name"] = name
            self.parameters = {}

        def declare_parameter(self, name, default):
            self.parameters[name] = observed["overrides"].get(name, default)

        def get_parameter(self, name):
            return NS(value=self.parameters[name])

        def create_publisher(self, message_type, topic, queue_size):
            observed["messages"][topic] = []
            return NS(publish=lambda message: observed["messages"][topic].append(message))

        def create_subscription(self, message_type, topic, callback, qos):
            observed["subscriptions"][topic] = (message_type, callback)

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

    def spin(node):
        pose = NS(header=NS(frame_id="map"), pose=NS(position=NS(x=0.0, y=1.0),
                  orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
        route = NS(header=NS(frame_id="map"), poses=[
            NS(header=NS(frame_id="map"), pose=NS(position=NS(x=x, y=0.0))) for x in (0.0, 10.0)
        ])
        image = NS(width=2, height=2, step=6, encoding="rgb8", data=bytes([10, 40, 76] * 4))
        observed["subscriptions"]["/uav/pose"][1](pose)
        observed["subscriptions"]["/uav/route"][1](route)
        observed["subscriptions"]["/survey/rgb"][1](image)
        node.publish_command()

    rclpy = ModuleType("rclpy")
    rclpy.ok = lambda: True
    rclpy.init, rclpy.spin = init, spin
    rclpy.shutdown = lambda: observed.update(shutdown=True)
    modules = {"rclpy": rclpy}
    for name, values in {
        "rclpy.node": {"Node": Node}, "rclpy.qos": {"qos_profile_sensor_data": 5},
        "geometry_msgs.msg": {"Twist": Twist, "PoseStamped": PoseStamped},
        "nav_msgs.msg": {"Path": RoutePath}, "sensor_msgs.msg": {"Image": Image},
        "std_msgs.msg": {"String": String},
    }.items():
        module = ModuleType(name)
        for key, value in values.items():
            setattr(module, key, value)
        modules[name] = module
        if "." in name:
            modules.setdefault(name.split(".")[0], ModuleType(name.split(".")[0]))
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    if use_process_arguments:
        monkeypatch.setattr(sys, "argv", ["rooftop_pv_controller", *ros_args])
        result = main()
    else:
        result = main(ros_args)

    assert result == 0
    assert observed["ros_args"] == ros_args
    assert observed["node_name"] == "rooftop_pv_controller"
    assert set(observed["subscriptions"]) == {"/uav/pose", "/uav/route", "/survey/rgb"}
    assert observed["subscriptions"]["/uav/pose"][0] is PoseStamped
    assert observed["subscriptions"]["/uav/route"][0] is RoutePath
    assert observed["subscriptions"]["/survey/rgb"][0] is Image
    commands = observed["messages"]["/survey/velocity"]
    assert commands[0].linear.x == commands[0].angular.z == 0.0
    assert 0.0 < commands[-1].linear.x <= 1.0
    assert -1.2 <= commands[-1].angular.z < 0.0
    status = json.loads(observed["messages"]["/rooftop_pv/status"][-1].data)
    assert status["state"] == "tracking"
    assert status["perception"]["pv_pixel_ratio"] == 1.0
    assert status["perception"]["source"] == "color_baseline_no_checkpoint"
    assert observed["destroyed"] and observed["shutdown"]


def test_replay_rejects_unknown_arguments(tmp_path: Path):
    with pytest.raises(SystemExit) as exc:
        main(["--replay", str(tmp_path / "replay.jsonl"), "--unknown"])
    assert exc.value.code == 2
