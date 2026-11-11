"""ROS boundary regressions with a controlled clock / 带受控时钟的 ROS 边界回归。"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

MODULE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_ROOT))


@pytest.fixture
def ros_runtime(monkeypatch, tmp_path):
    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    params = {"checkpoint": str(tmp_path / "weights.pt")}
    (tmp_path / "weights.pt").touch()

    class FakeNode:
        def __init__(self, name):
            self.messages = []
            self.errors = []
            self.destroyed = False

        def declare_parameter(self, name, default):
            return SimpleNamespace(value=params.get(name, default))

        def create_publisher(self, *args):
            return SimpleNamespace(publish=self.messages.append)

        def create_subscription(self, *args):
            return args

        def create_timer(self, *args):
            return args

        def get_logger(self):
            return SimpleNamespace(error=self.errors.append, warning=self.errors.append)

        def destroy_node(self):
            self.destroyed = True

    class Bridge:
        def imgmsg_to_cv2(self, msg, desired_encoding):
            return np.zeros((msg.height, msg.width, 3), dtype=np.uint8)

    active = [False]
    fake_ros = SimpleNamespace(
        init=lambda **kwargs: active.__setitem__(0, True),
        ok=lambda: active[0],
        shutdown=lambda: active.__setitem__(0, False),
        spin=lambda node: None,
    )
    for name, module in {
        "rclpy": fake_ros, "rclpy.node": SimpleNamespace(Node=FakeNode),
        "cv_bridge": SimpleNamespace(CvBridge=Bridge),
        "geometry_msgs": SimpleNamespace(), "geometry_msgs.msg": SimpleNamespace(Twist=Twist),
        "sensor_msgs": SimpleNamespace(), "sensor_msgs.msg": SimpleNamespace(Image=object),
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("_ros_steering_test", MODULE_ROOT / "ros_node.py")
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    clock = [10.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])

    class Core:
        def __init__(self, *args):
            pass

        def predict(self, image):
            return module.TwistCommand(1.0, 0.65, "LEFT", 1.0)

    return SimpleNamespace(module=module, core=Core, clock=clock, params=params,
                           ros=fake_ros, active=active, monkeypatch=monkeypatch)


def make_node(runtime):
    runtime.monkeypatch.setattr(runtime.module, "VisualSteeringCore", runtime.core)
    return runtime.module.VisualSteeringNode()


def valid_image():
    return SimpleNamespace(width=2, height=2, data=b"rgb" * 4)


def assert_stopped(node):
    assert node.messages[-1].linear.x == 0.0
    assert node.messages[-1].angular.z == 0.0


def test_missing_checkpoint_keeps_zero_velocity_node_alive(ros_runtime):
    ros_runtime.params["checkpoint"] = ""
    node = ros_runtime.module.VisualSteeringNode()
    assert node.core is None
    node.on_image(valid_image())
    assert_stopped(node)
    assert node.errors


@pytest.mark.parametrize("failure", ["empty", "decode", "inference"])
def test_failed_frame_stops_and_next_valid_frame_recovers(ros_runtime, failure):
    node = make_node(ros_runtime)
    node.on_image(valid_image())
    assert node.messages[-1].linear.x == 1.0
    image = valid_image()
    if failure == "empty":
        image.data = b""
    elif failure == "decode":
        node.bridge.imgmsg_to_cv2 = lambda *args, **kwargs: (_ for _ in ()).throw(ValueError("decode"))
    else:
        node.core.predict = lambda image: (_ for _ in ()).throw(RuntimeError("inference"))
    node.on_image(image)
    assert_stopped(node)
    assert node._last_image_time is None
    node.bridge = ros_runtime.module.CvBridge()
    node.core = ros_runtime.core()
    node.on_image(valid_image())
    assert node.messages[-1].linear.x == 1.0


def test_watchdog_stops_after_image_stream_timeout(ros_runtime):
    node = make_node(ros_runtime)
    node.on_image(valid_image())
    ros_runtime.clock[0] += 0.6
    node.check_timeout()
    assert_stopped(node)


def test_overdue_inference_does_not_publish_stale_motion(ros_runtime):
    node = make_node(ros_runtime)

    def slow_inference(image):
        ros_runtime.clock[0] += 0.6
        return ros_runtime.module.TwistCommand(1.0, 0.65, "LEFT", 1.0)

    node.core.predict = slow_inference
    node.on_image(valid_image())
    assert_stopped(node)
    assert not any(msg.linear.x for msg in node.messages)


def test_shutdown_publishes_zero_and_releases_node(ros_runtime):
    node = make_node(ros_runtime)
    ros_runtime.monkeypatch.setattr(ros_runtime.module, "VisualSteeringNode", lambda: node)
    ros_runtime.ros.spin = lambda node: (_ for _ in ()).throw(KeyboardInterrupt())
    assert ros_runtime.module.ros_main() == 0
    assert_stopped(node)
    assert node.destroyed
    assert not ros_runtime.active[0]


def test_constructor_failure_still_shuts_down_ros(ros_runtime):
    def fail():
        raise RuntimeError("construction error")

    ros_runtime.monkeypatch.setattr(ros_runtime.module, "VisualSteeringNode", fail)
    with pytest.raises(RuntimeError, match="construction error"):
        ros_runtime.module.ros_main()
    assert not ros_runtime.active[0]


@pytest.mark.parametrize("index,angular,linear", [(0, 0.65, 1.0), (1, 0.0, 1.0),
                                               (2, -0.65, 1.0), (3, 0.0, 0.0)])
def test_inference_labels_follow_ros_positive_left_convention(ros_runtime, index, angular, linear):
    torch = pytest.importorskip("torch")
    module = ros_runtime.module
    core = module.VisualSteeringCore.__new__(module.VisualSteeringCore)
    core.torch = torch
    core.steering_rate = 0.65

    class FixedLogits:
        image_height = 90
        image_width = 160
        labels = ("LEFT", "STRAIGHT", "RIGHT", "STOP")

        def __call__(self, images):
            output = torch.zeros((1, 4))
            output[0, index] = 10.0
            return output

    core.model = FixedLogits()
    command = core.predict(Image.new("RGB", (160, 90)))
    assert command.angular_z == angular
    assert command.linear_x == linear
