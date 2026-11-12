"""测试用的最小桩：rclpy / sensor_msgs / cv_bridge / cv2。

被测的是节点参数解析、话题名规范化、保存路径与保存调用，不需要真的 ROS 或 OpenCV。
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

WRITTEN: list[str] = []
IMWRITE_RESULT = [True]
IMSHOW_CALLS: list[str] = []


def _install_stubs() -> None:
    for name in ("rclpy", "rclpy.node", "sensor_msgs", "sensor_msgs.msg",
                 "cv_bridge", "cv2"):
        sys.modules.pop(name, None)

    class _Param:
        def __init__(self, value):
            self.value = value

    class _Logger:
        def __init__(self):
            self.infos = []
            self.errors = []

        def info(self, msg):
            self.infos.append(str(msg))

        def error(self, msg):
            self.errors.append(str(msg))

    class Node:
        PARAMS: dict = {}
        INSTANCES: list = []

        def __init__(self, name):
            self._name = name
            self._params = {}
            self.logger = _Logger()
            self.destroyed = False
            Node.INSTANCES.append(self)

        def declare_parameter(self, name, default=None):
            value = Node.PARAMS.get(name, default)
            self._params[name] = value
            return _Param(value)

        def create_subscription(self, msg_type, topic, callback, qos):
            self.subscription_topic = topic
            self.subscription_callback = callback
            return object()

        def get_logger(self):
            return self.logger

        def destroy_node(self):
            self.destroyed = True

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node
    rclpy.node = node_mod

    exec_mod = types.ModuleType("rclpy.executors")

    class ExternalShutdownException(Exception):
        pass

    exec_mod.ExternalShutdownException = ExternalShutdownException
    rclpy.executors = exec_mod

    sensor_msgs = types.ModuleType("sensor_msgs")
    smsg = types.ModuleType("sensor_msgs.msg")

    class Image:
        pass

    smsg.Image = Image
    sensor_msgs.msg = smsg

    cv_bridge = types.ModuleType("cv_bridge")

    class CvBridge:
        def imgmsg_to_cv2(self, msg, encoding):
            return f"image:{encoding}"

    cv_bridge.CvBridge = CvBridge

    cv2 = types.ModuleType("cv2")

    def imwrite(path, image):
        WRITTEN.append(str(path))
        return IMWRITE_RESULT[0]

    def imshow(title, image):
        IMSHOW_CALLS.append(title)

    cv2.imwrite = imwrite
    cv2.imshow = imshow
    cv2.waitKey = lambda delay: 13
    cv2.destroyAllWindows = lambda: None

    globals()["StubNode"] = Node

    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": node_mod,
        "rclpy.executors": exec_mod,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": smsg,
        "cv_bridge": cv_bridge,
        "cv2": cv2,
    })


_install_stubs()
