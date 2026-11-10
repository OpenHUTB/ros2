"""Node-level tests for the laser-scan fail-safe.

The ROS node is imported with lightweight ``rospy``/message stubs so the tests
run on machines without ROS.  Only the control loop and the command publisher
are exercised; the DRL policy is replaced by ``mode:=baseline``.

Run with::

    python3 -m unittest discover -s tests -v
"""

from __future__ import annotations

import math
import os
import sys
import types
import unittest

import numpy as np

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(os.path.dirname(TESTS_DIR), "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

CONFIG_PATH = os.path.join(
    os.path.dirname(TESTS_DIR), "config", "drl_nav.yaml"
)


# ---------------------------------------------------------------------------
# Minimal ROS 1 stubs
# ---------------------------------------------------------------------------


class FakeTime:
    def __init__(self, seconds: float = 0.0) -> None:
        self.seconds = float(seconds)

    @classmethod
    def now(cls) -> "FakeTime":
        return FakeClock.now()

    def to_sec(self) -> float:
        return self.seconds

    def __sub__(self, other: "FakeTime") -> "FakeDuration":
        return FakeDuration(self.seconds - other.to_sec())


class FakeDuration:
    def __init__(self, seconds: float = 0.0) -> None:
        self.seconds = float(seconds)

    def to_sec(self) -> float:
        return self.seconds


class FakeClock:
    seconds = 100.0

    @classmethod
    def now(cls) -> FakeTime:
        return FakeTime(cls.seconds)

    @classmethod
    def advance(cls, delta: float) -> None:
        cls.seconds += delta

    @classmethod
    def reset(cls, seconds: float = 100.0) -> None:
        cls.seconds = seconds


class Vector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Quaternion:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class Pose:
    def __init__(self) -> None:
        self.position = Vector3()
        self.orientation = Quaternion()


class Header:
    def __init__(self, frame_id: str = "odom") -> None:
        self.frame_id = frame_id
        self.stamp = FakeTime(0.0)


class Twist:
    def __init__(self) -> None:
        self.linear = Vector3()
        self.angular = Vector3()


class Odometry:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 2.0) -> None:
        self.header = Header()
        self.pose = types.SimpleNamespace(pose=Pose())
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        self.pose.pose.orientation.w = 1.0
        self.twist = types.SimpleNamespace(twist=Twist())


class LaserScan:
    def __init__(self, ranges: np.ndarray) -> None:
        self.header = Header()
        self.ranges = [float(value) for value in ranges]
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / len(self.ranges)
        self.range_min = 0.1
        self.range_max = 50.0


class Path:
    def __init__(self) -> None:
        self.header = Header()
        self.poses = []


class PoseStamped:
    def __init__(self) -> None:
        self.header = Header()
        self.pose = Pose()


class String:
    def __init__(self) -> None:
        self.data = ""


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


LOGS = []
PARAMS = {
    "~config_file": CONFIG_PATH,
    "~mode": "baseline",
    "~odom_topic": "/uav/odom",
    "~scan_topic": "/uav/scan",
    "~cmd_topic": "/uav/cmd_vel",
    "~path_topic": "~path",
    "~status_topic": "~status",
    "~command_frame": "body",
    "~control_rate": 10.0,
    "~frame_id": "odom",
    "~scan_timeout": 1.0,
}


def _record(level):
    def _log(message, *args):
        LOGS.append((level, message % args if args else message))
    return _log


def _record_throttle(level):
    def _log(period, message, *args):
        LOGS.append((level, message % args if args else message))
    return _log


def _install_ros_stubs() -> None:
    rospy = types.ModuleType("rospy")
    rospy.Time = FakeTime
    rospy.Duration = FakeDuration
    rospy.get_param = lambda name, default=None: PARAMS.get(name, default)
    rospy.Publisher = lambda *args, **kwargs: RecordingPublisher()
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.on_shutdown = lambda callback: None
    rospy.Rate = lambda rate: None
    rospy.is_shutdown = lambda: False
    rospy.loginfo = _record("INFO")
    rospy.logwarn = _record("WARN")
    rospy.logwarn_throttle = _record_throttle("WARN")
    rospy.logerr = _record("ERROR")
    rospy.logerr_throttle = _record_throttle("ERROR")
    rospy.logfatal = _record("FATAL")
    sys.modules["rospy"] = rospy

    messages = {
        "geometry_msgs": {"PoseStamped": PoseStamped, "Twist": Twist},
        "nav_msgs": {"Odometry": Odometry, "Path": Path},
        "sensor_msgs": {"LaserScan": LaserScan},
        "std_msgs": {"String": String},
    }
    for package, classes in messages.items():
        package_module = types.ModuleType(package)
        message_module = types.ModuleType(package + ".msg")
        for name, cls in classes.items():
            setattr(message_module, name, cls)
        package_module.msg = message_module
        sys.modules[package] = package_module
        sys.modules[package + ".msg"] = message_module


_install_ros_stubs()
from drl_nav_node import DrlNavNode  # noqa: E402


class LaserSafetyTest(unittest.TestCase):
    """Missing or stale laser data must latch a zero-velocity command."""

    def setUp(self) -> None:
        FakeClock.reset()
        LOGS.clear()
        self.node = DrlNavNode()
        self.node._odom_callback(Odometry())

    def _assert_stopped(self, twist: Twist) -> None:
        self.assertEqual(twist.linear.x, 0.0)
        self.assertEqual(twist.linear.y, 0.0)
        self.assertEqual(twist.linear.z, 0.0)
        self.assertEqual(twist.angular.x, 0.0)
        self.assertEqual(twist.angular.y, 0.0)
        self.assertEqual(twist.angular.z, 0.0)

    def test_no_scan_at_start_publishes_zero(self) -> None:
        self.node._tick()
        self.assertTrue(self.node.cmd_pub.messages)
        self._assert_stopped(self.node.cmd_pub.messages[-1])
        self.assertTrue(
            any("no LaserScan" in message for _, message in LOGS)
        )

    def test_stale_scan_publishes_zero(self) -> None:
        self.node._scan_callback(LaserScan(np.full(360, 50.0)))
        FakeClock.advance(1.5)
        self.node._tick()
        self._assert_stopped(self.node.cmd_pub.messages[-1])
        self.assertTrue(any("stale" in message for _, message in LOGS))

    def test_fresh_scan_resumes_control(self) -> None:
        self.node._tick()
        self._assert_stopped(self.node.cmd_pub.messages[-1])

        self.node._scan_callback(LaserScan(np.full(360, 50.0)))
        self.node._tick()
        command = self.node.cmd_pub.messages[-1]
        self.assertGreater(command.linear.x, 0.0)
        self.assertTrue(
            any("resuming control" in message for _, message in LOGS)
        )


if __name__ == "__main__":
    unittest.main()
