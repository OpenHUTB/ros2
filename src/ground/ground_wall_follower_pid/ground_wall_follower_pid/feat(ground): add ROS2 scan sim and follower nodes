"""ROS 2 nodes for the right-wall PID follower.

Topics (all ``std_msgs/String`` carrying JSON for zero extra deps):

* ``/wall_follower/scan``     : published by :class:`ScanSimNode`,
  keys ``side`` (right-wall distance) and ``forward``.
* ``/wall_follower/cmd_vel``  : published by :class:`WallFollowerNode`,
  keys ``linear`` and ``angular``.
* ``/wall_follower/state``    : published by :class:`WallFollowerNode`,
  keys ``state``, ``linear``, ``angular``.
"""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core import WallFollower


class ScanSimNode(Node):
    """Minimal 2D differential-drive scan simulator.

    The right wall sits on ``y = 0`` and the forward corridor ends at
    ``x = 10.0``. The node integrates the incoming ``/wall_follower/cmd_vel``
    message and publishes the resulting ``side`` / ``forward`` distances.
    """

    def __init__(self) -> None:
        super().__init__("wall_scan_sim")
        self._x = 0.0
        self._y = 0.55  # starting distance to the right wall
        self._yaw = 0.0
        self._v = 0.0
        self._w = 0.0

        self.create_subscription(
            String, "/wall_follower/cmd_vel", self._on_cmd, 10
        )
        self._pub = self.create_publisher(String, "/wall_follower/scan", 10)
        self._timer = self.create_timer(0.1, self._tick)

    def _on_cmd(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._v = float(data.get("linear", 0.0))
            self._w = float(data.get("angular", 0.0))
        except (json.JSONDecodeError, ValueError):
            self._v = 0.0
            self._w = 0.0

    def _tick(self) -> None:
        dt = 0.1
        self._yaw += self._w * dt
        self._x += self._v * math.cos(self._yaw) * dt
        self._y += self._v * math.sin(self._yaw) * dt

        side = max(0.05, self._y)
        forward = max(0.2, 10.0 - self._x)
        self._pub.publish(String(data=json.dumps(
            {"side": round(side, 3), "forward": round(forward, 3)}
        )))


class WallFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_follower")
        self._follower = WallFollower()
        self._last = self.get_clock().now()

        self._cmd_pub = self.create_publisher(String, "/wall_follower/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/wall_follower/state", 10)
        self.create_subscription(
            String, "/wall_follower/scan", self._on_scan, 10
        )

    def _on_scan(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            side = float(data.get("side", 0.5))
            forward = float(data.get("forward", 2.0))
        except (json.JSONDecodeError, ValueError):
            return

        now = self.get_clock().now()
        dt = max(0.01, (now - self._last).nanoseconds / 1e9)
        self._last = now

        cmd = self._follower.compute(side, forward, dt)
        self._cmd_pub.publish(String(data=json.dumps(
            {"linear": round(cmd.linear, 3), "angular": round(cmd.angular, 3)}
        )))
        self._state_pub.publish(String(data=json.dumps({
            "state": cmd.state,
            "linear": round(cmd.linear, 3),
            "angular": round(cmd.angular, 3),
        })))


def scan_main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def follower_main(args=None) -> None:
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
