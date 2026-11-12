#!/usr/bin/env python3
"""ROS 1 (Noetic) node for the neural collision-avoidance stack.

The same file serves every module of the assignment; the launch files simply
select it with the ``~module`` parameter (``perception``, ``planning``,
``control``, ``e2e`` or ``full``).  All heavy lifting is delegated to the
``ca_drone`` package, therefore the node can be unit-tested without a running
ROS master (see ``tools/ros_smoke_test.py``).

Topics
------
subscribed : ``/camera/depth/image_raw`` (sensor_msgs/Image, 32FC1 or 16UC1)
             ``/odom``                  (nav_msgs/Odometry)
             ``/move_base_simple/goal`` (geometry_msgs/PoseStamped, optional)
published  : ``/cmd_vel``               (geometry_msgs/Twist, body vx/vy + yaw rate)
             ``/ca/occupancy``          (nav_msgs/OccupancyGrid)
             ``/ca/path``               (nav_msgs/Path)
             ``/ca/status``             (std_msgs/String)
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Make the repository importable when it was not pip-installed.
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = _HERE
for _ in range(8):
    if os.path.isdir(os.path.join(_ROOT, "ca_drone")):
        break
    _ROOT = os.path.dirname(_ROOT)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ca_drone.control.expert import lookahead_point  # noqa: E402
from ca_drone.mapping import GridMap, patch_clearance  # noqa: E402
from ca_drone.models.control import control_features, predict_command  # noqa: E402
from ca_drone.models.e2e import goal_features  # noqa: E402
from ca_drone.models.perception import predict_mask  # noqa: E402
from ca_drone.nn.autograd import Tensor  # noqa: E402
from ca_drone.pipeline import (DEPTH_H, DEPTH_W, FOV, MAX_CMD, MAX_RANGE,  # noqa: E402
                               NeuralStack, _plan)


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _decode_image(msg: Image) -> np.ndarray:
    """Convert a ROS depth image to a 48x64 float32 range image [m]."""
    h, w = int(msg.height), int(msg.width)
    if msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
    elif msg.encoding == "16UC1":
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).astype(np.float32) / 1000.0
    elif msg.encoding in ("8UC1", "mono8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w).astype(np.float32) / 255.0 * MAX_RANGE
    else:
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    ys = np.linspace(0, h - 1, DEPTH_H).astype(int)
    xs = np.linspace(0, w - 1, DEPTH_W).astype(int)
    out = arr[np.ix_(ys, xs)]
    out = np.where(np.isfinite(out), out, MAX_RANGE)
    return np.clip(out, 0.0, MAX_RANGE).astype(np.float32)


class NeuralDroneNode:
    def __init__(self):
        self.module = rospy.get_param("~module", "full")
        self.weights = rospy.get_param("~weights", os.path.join(_ROOT, "weights"))
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.goal = np.array([
            float(rospy.get_param("~goal_x", 49.0)),
            float(rospy.get_param("~goal_y", 0.0)),
            float(rospy.get_param("~goal_z", 3.0))])
        self.replan_period = float(rospy.get_param("~replan_period", 1.0))

        self.stack = NeuralStack(self.weights)
        self.gm = GridMap(res=0.5, x0=0.0, y0=-22.0, nx=104, ny=88)
        self.depth = None
        self.depth_stamp = 0.0
        self.pos = np.zeros(3)
        self.yaw = 0.0
        self.vel = np.zeros(3)
        self.path = None
        self.last_plan = -1e9
        self.frames = 0

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_occ = rospy.Publisher("/ca/occupancy", OccupancyGrid, queue_size=1)
        self.pub_path = rospy.Publisher("/ca/path", Path, queue_size=1)
        self.pub_status = rospy.Publisher("/ca/status", String, queue_size=1)

        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb, queue_size=1,
                         buff_size=2 ** 24)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)
        rospy.loginfo("neural_ca_drone_ros started: module=%s weights=%s",
                      self.module, self.weights)

    # ------------------------------------------------------------- callbacks
    def depth_cb(self, msg: Image):
        try:
            self.depth = _decode_image(msg)
            self.depth_stamp = rospy.get_time()
        except Exception as exc:  # pragma: no cover - depends on the driver
            rospy.logwarn_throttle(5.0, "depth decode failed: %s", exc)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear
        self.vel = np.array([v.x, v.y, v.z])

    def goal_cb(self, msg: PoseStamped):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y,
                              msg.pose.position.z or 3.0])
        rospy.loginfo("new goal: %s", np.round(self.goal, 2))

    # ------------------------------------------------------------- main loop
    def tick(self, _event):
        if self.depth is None:
            self.pub_status.publish(String("waiting for /camera/depth/image_raw"))
            return
        if rospy.get_time() - self.depth_stamp > 1.0:
            self.pub_cmd.publish(Twist())  # stale sensor -> hover
            self.pub_status.publish(String("stale depth -> hover"))
            return

        depth = self.depth
        prob = None
        if self.module in ("perception", "planning", "control", "full"):
            prob = predict_mask(self.stack.perception, depth, MAX_RANGE)
            self.gm.integrate_depth(depth, prob, self.pos, self.yaw, FOV, MAX_RANGE,
                                    threshold=0.55)
        if self.module in ("perception", "full"):
            self.publish_occupancy()
        if self.module in ("planning", "control", "full"):
            self.maybe_plan()
        if self.module in ("control", "full"):
            self.publish_command(self.control_command())
        elif self.module == "e2e":
            self.publish_command(self.e2e_command(depth))
        elif self.module == "planning":
            self.publish_zero_velocity()

    # --------------------------------------------------------------- helpers
    def maybe_plan(self):
        now = rospy.get_time()
        if self.path is not None and now - self.last_plan < self.replan_period:
            return
        r = _plan(self.stack, self.gm, self.pos, self.goal, use_neural=True)
        if r.get("success"):
            self.path = r["world_path"]
            self.publish_path(self.path)
        self.last_plan = now

    def control_command(self) -> np.ndarray:
        if self.path is None or len(self.path) < 2:
            return np.zeros(2)
        target = lookahead_point(self.path, self.pos, 2.0)
        delta = target[:2] - self.pos[:2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
        patch = self.gm.local_patch(self.pos, 9)
        feats = control_features(patch, gb, np.array([self.vel[0], self.vel[1]]),
                                 patch_clearance(patch, 0.5))
        cmd = predict_command(self.stack.control, feats)
        n = float(np.linalg.norm(cmd))
        return cmd * (MAX_CMD / n) if n > MAX_CMD else cmd

    def e2e_command(self, depth: np.ndarray) -> np.ndarray:
        delta = self.goal[:2] - self.pos[:2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
        feat = goal_features(gb, float(np.linalg.norm(delta)))
        cmd = self.stack.e2e(Tensor(np.clip(depth / MAX_RANGE, 0, 1)[None, None]),
                             Tensor(feat[None])).data[0]
        return np.asarray(cmd, dtype=float)

    def publish_command(self, cmd: np.ndarray):
        msg = Twist()
        msg.linear.x = float(np.clip(cmd[0], -1.5, 1.5))
        msg.linear.y = float(np.clip(cmd[1], -1.5, 1.5))
        # Align the heading with the commanded velocity (coordinated flight).
        speed = float(np.hypot(msg.linear.x, msg.linear.y))
        if speed > 0.15:
            cmd_yaw = math.atan2(msg.linear.y, msg.linear.x)
            err = (cmd_yaw + self.yaw + math.pi) % (2 * math.pi) - math.pi
            msg.angular.z = float(np.clip(1.5 * err, -1.2, 1.2))
        self.pub_cmd.publish(msg)
        self.pub_status.publish(String(
            f"module={self.module} cmd=({msg.linear.x:.2f},{msg.linear.y:.2f}) "
            f"yaw_rate={msg.angular.z:.2f} speed={speed:.2f}"))

    def publish_zero_velocity(self):
        self.pub_cmd.publish(Twist())

    def publish_occupancy(self):
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.5
        grid.info.width = self.gm.nx
        grid.info.height = self.gm.ny
        grid.info.origin.position.x = self.gm.x0
        grid.info.origin.position.y = self.gm.y0
        data = self.gm.occ.astype(np.int8).reshape(-1) * 100
        grid.data = data.tolist()
        self.pub_occ.publish(grid)

    def publish_path(self, pts: np.ndarray):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        for p in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2]) if len(p) > 2 else 3.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)


def main() -> int:
    rospy.init_node("neural_ca_drone", anonymous=False)
    NeuralDroneNode()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
