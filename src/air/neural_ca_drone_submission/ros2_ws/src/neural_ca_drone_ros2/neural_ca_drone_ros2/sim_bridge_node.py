#!/usr/bin/env python3
"""CA-Sim bridge for ROS 2: publishes depth + odometry, consumes /cmd_vel."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .common import add_repo_to_path, yaw_to_quat

_ROOT = add_repo_to_path()

from ca_drone.pipeline import DEPTH_H, DEPTH_W, FOV, MAX_RANGE  # noqa: E402
from ca_drone.render.scene import depth_image  # noqa: E402
from ca_drone.sim import make_world  # noqa: E402
from ca_drone.sim.drone import Drone  # noqa: E402


class SimBridge(Node):
    def __init__(self):
        super().__init__("ca_sim_bridge")
        self.declare_parameter("scene", "sparse")
        self.declare_parameter("rate", 10.0)
        self.scene = self.get_parameter("scene").value
        rate = float(self.get_parameter("rate").value)

        self.world = make_world(self.scene)
        self.drone = Drone(self.world)
        self.drone.dt = 1.0 / rate
        self.cmd = np.zeros(2)
        self.reached = False
        self.steps = 0

        self.pub_depth = self.create_publisher(Image, "/camera/depth/image_raw", 1)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 1)
        self.pub_status = self.create_publisher(String, "/ca/sim_status", 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 1)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info("CA-Sim bridge: scene=%s" % self.scene)

    def cmd_cb(self, msg: Twist):
        self.cmd = np.array([msg.linear.x, msg.linear.y])

    def tick(self):
        self.drone.step(self.cmd)
        self.world.t = self.drone.t
        self.steps += 1
        if not self.reached and self.drone.reached():
            self.reached = True
            self.get_logger().info("goal reached in %d steps" % self.steps)
        state = self.drone.state
        self.publish_odom(state)
        self.publish_depth(state)
        if self.steps % 10 == 0:
            self.pub_status.publish(String(data=(
                "scene=%s pos=(%.2f,%.2f) clear=%.2f reached=%s" % (
                    self.scene, state.pos[0], state.pos[1],
                    self.world.clearance(state.pos), self.reached))))

    def publish_odom(self, state):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(state.pos[0])
        msg.pose.pose.position.y = float(state.pos[1])
        msg.pose.pose.position.z = float(state.pos[2])
        qx, qy, qz, qw = yaw_to_quat(float(state.yaw))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        v = self.drone.to_world(self.drone.body_velocity())
        msg.twist.twist.linear.x = float(v[0])
        msg.twist.twist.linear.y = float(v[1])
        self.pub_odom.publish(msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = float(state.pos[0])
        tf_msg.transform.translation.y = float(state.pos[1])
        tf_msg.transform.translation.z = float(state.pos[2])
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y = qx, qy
        tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_depth(self, state):
        depth, ids = depth_image(self.world, state.pos, state.yaw,
                                 width=DEPTH_W, height=DEPTH_H, fov_deg=FOV,
                                 max_range=MAX_RANGE)
        depth = np.where(ids < 0, MAX_RANGE, depth).astype(np.float32)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height = int(DEPTH_H)
        msg.width = int(DEPTH_W)
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = int(DEPTH_W * 4)
        msg.data = depth.tobytes()
        self.pub_depth.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
