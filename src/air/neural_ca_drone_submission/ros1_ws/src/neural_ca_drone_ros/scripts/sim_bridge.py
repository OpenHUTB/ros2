#!/usr/bin/env python3
"""CA-Sim <-> ROS 1 bridge.

Runs the light-weight CA-Sim world inside the ROS node graph so that the whole
perception -> planning -> control stack can be launched with a single
``roslaunch`` command, without a GPU or a Gazebo installation::

    roslaunch neural_ca_drone_ros main.launch scene:=dense module:=full

Published topics
----------------
``/camera/depth/image_raw`` : sensor_msgs/Image (32FC1, metres, 64 x 48)
``/odom``                   : nav_msgs/Odometry (map frame)
``/tf``                     : map -> base_link
``/ca/sim_status``          : std_msgs/String

Subscribed topics
-----------------
``/cmd_vel``                : geometry_msgs/Twist (body vx, vy, yaw rate)
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = _HERE
for _ in range(8):
    if os.path.isdir(os.path.join(_ROOT, "ca_drone")):
        break
    _ROOT = os.path.dirname(_ROOT)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ca_drone.pipeline import DEPTH_H, DEPTH_W, FOV, MAX_RANGE  # noqa: E402
from ca_drone.render.scene import depth_image  # noqa: E402
from ca_drone.sim import make_world  # noqa: E402
from ca_drone.sim.drone import Drone  # noqa: E402


class SimBridge:
    def __init__(self):
        self.scene = rospy.get_param("~scene", "sparse")
        self.rate = float(rospy.get_param("~rate", 10.0))
        self.world = make_world(self.scene)
        self.drone = Drone(self.world)
        self.drone.dt = 1.0 / self.rate
        self.cmd = np.zeros(3, dtype=float)
        self.reached = False
        self.steps = 0

        self.pub_depth = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=1)
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.pub_status = rospy.Publisher("/ca/sim_status", String, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
        rospy.loginfo("CA-Sim bridge: scene=%s start=%s goal=%s",
                      self.scene, np.round(self.world.start, 2),
                      np.round(self.world.goal, 2))

    def cmd_cb(self, msg: Twist):
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)

    def tick(self, _event):
        dt = 1.0 / self.rate
        self.drone.step(self.cmd[:2])
        self.world.t = self.drone.t
        self.steps += 1
        if not self.reached and self.drone.reached():
            self.reached = True
            rospy.loginfo("goal reached in %d steps", self.steps)
        state = self.drone.state
        self.publish_odom(state, dt)
        self.publish_depth(state)
        if self.steps % 10 == 0:
            self.pub_status.publish(String(
                "scene=%s pos=(%.2f,%.2f) clear=%.2f reached=%s" % (
                    self.scene, state.pos[0], state.pos[1],
                    self.world.clearance(state.pos), self.reached)))

    def publish_odom(self, state, dt):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(state.pos[0])
        msg.pose.pose.position.y = float(state.pos[1])
        msg.pose.pose.position.z = float(state.pos[2])
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, float(state.yaw))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = float(q[0]), float(q[1])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = float(q[2]), float(q[3])
        v = self.drone.to_world(self.drone.body_velocity())
        msg.twist.twist.linear.x = float(v[0])
        msg.twist.twist.linear.y = float(v[1])
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = 0.0
        self.pub_odom.publish(msg)
        self.tf_broadcaster.sendTransform(
            (float(state.pos[0]), float(state.pos[1]), float(state.pos[2])),
            (float(q[0]), float(q[1]), float(q[2]), float(q[3])),
            rospy.Time.now(), "base_link", "map")

    def publish_depth(self, state):
        depth, ids = depth_image(self.world, state.pos, state.yaw,
                                 width=DEPTH_W, height=DEPTH_H, fov_deg=FOV,
                                 max_range=MAX_RANGE)
        depth = np.where(ids < 0, MAX_RANGE, depth).astype(np.float32)
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.height = int(DEPTH_H)
        msg.width = int(DEPTH_W)
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = int(DEPTH_W * 4)
        msg.data = depth.tobytes()
        self.pub_depth.publish(msg)


def main():
    rospy.init_node("ca_sim_bridge", anonymous=False)
    SimBridge()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
