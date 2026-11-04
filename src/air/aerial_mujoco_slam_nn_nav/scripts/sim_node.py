#!/usr/bin/env python3
"""无人机仿真节点：MuJoCo 物理 + 飞控 + 传感器广播。

订阅：/cmd_vel (Twist: linear.x=vx, linear.y=vy, angular.z=yaw_rate)、/drone/altitude (Float64)
发布：/scan、/camera/depth/image_raw、/drone/odom、/drone/ground_truth、/drone/imu、/tf
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Empty, Float64

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aerial_mujoco_slam_nn_nav.sensors import DepthCamera, IMU, Lidar, Odometry as Odom  # noqa: E402
from aerial_mujoco_slam_nn_nav.sim import DroneSim                                     # noqa: E402


class SimNode:
    def __init__(self):
        scene = rospy.get_param("~scene", "warehouse")
        self.rate = float(rospy.get_param("~rate", 100.0))
        self.sim = DroneSim(scene=scene)
        self.lidar = Lidar(self.sim.model, n_beams=180)
        self.depth = DepthCamera(self.sim.model, 32, 24)
        self.imu = IMU()
        self.odom = Odom()
        sx, sy, syaw = self.sim.layout.start
        self.odom.reset(sx, sy, syaw)
        self._cmd = (0.0, 0.0, 0.0)
        self._alt = 1.2
        self._last_cmd = rospy.Time.now()
        self._last_scan = rospy.Time(0)
        self._last_depth = rospy.Time(0)

        self.pub_scan = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.pub_depth = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=1)
        self.pub_odom = rospy.Publisher("/drone/odom", Odometry, queue_size=1)
        self.pub_gt = rospy.Publisher("/drone/ground_truth", PoseStamped, queue_size=1)
        self.pub_imu = rospy.Publisher("/drone/imu", Imu, queue_size=1)
        self.tf = tf.TransformBroadcaster()

        rospy.Subscriber("/cmd_vel", Twist, self._on_cmd, queue_size=1)
        rospy.Subscriber("/drone/altitude", Float64, self._on_alt, queue_size=1)
        rospy.Subscriber("/drone/reset", Empty, self._on_reset, queue_size=1)

    def _on_cmd(self, msg):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd = rospy.Time.now()

    def _on_alt(self, msg):
        self._alt = float(msg.data)

    def _on_reset(self, _msg):
        self.sim.reset()
        sx, sy, syaw = self.sim.layout.start
        self.odom.reset(sx, sy, syaw)

    def spin(self):
        rate = rospy.Rate(self.rate)
        dt = 1.0 / self.rate
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self._last_cmd).to_sec() > 0.5:
                self._cmd = (0.0, 0.0, 0.0)
            self.sim.set_command(vx=self._cmd[0], vy=self._cmd[1],
                                 yaw_rate=self._cmd[2], z_ref=self._alt)
            self.sim.step_dt(dt)
            self._publish(now)
            rate.sleep()

    def _publish(self, now):
        st = self.sim.state()
        gt = PoseStamped()
        gt.header.stamp = now
        gt.header.frame_id = "world"
        gt.pose.position.x, gt.pose.position.y, gt.pose.position.z = st["pos"]
        q = tf.transformations.quaternion_from_euler(st["roll"], st["pitch"], st["yaw"])
        gt.pose.orientation.x, gt.pose.orientation.y = q[0], q[1]
        gt.pose.orientation.z, gt.pose.orientation.w = q[2], q[3]
        self.pub_gt.publish(gt)
        self.tf.sendTransform(tuple(st["pos"]), q, now, "base_link", "world")

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "world"
        od.child_frame_id = "base_link"
        o = self.odom.update(st["vel"][0], st["vel"][1], 0.0, 0.1)
        od.pose.pose.position.x, od.pose.pose.position.y = o["x"], o["y"]
        q2 = tf.transformations.quaternion_from_euler(0, 0, o["yaw"])
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = q2[2], q2[3]
        od.twist.twist.linear.x = st["vel"][0]
        od.twist.twist.linear.y = st["vel"][1]
        self.pub_odom.publish(od)

        im = Imu()
        im.header.stamp = now
        im.header.frame_id = "base_link"
        self.pub_imu.publish(im)

        if (now - self._last_scan).to_sec() >= 0.1:
            self._last_scan = now
            ranges = self.lidar.scan(self.sim.data)
            msg = LaserScan()
            msg.header.stamp = now
            msg.header.frame_id = "base_link"
            msg.angle_min = -math.pi
            msg.angle_max = math.pi
            msg.angle_increment = 2 * math.pi / len(ranges)
            msg.range_min = 0.05
            msg.range_max = 8.0
            msg.ranges = [float(r) for r in ranges]
            self.pub_scan.publish(msg)

        if (now - self._last_depth).to_sec() >= 0.2:
            self._last_depth = now
            depth = self.depth.depth(self.sim.data).astype(np.float32)
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = "fpv"
            msg.height, msg.width = depth.shape
            msg.encoding = "32FC1"
            msg.step = depth.shape[1] * 4
            msg.data = depth.tobytes()
            self.pub_depth.publish(msg)


if __name__ == "__main__":
    rospy.init_node("drone_sim")
    SimNode().spin()
