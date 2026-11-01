#!/usr/bin/env python3
"""MuJoCo simulation node: physics, lidar, RGB-D/BEV cameras, odometry and TF."""

from __future__ import annotations

import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from gvnn.runtime import ensure_venv

ensure_venv(__file__)

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage

from gvnn.config import LIDAR_BEAMS, LIDAR_RANGE_MAX, LIDAR_RANGE_MIN, MAX_OMEGA, MAX_SPEED
from gvnn.ros_utils import (depth_image, image, imu_message, laser_scan,
                            odometry, pose_stamped, tf_message, transform)
from gvnn.sim.mujoco_backend import MuJoCoSimulation


class SimNode:
    def __init__(self) -> None:
        self.scene_name = rospy.get_param("~scene", "warehouse")
        self.seed = int(rospy.get_param("~seed", 20261026))
        self.noise = bool(rospy.get_param("~noise", True))
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.sensor_hz = float(rospy.get_param("~sensor_hz", 10.0))
        self.sim = MuJoCoSimulation(self.scene_name, seed=self.seed, noise=self.noise)
        self.cmd = [0.0, 0.0]
        self.cmd_stamp = rospy.Time.now()
        self.last_sensor = rospy.Time(0)
        self.last_status = rospy.Time(0)

        self.pub_scan = rospy.Publisher("/scan", LaserScan, queue_size=5)
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.pub_truth = rospy.Publisher("/ground_truth/pose", PoseStamped, queue_size=5)
        self.pub_imu = rospy.Publisher("/imu", Imu, queue_size=5)
        self.pub_bev = rospy.Publisher("/camera/bev/image_raw", Image, queue_size=2)
        self.pub_rgb = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size=2)
        self.pub_depth = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=2)
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.pub_state = rospy.Publisher("/gvnn/state", String, queue_size=5)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=5)
        rospy.Subscriber("/gvnn/reset", Empty, self.reset_cb, queue_size=2)
        rospy.Subscriber("/gvnn/scene", String, self.scene_cb, queue_size=2)
        rospy.on_shutdown(self.shutdown)

    # ------------------------------------------------------------------ callbacks
    def cmd_cb(self, msg: Twist) -> None:
        self.cmd[0] = max(-0.6 * MAX_SPEED, min(MAX_SPEED, float(msg.linear.x)))
        self.cmd[1] = max(-MAX_OMEGA, min(MAX_OMEGA, float(msg.angular.z)))
        self.cmd_stamp = rospy.Time.now()

    def reset_cb(self, _msg: Empty) -> None:
        self.sim.reset()
        self.cmd = [0.0, 0.0]
        rospy.loginfo("simulation reset")

    def scene_cb(self, msg: String) -> None:
        name = msg.data.strip()
        if name and name != self.scene_name:
            try:
                self.sim = MuJoCoSimulation(name, seed=self.seed, noise=self.noise)
                self.scene_name = name
                self.cmd = [0.0, 0.0]
                rospy.loginfo("scene -> %s", name)
            except Exception as exc:
                rospy.logwarn("cannot switch scene: %s", exc)

    # ------------------------------------------------------------------ publish
    def publish_sensors(self, stamp) -> None:
        ranges = self.sim.scan()
        self.pub_scan.publish(laser_scan(ranges, stamp, "laser",
                                         angle_min=0.0,
                                         angle_increment=2.0 * math.pi / LIDAR_BEAMS,
                                         range_min=LIDAR_RANGE_MIN,
                                         range_max=LIDAR_RANGE_MAX))
        rgb, depth = self.sim.depth_image(ranges)
        self.pub_rgb.publish(image(rgb, stamp, "camera_rgb"))
        self.pub_depth.publish(depth_image(depth, stamp, "camera_depth"))
        self.pub_bev.publish(image(self.sim.bev_image(), stamp, "camera_bev"))
        state = self.sim.state()
        self.pub_imu.publish(imu_message(float(state["yaw"]), float(state["w"]), stamp))

    def publish_state(self, stamp) -> None:
        s = self.sim.state()
        payload = {
            "scene": s["scene"],
            "time": round(float(s["time"]), 3),
            "truth": [round(float(s["x"]), 4), round(float(s["y"]), 4), round(float(s["yaw"]), 4)],
            "odom": [round(float(v), 4) for v in s["odom"]],
            "v": round(float(s["v"]), 4),
            "w": round(float(s["w"]), 4),
            "cmd": [round(float(v), 4) for v in s["cmd"]],
            "collisions": int(s["collisions"]),
            "goal_distance": round(float(s["goal_distance"]), 4),
        }
        self.pub_state.publish(String(data=json.dumps(payload)))

    def publish_tf(self, stamp) -> None:
        odom = self.sim.odom
        transforms = [
            transform("odom", "base_link", odom[0], odom[1], odom[2], stamp),
            transform("base_link", "laser", 0.0, 0.0, 0.0, stamp, z=0.12),
            transform("base_link", "camera_rgb", 0.15, 0.0, 0.0, stamp, z=0.24),
            transform("base_link", "camera_depth", 0.15, 0.0, 0.0, stamp, z=0.24),
            transform("base_link", "camera_bev", 0.0, 0.0, 0.0, stamp, z=0.40),
        ]
        self.pub_tf.publish(tf_message(transforms))

    # ------------------------------------------------------------------ loop
    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        period = 1.0 / self.sensor_hz
        rospy.loginfo("GVNN sim ready: scene=%s seed=%d", self.scene_name, self.seed)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.cmd_stamp).to_sec() > 0.6:
                self.cmd = [0.0, 0.0]
            state = self.sim.step(self.cmd[0], self.cmd[1], 1.0 / self.rate_hz)
            stamp = rospy.Time.now()
            self.pub_odom.publish(odometry(self.sim.odom, float(state["v"]),
                                           float(state["w"]), stamp))
            self.pub_truth.publish(pose_stamped(self.sim.truth, "map", stamp))
            self.publish_tf(stamp)
            if (now - self.last_sensor).to_sec() >= period:
                self.last_sensor = now
                self.publish_sensors(stamp)
            if (now - self.last_status).to_sec() >= 0.2:
                self.last_status = now
                self.publish_state(stamp)
            rate.sleep()

    def shutdown(self) -> None:
        rospy.loginfo("GVNN sim shutting down")


if __name__ == "__main__":
    rospy.init_node("gvnn_sim")
    SimNode().spin()
