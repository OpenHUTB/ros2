#!/usr/bin/env python3
"""Online occupancy-grid SLAM node (incremental odometry + scan matching)."""

from __future__ import annotations

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from gvnn.runtime import ensure_venv

ensure_venv(__file__)

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage

from gvnn.config import MAP_RESOLUTION
from gvnn.ros_utils import occupancy_grid, pose_stamped, tf_message, transform
from gvnn.slam.frontend import SlamFrontend


class SlamNode:
    def __init__(self) -> None:
        self.width = float(rospy.get_param("~width", 24.0))
        self.height = float(rospy.get_param("~height", 16.0))
        self.start = [float(rospy.get_param("~start_x", 2.0)),
                      float(rospy.get_param("~start_y", 2.0)),
                      float(rospy.get_param("~start_yaw", 0.0))]
        self.fe = SlamFrontend(self.width, self.height, initial_pose=self.start)
        self.odom = np.asarray(self.start, dtype=np.float64)
        self.pose = np.asarray(self.start, dtype=np.float64)
        self.latest_scan = None
        self.latest_score = 0.0
        self.last_map = rospy.Time(0)
        self.last_status = rospy.Time(0)

        self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=2, latch=True)
        self.pub_pose = rospy.Publisher("/slam_pose", PoseStamped, queue_size=5)
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=10)
        self.pub_status = rospy.Publisher("/gvnn/slam_status", String, queue_size=5)

        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=3)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/gvnn/reset", Empty, self.reset_cb, queue_size=2)
        rospy.Subscriber("/gvnn/scene", String, self.scene_cb, queue_size=2)
        rospy.on_shutdown(self.shutdown)

    def odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = np.array([msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw], dtype=np.float64)

    def scan_cb(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max,
                               neginf=msg.range_min)
        self.latest_scan = ranges
        self.pose, self.latest_score = self.fe.update(self.odom, ranges)
        stamp = rospy.Time.now()
        self.pub_pose.publish(pose_stamped(self.pose, "map", stamp))
        map_odom = self.fe.map_to_odom(self.odom)
        self.pub_tf.publish(tf_message([
            transform("map", "odom", map_odom[0], map_odom[1], map_odom[2], stamp)
        ]))

    def reset_cb(self, _msg: Empty) -> None:
        self.fe.reset(self.start)
        self.pose = np.asarray(self.start, dtype=np.float64)
        rospy.loginfo("SLAM reset")

    def scene_cb(self, _msg: String) -> None:
        self.reset_cb(Empty())

    def publish_map(self, stamp) -> None:
        data = self.fe.grid.data()
        self.pub_map.publish(occupancy_grid(data, MAP_RESOLUTION, stamp))
        self.last_map = stamp

    def publish_status(self, stamp) -> None:
        payload = {
            "pose": [round(float(v), 4) for v in self.pose],
            "score": round(float(self.latest_score), 4),
            "coverage": round(float(self.fe.coverage), 4),
            "updates": int(self.fe.grid.updates),
            "matches": int(self.fe.matches),
        }
        self.pub_status.publish(String(data=json.dumps(payload)))
        self.last_status = stamp

    def spin(self) -> None:
        rate = rospy.Rate(10.0)
        rospy.loginfo("GVNN SLAM ready: %.1f x %.1f m @ %.2f m", self.width,
                      self.height, MAP_RESOLUTION)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.last_map).to_sec() > 0.5:
                self.publish_map(now)
            if (now - self.last_status).to_sec() > 0.3:
                self.publish_status(now)
            rate.sleep()

    def shutdown(self) -> None:
        rospy.loginfo("GVNN SLAM shutting down")


if __name__ == "__main__":
    rospy.init_node("gvnn_slam")
    SlamNode().spin()
