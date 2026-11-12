#!/usr/bin/env python3
"""End-to-end CNN policy node: bird's-eye RGB image -> (v, omega)."""

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

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

from gvnn.config import MAX_OMEGA, MAX_SPEED
from gvnn.control.safety import safety_filter
from gvnn.nn.cnn import CNN


class E2ENode:
    def __init__(self) -> None:
        self.model_path = rospy.get_param(
            "~model", os.path.join(ROOT, "models", "e2e_cnn.npz"))
        self.model = None
        if os.path.exists(self.model_path):
            try:
                self.model = CNN.load(self.model_path)
                rospy.loginfo("E2E CNN loaded: %s", self.model_path)
            except Exception as exc:
                rospy.logwarn("E2E model unavailable: %s", exc)
        self.enabled = True
        self.image = None
        self.scan = np.full(360, 9.0)
        self.odom = np.zeros(3)
        self.goal = np.array([float(rospy.get_param("~goal_x", 22.0)),
                              float(rospy.get_param("~goal_y", 14.0))])
        self.last_status = rospy.Time(0)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.pub_status = rospy.Publisher("/gvnn/e2e_status", String, queue_size=5)
        rospy.Subscriber("/camera/bev/image_raw", Image, self.image_cb, queue_size=2)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=2)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/gvnn/enable", Bool, self.enable_cb, queue_size=2)

    def odom_cb(self, msg) -> None:
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def enable_cb(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.pub.publish(Twist())

    def scan_cb(self, msg: LaserScan) -> None:
        self.scan = np.asarray(msg.ranges, dtype=np.float64)
        self.scan = np.nan_to_num(self.scan, nan=msg.range_max, posinf=msg.range_max,
                                  neginf=msg.range_min)

    def image_cb(self, msg: Image) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "bgr8":
            img = arr.reshape(msg.height, msg.width, 3)[:, :, ::-1]
        elif msg.encoding == "rgb8":
            img = arr.reshape(msg.height, msg.width, 3)
        elif msg.encoding == "mono8":
            img = np.repeat(arr.reshape(msg.height, msg.width, 1), 3, axis=2)
        else:
            return
        self.image = img

    def control(self) -> Twist:
        out = Twist()
        if not self.enabled or self.model is None or self.image is None:
            return out
        inp = self.image.astype(np.float64)[None, ...] / 255.0
        pred = self.model.predict(inp)[0]
        v = float(max(0.0, (pred[0] + 1.0) * 0.5 * 1.1))
        w = float(pred[1] * 1.6)
        bearing = math.atan2(self.goal[1] - self.odom[1],
                             self.goal[0] - self.odom[0]) - self.odom[2]
        bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi
        v = min(v, 0.55)
        w = 0.70 * w + 0.50 * max(-1.2, min(1.2, 1.2 * bearing))
        if abs(bearing) > 0.9:
            v = min(v, 0.25)
        dist_goal = float(np.linalg.norm(self.goal - self.odom[:2]))
        if dist_goal < 0.9:
            v = 0.0
        elif dist_goal < 1.8:
            v = min(v, 0.35)
        else:
            v = max(v, 0.28)
        v, w, _ = safety_filter(self.scan, v, w)
        out.linear.x = float(np.clip(v, 0.0, MAX_SPEED))
        out.angular.z = float(np.clip(w, -MAX_OMEGA, MAX_OMEGA))
        return out

    def spin(self) -> None:
        rate = rospy.Rate(10.0)
        rospy.loginfo("GVNN E2E ready: model=%s", os.path.basename(self.model_path))
        while not rospy.is_shutdown():
            cmd = self.control()
            self.pub.publish(cmd)
            now = rospy.Time.now()
            if (now - self.last_status).to_sec() > 0.5:
                self.last_status = now
                self.pub_status.publish(String(data=json.dumps({
                    "v": round(cmd.linear.x, 3),
                    "w": round(cmd.angular.z, 3),
                    "enabled": self.enabled,
                    "model": os.path.basename(self.model_path),
                })))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gvnn_e2e")
    E2ENode().spin()
