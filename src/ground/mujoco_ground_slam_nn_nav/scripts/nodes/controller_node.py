#!/usr/bin/env python3
"""Path-following controller node: pure pursuit or the neural controller."""

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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String

from gvnn.config import MAX_OMEGA, MAX_SPEED
from gvnn.control.pure_pursuit import PurePursuit
from gvnn.control.safety import front_clearance, safety_filter
from gvnn.nn.mlp import MLP


class ControllerNode:
    def __init__(self) -> None:
        self.use_nn = bool(rospy.get_param("~use_nn", False))
        self.model_path = rospy.get_param(
            "~control_model", os.path.join(ROOT, "models", "control_mlp.npz"))
        self.model = None
        if self.use_nn and os.path.exists(self.model_path):
            try:
                self.model = MLP.load(self.model_path)
                rospy.loginfo("neural controller loaded: %s", self.model_path)
            except Exception as exc:
                rospy.logwarn("neural controller unavailable: %s", exc)
        self.use_nn = self.model is not None
        self.pp = PurePursuit(max_speed=0.90)
        self.path = []
        self.pose = np.array([2.0, 2.0, 0.0])
        self.speed = 0.0
        self.scan = np.full(360, 9.0)
        self.enabled = True
        self.last_status = rospy.Time(0)
        self.last_cmd = Twist()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.pub_status = rospy.Publisher("/gvnn/controller_status", String, queue_size=5)
        rospy.Subscriber("/plan", Path, self.path_cb, queue_size=2)
        rospy.Subscriber("/slam_pose", PoseStamped, self.pose_cb, queue_size=5)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=2)
        rospy.Subscriber("/gvnn/enable", Bool, self.enable_cb, queue_size=2)
        rospy.Subscriber("/gvnn/reset", Empty, self.reset_cb, queue_size=2)
        rospy.Subscriber("/gvnn/scene", String, self.reset_cb, queue_size=2)

    def path_cb(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def pose_cb(self, msg) -> None:
        q = msg.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, yaw])

    def odom_cb(self, msg: Odometry) -> None:
        self.speed = float(msg.twist.twist.linear.x)

    def scan_cb(self, msg: LaserScan) -> None:
        self.scan = np.asarray(msg.ranges, dtype=np.float64)
        self.scan = np.nan_to_num(self.scan, nan=msg.range_max, posinf=msg.range_max,
                                  neginf=msg.range_min)

    def enable_cb(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.pub.publish(Twist())

    def reset_cb(self, _msg) -> None:
        self.path = []
        self.pub.publish(Twist())

    def compute(self) -> Twist:
        out = Twist()
        if not self.enabled or len(self.path) < 2:
            return out
        fc = front_clearance(self.scan)
        cmd = self.pp.control(self.pose, self.path, fc)
        v, w = cmd["v"], cmd["w"]
        if self.use_nn:
            features = np.array([[self.speed / 1.1, cmd["cross_track"],
                                  cmd["heading_error"] / np.pi,
                                  cmd["curvature"] / 2.0, fc / 9.0]])
            pred = self.model.predict(features)[0]
            v_nn = float(max(0.0, (pred[0] + 1.0) * 0.5 * 1.1))
            w_nn = float(pred[1] * 1.6)
            # Pure-pursuit supervisor for large tracking errors.
            if abs(cmd["cross_track"]) < 0.55 and abs(cmd["heading_error"]) < 0.65:
                v, w = v_nn, w_nn
            if cmd["distance"] > 1.5:
                v = max(v, 0.25)
        v, w, info = safety_filter(self.scan, v, w)
        out.linear.x = float(np.clip(v, 0.0, MAX_SPEED))
        out.angular.z = float(np.clip(w, -MAX_OMEGA, MAX_OMEGA))
        return out

    def spin(self) -> None:
        rate = rospy.Rate(20.0)
        rospy.loginfo("GVNN controller ready: neural=%s", self.use_nn)
        while not rospy.is_shutdown():
            cmd = self.compute()
            self.pub.publish(cmd)
            now = rospy.Time.now()
            if (now - self.last_status).to_sec() > 0.5:
                self.last_status = now
                payload = {"mode": "neural" if self.use_nn else "pure_pursuit",
                           "v": round(cmd.linear.x, 3), "w": round(cmd.angular.z, 3),
                           "enabled": self.enabled, "waypoints": len(self.path)}
                self.pub_status.publish(String(data=json.dumps(payload)))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gvnn_controller")
    ControllerNode().spin()
