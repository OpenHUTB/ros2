#!/usr/bin/env python3
"""Lidar perception node: neural denoising and obstacle confidence."""

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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from gvnn.config import LIDAR_RANGE_MAX
from gvnn.nn.cnn import CNN
from gvnn.nn.mlp import MLP
from gvnn.ros_utils import laser_scan


class PerceptionNode:
    def __init__(self) -> None:
        self.model_path = rospy.get_param(
            "~model", os.path.join(ROOT, "models", "perception_cnn.npz"))
        if not os.path.exists(self.model_path):
            fallback = os.path.join(ROOT, "models", "perception_mlp.npz")
            if os.path.exists(fallback):
                self.model_path = fallback
        self.model = None
        self.is_cnn = False
        if os.path.exists(self.model_path):
            try:
                if self.model_path.endswith("perception_cnn.npz"):
                    self.model = CNN.load(self.model_path)
                    self.is_cnn = True
                else:
                    self.model = MLP.load(self.model_path)
                rospy.loginfo("perception model loaded: %s", self.model_path)
            except Exception as exc:
                rospy.logwarn("perception model unavailable: %s", exc)
        self.enabled = True
        self.last_status = rospy.Time(0)
        self.pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=3)
        self.pub_status = rospy.Publisher("/gvnn/perception_status", String, queue_size=5)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=3)
        rospy.Subscriber("/gvnn/enable", Bool, self.enable_cb, queue_size=2)

    def enable_cb(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)

    def scan_cb(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max,
                               neginf=msg.range_min)
        if not self.enabled or self.model is None:
            self.pub.publish(laser_scan(ranges, rospy.Time.now(), msg.header.frame_id,
                                        msg.angle_min, msg.angle_increment,
                                        msg.range_min, msg.range_max))
            return
        inp = np.clip(ranges / LIDAR_RANGE_MAX, 0.0, 1.0)
        if self.is_cnn:
            residual = self.model.predict(inp.reshape(1, 1, -1, 1)).reshape(-1)
        else:
            residual = self.model.predict(inp.reshape(1, -1))[0]
        pred = np.clip(inp + residual, 0.0, 1.0) * LIDAR_RANGE_MAX
        pred = np.clip(pred, msg.range_min, msg.range_max)
        filtered = laser_scan(pred, rospy.Time.now(), msg.header.frame_id,
                              msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max)
        self.pub.publish(filtered)
        now = rospy.Time.now()
        if (now - self.last_status).to_sec() > 0.5:
            self.last_status = now
            self.pub_status.publish(String(data=json.dumps({
                "model": os.path.basename(self.model_path),
                "obstacles": int(np.sum(pred < 1.0)),
                "enabled": self.enabled,
            })))


if __name__ == "__main__":
    rospy.init_node("gvnn_perception")
    node = PerceptionNode()
    rospy.loginfo("GVNN perception ready")
    rospy.spin()
