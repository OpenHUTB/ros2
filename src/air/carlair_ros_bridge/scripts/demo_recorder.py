#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同步采集 RGB / 深度 / 里程计，保存为离线数据集.

用法：
    rosrun carlair_ros_bridge demo_recorder.py --output-dir /tmp/demo_data
    rosrun carlair_ros_bridge demo_recorder.py --max-frames 50 --rate 3

输出：
    <output_dir>/rgb/000001.png
    <output_dir>/depth/000001.npy
    <output_dir>/meta.csv
"""
from __future__ import annotations

import argparse
import csv
import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


def frame_name(index):
    return "%06d" % int(index)


def depth_to_meters(array, encoding):
    """深度图转 float32 米；无效值置 0."""
    depth = np.asarray(array)
    if encoding == "16UC1":
        depth = depth.astype(np.float32) / 1000.0
    else:
        depth = depth.astype(np.float32)
    depth[~np.isfinite(depth)] = 0.0
    return depth


def is_fresh(now, stamps, max_age):
    """所有时间戳都在 max_age 内才认为同步."""
    if not stamps:
        return False
    return all(abs(float(now) - float(s)) <= float(max_age) for s in stamps)


def make_meta(index, rgb_stamp, depth_stamp, odom_stamp, pose, max_age):
    row = {
        "frame": frame_name(index),
        "rgb_stamp": "%.6f" % rgb_stamp,
        "depth_stamp": "%.6f" % depth_stamp,
        "odom_stamp": "%.6f" % odom_stamp,
        "max_age": "%.3f" % max_age,
    }
    for key in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        row[key] = "%.6f" % pose[key]
    return row


class DemoRecorder(object):
    def __init__(self, output_dir, max_frames=100, rate=3.0, max_age=0.3):
        self.output_dir = os.path.abspath(output_dir)
        self.max_frames = int(max_frames)
        self.rate = float(rate)
        self.max_age = float(max_age)
        self.bridge = CvBridge()
        self.image = None
        self.depth = None
        self.odom = None
        self.image_time = None
        self.depth_time = None
        self.odom_time = None
        self.index = 0

        os.makedirs(os.path.join(self.output_dir, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "depth"), exist_ok=True)
        self.meta_path = os.path.join(self.output_dir, "meta.csv")
        self.meta_file = open(self.meta_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.meta_file, fieldnames=[
            "frame", "rgb_stamp", "depth_stamp", "odom_stamp", "max_age",
            "x", "y", "z", "qx", "qy", "qz", "qw",
        ])
        self.writer.writeheader()

        rospy.Subscriber("/camera/image_raw", Image, self._on_image, queue_size=2)
        rospy.Subscriber("/camera/depth", Image, self._on_depth, queue_size=2)
        rospy.Subscriber("/uav/odom", Odometry, self._on_odom, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.rate, 0.1)), self._on_timer)
        rospy.loginfo("demo_recorder: 输出到 %s", self.output_dir)

    def _on_image(self, msg):
        self.image = msg
        self.image_time = msg.header.stamp.to_sec()

    def _on_depth(self, msg):
        self.depth = msg
        self.depth_time = msg.header.stamp.to_sec()

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_time = msg.header.stamp.to_sec()

    def _on_timer(self, _event):
        if self.index >= self.max_frames:
            return
        if self.image is None or self.depth is None or self.odom is None:
            return
        now = rospy.Time.now().to_sec()
        if not is_fresh(now, (self.image_time, self.depth_time, self.odom_time),
                       self.max_age):
            return

        rgb = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
        depth_m = depth_to_meters(depth, self.depth.encoding)

        self.index += 1
        name = frame_name(self.index)
        cv2.imwrite(os.path.join(self.output_dir, "rgb", name + ".png"), rgb)
        np.save(os.path.join(self.output_dir, "depth", name + ".npy"), depth_m)

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        pose = {"x": p.x, "y": p.y, "z": p.z,
                "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
        self.writer.writerow(make_meta(
            self.index, self.image_time, self.depth_time, self.odom_time,
            pose, self.max_age))
        self.meta_file.flush()
        rospy.loginfo("demo_recorder: 已保存 %s", name)
        if self.index >= self.max_frames:
            rospy.loginfo("demo_recorder: 达到 max_frames=%d，停止", self.max_frames)
            rospy.signal_shutdown("recording finished")

    def close(self):
        try:
            self.meta_file.close()
        except Exception:  # noqa: BLE001
            pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="同步 RGB-D/odom 采集")
    parser.add_argument("--output-dir", default="/tmp/carlair_demo_data")
    parser.add_argument("--max-frames", type=int, default=100)
    parser.add_argument("--rate", type=float, default=3.0)
    parser.add_argument("--max-age", type=float, default=0.3)
    return parser.parse_known_args(argv)[0]


def main(argv=None):
    args = parse_args(argv)
    rospy.init_node("demo_recorder", anonymous=False)
    recorder = DemoRecorder(args.output_dir, args.max_frames, args.rate, args.max_age)
    try:
        rospy.spin()
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
