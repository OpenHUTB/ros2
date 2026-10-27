#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""里程计协方差本地测试（不需要 ROS / 仿真器）."""
from __future__ import annotations

import os
import sys
import types

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
sys.path.insert(0, os.path.join(PKG, "scripts"))


class Header(object):
    def __init__(self):
        self.stamp = None
        self.frame_id = ""
        self.seq = 0


class Pose(object):
    def __init__(self):
        self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class Twist(object):
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class WithCovariance(object):
    def __init__(self, inner):
        self.pose = inner if isinstance(inner, Pose) else None
        self.twist = inner if isinstance(inner, Twist) else None
        self.covariance = [0.0] * 36


class Odometry(object):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.pose = WithCovariance(Pose())
        self.twist = WithCovariance(Twist())


class TransformStamped(object):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class Pub(object):
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class TF(object):
    def __init__(self):
        self.msgs = []

    def sendTransform(self, msg):
        self.msgs.append(msg)


rospy = types.ModuleType("rospy")
rospy.Time = types.SimpleNamespace(now=lambda: 100.0)
sys.modules["rospy"] = rospy
nav = types.ModuleType("nav_msgs")
navm = types.ModuleType("nav_msgs.msg")
navm.Odometry = Odometry
nav.msg = navm
sys.modules["nav_msgs"] = nav
sys.modules["nav_msgs.msg"] = navm
gm = types.ModuleType("geometry_msgs")
gmm = types.ModuleType("geometry_msgs.msg")
gmm.TransformStamped = TransformStamped
gm.msg = gmm
sys.modules["geometry_msgs"] = gm
sys.modules["geometry_msgs.msg"] = gmm
tf = types.ModuleType("tf2_ros")
tf.TransformBroadcaster = TF
sys.modules["tf2_ros"] = tf

import odom_pub  # noqa: E402


def test_covariance():
    print("== 里程计协方差 ==")
    node = object.__new__(odom_pub.OdomPublisher)
    node.world_frame = "world"
    node.body_frame = "base_link"
    node.pub = Pub()
    node.tf_broadcaster = TF()
    node.publish([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0],
                 [0.1, 0.2, 0.3], [0.01, 0.02, 0.03])
    msg = node.pub.msgs[-1]
    for i in (0, 7, 14):
        check(msg.pose.covariance[i] == 0.01, "pose 线性协方差[%d]=0.01" % i)
        check(msg.twist.covariance[i] == 0.01, "twist 线性协方差[%d]=0.01" % i)
    for i in (21, 28, 35):
        check(msg.pose.covariance[i] == 0.1, "pose 姿态协方差[%d]=0.1" % i)
        check(msg.twist.covariance[i] == 0.1, "twist 角速度协方差[%d]=0.1" % i)
    check(msg.header.frame_id == "world" and msg.child_frame_id == "base_link",
          "frame_id 正确")
    check(len(node.tf_broadcaster.msgs) == 1, "仍发布 1 个 TF")


if __name__ == "__main__":
    test_covariance()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
