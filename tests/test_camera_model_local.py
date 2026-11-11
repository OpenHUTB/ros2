#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机模型本地测试（桩依赖，不需要 ROS / 仿真器）.

A. CameraInfo 的针孔内参
B. 深度图 -> 相机光学系点云
C. PointCloud2 的字段布局
D. 节点发布/静态 TF
E. 工程文件一致性

用法: python3 tests/test_camera_model_local.py
"""
from __future__ import annotations

import math
import os
import sys
import types

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
SCRIPTS = os.path.join(PKG, "scripts")
sys.path.insert(0, SCRIPTS)


# ---------------------------------------------------------------- 消息桩
class Header(object):
    def __init__(self):
        self.stamp = None
        self.frame_id = ""
        self.seq = 0


class CameraInfo(object):
    def __init__(self):
        self.header = Header()
        self.width = 0
        self.height = 0
        self.distortion_model = ""
        self.D = []
        self.K = []
        self.R = []
        self.P = []


class Image(object):
    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = False
        self.step = 0
        self.data = b""


class PointField(object):
    FLOAT32 = 7

    def __init__(self, name="", offset=0, datatype=0, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class PointCloud2(object):
    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.fields = []
        self.is_bigendian = False
        self.point_step = 0
        self.row_step = 0
        self.data = b""
        self.is_dense = False


class Vector3(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Quaternion(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Transform(object):
    def __init__(self):
        self.translation = Vector3()
        self.rotation = Quaternion()


class TransformStamped(object):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.transform = Transform()


class _Publisher(object):
    def __init__(self, name, msg_type, queue_size=10):
        self.name = name
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Subscriber(object):
    def __init__(self, topic, msg_type, cb, queue_size=1):
        self.topic = topic
        self.cb = cb


class _StaticTF(object):
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transforms):
        self.transforms.extend(transforms)


PUBS = {}
PARAMS = {
    "frame/body": "base_link",
    "camera/rgb_frame": "front_rgb",
    "camera/depth_frame": "front_depth",
    "camera/fov_degrees": 90.0,
    "camera/mount_x": 0.3,
    "camera/mount_y": 0.0,
    "camera/mount_z": 0.1,
    "topic/image": "/camera/image_raw",
    "topic/depth": "/camera/depth",
    "topic/camera_info": "/camera/camera_info",
    "topic/depth_info": "/camera/depth/camera_info",
    "topic_depth_points": "/camera/depth/points",
    "topic/depth_points": "/camera/depth/points",
}


def install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda name, default=None: PARAMS.get(name, default)
    rospy.Publisher = lambda name, msg_type, queue_size=10: PUBS.setdefault(
        name, _Publisher(name, msg_type, queue_size))
    rospy.Subscriber = _Subscriber
    rospy.Time = types.SimpleNamespace(now=lambda: 100.0)
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda: None
    rospy.loginfo = lambda *a, **k: None
    sys.modules["rospy"] = rospy

    sm = types.ModuleType("sensor_msgs")
    smm = types.ModuleType("sensor_msgs.msg")
    smm.CameraInfo, smm.Image = CameraInfo, Image
    smm.PointCloud2, smm.PointField = PointCloud2, PointField
    sm.msg = smm
    sys.modules["sensor_msgs"] = sm
    sys.modules["sensor_msgs.msg"] = smm

    gm = types.ModuleType("geometry_msgs")
    gmm = types.ModuleType("geometry_msgs.msg")
    gmm.TransformStamped = TransformStamped
    gm.msg = gmm
    sys.modules["geometry_msgs"] = gm
    sys.modules["geometry_msgs.msg"] = gmm

    tf = types.ModuleType("tf2_ros")
    tf.StaticTransformBroadcaster = _StaticTF
    sys.modules["tf2_ros"] = tf


install_stubs()
import camera_model_pub  # noqa: E402


def test_camera_info():
    print("== A. CameraInfo ==")
    msg = camera_model_pub.make_camera_info(640, 480, 90.0, "front_rgb", 100.0, 3)
    fx = 640 / (2 * math.tan(math.radians(90) / 2))
    check(abs(msg.K[0] - fx) < 1e-6, "K[0]=fx=%.3f" % fx)
    check(abs(msg.K[2] - 320) < 1e-9 and abs(msg.K[5] - 240) < 1e-9, "主点 (320, 240)")
    check(len(msg.K) == 9 and len(msg.P) == 12 and len(msg.R) == 9, "K/P/R 长度正确")
    check(msg.header.frame_id == "front_rgb" and msg.header.seq == 3, "header 正确")
    check(msg.distortion_model == "plumb_bob", "distortion_model=plumb_bob")


def test_depth_points():
    print("== B. 深度反投影 ==")
    depth = np.array([[1.0, 2.0], [3.0, 65504.0]], dtype=np.float32)
    pts = camera_model_pub.depth_to_points(depth, 90.0)
    check(pts.shape == (3, 3), "过滤 65504 后剩 3 点: %r" % (pts.shape,))
    check(np.allclose(pts[0], [-1.0, -1.0, 1.0]), "像素(0,0) -> (-1,-1,1): %r" % (pts[0],))
    check(np.allclose(pts[1], [0.0, -2.0, 2.0]), "像素(1,0) -> (0,-1,2): %r" % (pts[1],))
    check(np.allclose(pts[2], [-3.0, 0.0, 3.0]), "像素(0,1) -> (-1,0,3): %r" % (pts[2],))


def test_pointcloud():
    print("== C. PointCloud2 ==")
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    msg = camera_model_pub.make_pointcloud2(pts, "front_depth", 100.0, 2)
    check(msg.width == 2 and msg.height == 1, "width/height 正确")
    check(msg.point_step == 12 and msg.row_step == 24, "point_step=12, row_step=24")
    check([f.offset for f in msg.fields] == [0, 4, 8], "字段偏移 0/4/8")
    check(len(msg.data) == 24, "data 长度 24")
    back = np.frombuffer(msg.data, dtype="<f4").reshape(-1, 3)
    check(np.allclose(back, pts), "字节流可还原坐标")


def test_node():
    print("== D. 节点 ==")
    node = camera_model_pub.CameraModelPublisher()
    check(len(node.tf_broadcaster.transforms) == 2, "广播 2 个静态 TF")
    frames = {t.child_frame_id for t in node.tf_broadcaster.transforms}
    check(frames == {"front_rgb", "front_depth"}, "TF 子帧正确: %r" % (frames,))

    rgb = Image()
    rgb.width, rgb.height = 640, 480
    node.rgb_cb(rgb)
    node.depth_cb(types.SimpleNamespace(width=2, height=2, data=depth_bytes()))
    check(len(PUBS["/camera/camera_info"].msgs) == 1, "发布 /camera/camera_info")
    check(len(PUBS["/camera/depth/camera_info"].msgs) == 1, "发布 /camera/depth/camera_info")
    check(len(PUBS["/camera/depth/points"].msgs) == 1, "发布 /camera/depth/points")
    check(PUBS["/camera/depth/points"].msgs[0].width == 3, "点云 3 点")


def depth_bytes():
    return np.array([[1.0, 2.0], [3.0, 65504.0]], dtype=np.float32).tobytes()


def test_project_files():
    print("== E. 工程文件 ==")
    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    launch = open(os.path.join(PKG, "launch", "main.launch"), encoding="utf-8").read()
    cfg = open(os.path.join(PKG, "config", "bridge.yaml"), encoding="utf-8").read()
    readme = open(os.path.join(PKG, "README.md"), encoding="utf-8").read()
    check("camera_model_pub.py" in cmake, "CMakeLists 安装 camera_model_pub.py")
    check('type="camera_model_pub.py"' in launch, "main.launch 启动 camera_model_pub.py")
    for key in ("fov_degrees", "rgb_frame", "depth_frame", "camera_info", "depth_points"):
        check(key in cfg, "bridge.yaml 含 %s" % key)
    check("/camera/depth/points" in readme, "README 记录 /camera/depth/points")


if __name__ == "__main__":
    test_camera_info()
    test_depth_points()
    test_pointcloud()
    test_node()
    test_project_files()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
