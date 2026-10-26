#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU 桥接本地测试（桩依赖，不需要 ROS / 仿真器）.

A. 机体系 NED -> ROS FLU 的轴变换
B. SimClient.get_imu_enu 的四元数/角速度/线加速度
C. imu_pub.ImuPublisher.publish 的消息字段与协方差
D. 工程文件一致性：CMakeLists / main.launch / bridge.yaml / README

用法: python3 tests/test_imu_local.py
"""
from __future__ import annotations

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


class Vector3(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Imu(object):
    def __init__(self):
        self.header = Header()
        self.orientation = Vector3()
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9


class _Publisher(object):
    def __init__(self, name, msg_type, queue_size=10):
        self.name = name
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda name, default=None: default
    rospy.Publisher = _Publisher
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.Time = types.SimpleNamespace(now=lambda: 123.0)
    rospy.init_node = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.loginfo = lambda *a, **k: None
    rospy.logwarn = lambda *a, **k: None
    sys.modules["rospy"] = rospy

    sm = types.ModuleType("sensor_msgs")
    smm = types.ModuleType("sensor_msgs.msg")
    smm.Imu = Imu
    sm.msg = smm
    sys.modules["sensor_msgs"] = sm
    sys.modules["sensor_msgs.msg"] = smm


install_stubs()
import imu_pub  # noqa: E402
import sim_client  # noqa: E402


def test_body_axes():
    print("== A. 机体系轴变换 ==")
    v = sim_client.body_ned_to_flu([1.0, 2.0, 3.0])
    check(np.allclose(v, [1.0, -2.0, -3.0]), "NED(前右下) -> FLU(前左上): %r" % (v,))
    check(np.allclose(sim_client.body_ned_to_flu([0, 0, 0]), [0, 0, 0]),
          "零向量保持为零")


def _fake_imu_data():
    q = types.SimpleNamespace(w_val=1.0, x_val=0.0, y_val=0.0, z_val=0.0)
    av = types.SimpleNamespace(x_val=1.0, y_val=2.0, z_val=3.0)
    la = types.SimpleNamespace(x_val=4.0, y_val=5.0, z_val=6.0)
    return types.SimpleNamespace(orientation=q, angular_velocity=av,
                                 linear_acceleration=la)


def test_get_imu_enu():
    print("== B. get_imu_enu ==")
    sim = sim_client.SimClient()
    sim.client = types.SimpleNamespace(getImuData=lambda vehicle_name="": _fake_imu_data())
    sim.vehicle_name = ""
    quat, ang, acc = sim.get_imu_enu()
    check(abs(np.linalg.norm(quat) - 1.0) < 1e-9, "四元数归一化: norm=%.6f" % np.linalg.norm(quat))
    check(np.allclose(ang, [1.0, -2.0, -3.0]), "角速度 NED->FLU: %r" % (ang,))
    check(np.allclose(acc, [4.0, -5.0, -6.0]), "线加速度 NED->FLU: %r" % (acc,))
    expected = (np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0)
    check(np.allclose(quat, expected, atol=1e-9),
          "NED 恒等姿态 -> ENU 180°(x,y) 四元数: %r" % (quat,))


def test_publish():
    print("== C. imu_pub.publish ==")
    node = object.__new__(imu_pub.ImuPublisher)
    node.frame_id = "base_link"
    node.pub = _Publisher("/imu", Imu)
    node.publish((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0), (4.0, 5.0, 6.0))
    msg = node.pub.msgs[-1]
    check(msg.header.frame_id == "base_link", "header.frame_id=base_link")
    check(msg.header.stamp == 123.0, "header.stamp 已填")
    check(msg.orientation.w == 1.0, "orientation.w 正确")
    check(msg.angular_velocity.x == 1.0 and msg.angular_velocity.z == 3.0,
          "angular_velocity 三轴正确")
    check(msg.linear_acceleration.z == 6.0, "linear_acceleration.z 正确")
    check(msg.orientation_covariance[0] == 0.01, "orientation 协方差已填")
    check(msg.linear_acceleration_covariance[8] == 0.1, "加速度协方差已填")


def test_project_files():
    print("== D. 工程文件一致性 ==")
    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    launch = open(os.path.join(PKG, "launch", "main.launch"), encoding="utf-8").read()
    yaml = open(os.path.join(PKG, "config", "bridge.yaml"), encoding="utf-8").read()
    readme = open(os.path.join(PKG, "README.md"), encoding="utf-8").read()
    check("scripts/imu_pub.py" in cmake, "CMakeLists 安装 imu_pub.py")
    check('type="imu_pub.py"' in launch, "main.launch 启动 imu_pub.py")
    check('name="publish_imu"' in launch, "main.launch 有 publish_imu 开关")
    check("imu_hz: 50.0" in yaml, "bridge.yaml 配置 imu_hz")
    check('imu: "/imu"' in yaml, "bridge.yaml 配置 /imu 话题")
    check("`/imu`" in readme, "README 记录 /imu 接口")


if __name__ == "__main__":
    test_body_axes()
    test_get_imu_enu()
    test_publish()
    test_project_files()
    print("\n通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    sys.exit(1 if FAIL else 0)
