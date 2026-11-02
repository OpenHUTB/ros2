#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""桥接模块本地测试（桩依赖，不需要 ROS / 仿真器 / GPU）.

A. cmd_vel_sub 状态机：无指令悬停 → 速度指令限幅下发 → 超时自动悬停 → 目标点让位
B. odom_pub 字段映射：/uav/odom 与 TF world->base_link
C. 工程文件合法性：bridge.yaml / settings.json / main.launch / package.xml / CMakeLists.txt

用法: python3 tests/test_bridge_local.py
"""
from __future__ import annotations

import json
import os
import sys
import time
import types
import xml.etree.ElementTree as ET

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


# ---------------------------------------------------------------- 路径
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
SCRIPTS = os.path.join(PKG, "scripts")
sys.path.insert(0, SCRIPTS)


# ---------------------------------------------------------------- 消息桩
class Vec3(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Quat(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Point(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Header(object):
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class Twist(object):
    def __init__(self):
        self.linear = Vec3()
        self.angular = Vec3()


class String(object):
    def __init__(self, data=""):
        self.data = data


class Pose(object):
    def __init__(self):
        self.position = Point()
        self.orientation = Quat()


class PoseWithCov(object):
    def __init__(self):
        self.pose = Pose()
        self.covariance = [0.0] * 36


class TwistWithCov(object):
    def __init__(self):
        self.twist = Twist()
        self.covariance = [0.0] * 36


class Odometry(object):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.pose = PoseWithCov()
        self.twist = TwistWithCov()


class Transform(object):
    def __init__(self):
        self.translation = Vec3()
        self.rotation = Quat()


class TransformStamped(object):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.transform = Transform()


# ---------------------------------------------------------------- rospy 桩
PARAMS = {
    "sim/host": "127.0.0.1", "sim/airsim_port": 41451, "sim/vehicle_name": "Drone1",
    "topic/cmd_vel": "/uav/cmd_vel", "topic/goal": "/uav/goal",
    "topic/status": "/uav/status", "topic/odom": "/uav/odom",
    "rate/odom_hz": 20.0, "rate/cmd_timeout": 0.5,
    "control/max_horiz_speed": 5.0, "control/max_vert_speed": 3.0,
    "control/max_yaw_rate": 1.0, "control/body_frame": True,
    "control/goal_speed": 2.0, "control/auto_takeoff": True,
    "frame/world": "world", "frame/body": "base_link",
}
PUBS, SUBS = {}, {}
CLOCK = {"t": 100.0}


class _Publisher(object):
    def __init__(self, name, msg_type, queue_size=10):
        self.name = name
        self.msgs = []
        PUBS[name] = self

    def publish(self, msg):
        self.msgs.append(msg)


class _Time(object):
    @staticmethod
    def now():
        return CLOCK["t"]


class _Rate(object):
    def __init__(self, hz):
        self.hz = hz

    def sleep(self):
        time.sleep(0.001)


def install_rospy_stub():
    m = types.ModuleType("rospy")
    m.get_param = lambda name, default=None: PARAMS.get(name, default)
    m.Publisher = _Publisher
    m.Subscriber = lambda topic, msg_type, cb, queue_size=1: SUBS.__setitem__(topic, cb)
    m.Rate = _Rate
    m.Time = _Time
    m.init_node = lambda *a, **k: None
    m.is_shutdown = lambda: False
    m.get_time = lambda: CLOCK["t"]
    m.loginfo = lambda *a, **k: None
    m.logwarn = lambda *a, **k: None
    m.logwarn_throttle = lambda *a, **k: None
    core = types.ModuleType("rospy.core")
    core.is_initialized = lambda: True
    m.core = core
    sys.modules["rospy"] = m
    sys.modules["rospy.core"] = core

    gm = types.ModuleType("geometry_msgs")
    gmm = types.ModuleType("geometry_msgs.msg")
    gmm.Point, gmm.Twist, gmm.TransformStamped = Point, Twist, TransformStamped
    gmm.Vector3, gmm.Quaternion, gmm.Pose = Vec3, Quat, Pose
    gm.msg = gmm
    sm = types.ModuleType("std_msgs")
    smm = types.ModuleType("std_msgs.msg")
    smm.String = String
    sm.msg = smm
    nm = types.ModuleType("nav_msgs")
    nmm = types.ModuleType("nav_msgs.msg")
    nmm.Odometry = Odometry
    nm.msg = nmm
    tf2 = types.ModuleType("tf2_ros")

    class _Broadcaster(object):
        sent = []

        def __init__(self):
            pass

        def sendTransform(self, msg):
            _Broadcaster.sent.append(msg)

    tf2.TransformBroadcaster = _Broadcaster
    sys.modules.update({
        "geometry_msgs": gm, "geometry_msgs.msg": gmm,
        "std_msgs": sm, "std_msgs.msg": smm,
        "nav_msgs": nm, "nav_msgs.msg": nmm, "tf2_ros": tf2,
    })
    return _Broadcaster


TF = install_rospy_stub()
import cmd_vel_sub  # noqa: E402
import odom_pub  # noqa: E402

import numpy as np  # noqa: E402


class FakeSim(object):
    """记录所有下发指令的假客户端."""

    def __init__(self, **kwargs):
        self.calls = []
        self.state = (np.array([1.0, 2.0, 3.0]), (0.0, 0.0, 0.7071, 0.7071),
                      np.array([0.1, 0.2, 0.3]), np.array([0.01, 0.02, 0.03]))

    def connect(self):
        self.calls.append(("connect",))

    def takeoff(self, timeout=20.0):
        self.calls.append(("takeoff",))

    def hover(self):
        self.calls.append(("hover",))

    def move_by_velocity_body(self, f, l, u, duration=0.2, yaw_rate=0.0):
        self.calls.append(("vel_body", f, l, u, yaw_rate))

    def move_by_velocity_enu(self, v, duration=0.2, yaw_rate=0.0):
        self.calls.append(("vel_enu", tuple(v), yaw_rate))

    def move_to_position_enu(self, target, speed=2.0):
        self.calls.append(("goal", tuple(target), speed))

    def get_state_enu(self):
        return self.state


# ================================================================= A. 状态机
def test_cmd_vel_state_machine():
    print("== A. cmd_vel_sub 状态机 ==")
    fake = FakeSim()
    cmd_vel_sub.SimClient = lambda **kw: fake
    node = cmd_vel_sub.CmdVelSubscriber()

    check(("connect",) in fake.calls, "构造时已连接仿真器")
    check(("takeoff",) in fake.calls, "auto_takeoff 生效（自动起飞并悬停）")

    # 1) 无指令 -> 悬停
    check(node.is_hovering, "初始状态为悬停")

    # 2) 速度指令 -> 限幅后按机体系下发
    CLOCK["t"] = 100.0
    tw = Twist()
    tw.linear.x, tw.linear.y, tw.linear.z = 100.0, -100.0, 100.0   # 故意超限
    tw.angular.z = 100.0
    SUBS["/uav/cmd_vel"](tw)
    node.step_once()
    vel_calls = [c for c in fake.calls if c[0] == "vel_body"]
    check(len(vel_calls) == 1, "收到速度指令后下发一次机体系速度")
    if vel_calls:
        _, f, l, u, wz = vel_calls[0]
        check(abs(f) <= 5.0 and abs(l) <= 5.0 and abs(u) <= 3.0 and abs(wz) <= 1.0,
              "速度与偏航角速度均被限幅 (%.1f, %.1f, %.1f, %.1f)" % (f, l, u, wz))
    check(node.is_hovering is False, "进入速度控制状态")
    check(PUBS["/uav/status"].msgs[-1].data == "VELOCITY", "状态话题变为 VELOCITY")

    # 3) 指令超时 -> 自动悬停（安全保护）
    CLOCK["t"] = 100.0 + 1.0   # 超过 cmd_timeout = 0.5 s
    before = len([c for c in fake.calls if c[0] == "hover"])
    node.step_once()
    after = len([c for c in fake.calls if c[0] == "hover"])
    check(after == before + 1, "指令超时后自动悬停（安全保护）")
    check(node.is_hovering, "回到悬停状态")
    check(PUBS["/uav/status"].msgs[-1].data == "HOVER", "状态话题变为 HOVER")

    # 4) 目标点任务 -> 期间速度通道让位
    p = Point()
    p.x, p.y, p.z = 30.0, 10.0, -8.0
    SUBS["/uav/goal"](p)
    time.sleep(0.05)
    goal_calls = [c for c in fake.calls if c[0] == "goal"]
    check(len(goal_calls) == 1 and goal_calls[0][1] == (30.0, 10.0, -8.0),
          "目标点指令原样下发给仿真器（ENU 坐标）")

    statuses = [m.data for m in PUBS["/uav/status"].msgs]
    check(any(s.startswith("GOAL ") for s in statuses), "状态话题出现 GOAL")
    check(any(s.startswith("GOAL_DONE") for s in statuses), "状态话题出现 GOAL_DONE")

    # 目标点执行中（模拟）时速度指令被忽略
    node.goal_active = True
    n_before = len([c for c in fake.calls if c[0] == "vel_body"])
    SUBS["/uav/cmd_vel"](tw)
    node.step_once()
    node.goal_active = False
    n_after = len([c for c in fake.calls if c[0] == "vel_body"])
    check(n_after == n_before, "目标点任务期间速度指令让位（不打断）")


# ================================================================= B. odom
def test_odom_publish():
    print("== B. odom_pub 字段与 TF ==")
    for d in (PUBS,):
        d.clear()
    TF.sent.clear()
    fake = FakeSim()
    odom_pub.SimClient = lambda **kw: fake
    node = odom_pub.OdomPublisher()
    node.publish(*fake.state)

    check("/uav/odom" in PUBS and len(PUBS["/uav/odom"].msgs) == 1, "发布了一条 /uav/odom")
    odom = PUBS["/uav/odom"].msgs[0]
    check((odom.pose.pose.position.x, odom.pose.pose.position.y,
           odom.pose.pose.position.z) == (1.0, 2.0, 3.0), "位置字段正确")
    check((odom.twist.twist.linear.x, odom.twist.twist.linear.z) == (0.1, 0.3),
          "线速度字段正确")
    check((odom.twist.twist.angular.y,) == (0.02,), "角速度字段正确")
    check(odom.header.frame_id == "world" and odom.child_frame_id == "base_link",
          "frame_id / child_frame_id 正确")

    check(len(TF.sent) == 1, "广播了一条 TF")
    tf_msg = TF.sent[0]
    check(tf_msg.header.frame_id == "world" and tf_msg.child_frame_id == "base_link",
          "TF 父子坐标系正确")
    check((tf_msg.transform.translation.x, tf_msg.transform.rotation.y) == (1.0, 0.0),
          "TF 平移与旋转字段写入正确")


# ================================================================= C. 工程文件
def test_project_files():
    print("== C. 工程文件合法性 ==")
    with open(os.path.join(PKG, "config", "bridge.yaml"), encoding="utf-8") as fh:
        cfg = __import__("yaml").safe_load(fh)
    check(cfg["sim"]["host"] and cfg["sim"]["airsim_port"] == 41451,
          "bridge.yaml 可解析且含 sim.host / airsim_port")
    check(all(k in cfg for k in ("rate", "frame", "topic", "control")),
          "bridge.yaml 含 rate/frame/topic/control 四组参数")

    with open(os.path.join(PKG, "config", "settings.json"), encoding="utf-8") as fh:
        sj = json.load(fh)
    drone = sj["Vehicles"]["Drone1"]
    check(sj["SimMode"] == "Multirotor", "settings.json: SimMode = Multirotor")
    check(set(drone["Cameras"]) >= {"front_rgb", "front_depth", "front_seg"},
          "settings.json: 相机含 RGB/深度/语义分割")
    check(drone["Sensors"]["lidar1"]["SensorType"] == 6,
          "settings.json: lidar1 为 Lidar 传感器 (SensorType=6)")

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    types_of = [n.get("type") for n in launch.iter("node")]
    check(set(types_of) >= {"main.py", "odom_pub.py", "cmd_vel_sub.py"},
          "main.launch 含 main.py / odom_pub.py / cmd_vel_sub.py 三个节点")
    check(any(n.tag == "rosparam" for n in launch.iter("rosparam")),
          "main.launch 已加载 bridge.yaml")

    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.find("name").text == "carlair_ros_bridge", "package.xml 包名正确")

    with open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8") as fh:
        cmake = fh.read()
    check("catkin_install_python" in cmake and "odom_pub.py" in cmake,
          "CMakeLists.txt 已安装三个节点脚本")


def main():
    for fn in (test_cmd_vel_state_machine, test_odom_publish, test_project_files):
        fn()
    print("")
    total = len(PASS) + len(FAIL)
    print("桥接本地测试: %d/%d 通过" % (len(PASS), total))
    if FAIL:
        print("失败项: %s" % FAIL)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
