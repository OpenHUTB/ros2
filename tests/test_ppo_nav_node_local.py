#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""部署节点 ppo_nav_node.py 的本地测试（用桩替身 mock 掉 rospy / 消息类型）.

让**没有 ROS 环境**的机器也能验证部署节点的真实逻辑：
    A. parse_goal      —— 兼容 roslaunch 传列表与字符串两种写法
    B. cloud_to_xyz    —— PointCloud2 -> (N,3)
    C. PpoNavNode      —— 观测装配（世界系语义）、推理、发布速度指令
    D. 边界            —— 尚无点云 / 尚无里程计时不崩

用法: python3 tests/test_ppo_nav_node_local.py
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
SCRIPTS = os.path.join(ROOT, "src", "air", "uav_ppo_nav", "scripts")
WEIGHTS = os.path.join(ROOT, "src", "air", "uav_ppo_nav", "models", "policy_weights.npz")
sys.path.insert(0, SCRIPTS)


# ---------------------------------------------------------------- 桩：消息类型
class _Vec(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quat(_Vec):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        _Vec.__init__(self, x, y, z)
        self.w = w


class _Pose(object):
    def __init__(self):
        self.position = _Vec()
        self.orientation = _Quat()


class _Twist(object):
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()


class _Odometry(object):
    def __init__(self):
        self.pose = types.SimpleNamespace(pose=_Pose())
        self.twist = types.SimpleNamespace(twist=_Twist())


class _PointCloud2(object):
    def __init__(self):
        self.data = b""
        self.point_step = 12


def _install_stub_modules():
    for mod, cls, nm in (("geometry_msgs.msg", _Twist, "Twist"),
                         ("nav_msgs.msg", _Odometry, "Odometry"),
                         ("sensor_msgs.msg", _PointCloud2, "PointCloud2")):
        m = types.ModuleType(mod)
        setattr(m, nm, cls)
        sys.modules[mod] = m
        parent = types.ModuleType(mod.split(".")[0])
        setattr(parent, "msg", m)
        sys.modules.setdefault(mod.split(".")[0], parent)

    rospy = types.ModuleType("rospy")
    rospy.params = {}
    rospy.published = []
    rospy.subs = {}
    rospy.logged = []           # [(level, text)]，供测试断言

    def _mk_logger(level):
        def _log(msg, *a, **k):
            rospy.logged.append((level, (msg % a) if a else msg))
        return _log

    class _Publisher(object):
        def __init__(self, topic, typ, queue_size=1):
            self.topic = topic

        def publish(self, msg):
            rospy.published.append((self.topic, msg))

    class _Subscriber(object):
        def __init__(self, topic, typ, cb, queue_size=1):
            rospy.subs[topic] = cb

    class _Rate(object):
        def __init__(self, hz):
            self.hz = hz

        def sleep(self):
            pass

    rospy.get_param = lambda name, default=None: rospy.params.get(name, default)
    rospy.Publisher = _Publisher
    rospy.Subscriber = _Subscriber
    rospy.Rate = _Rate
    rospy.init_node = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.loginfo = _mk_logger("info")
    rospy.logwarn = _mk_logger("warn")
    rospy.logerr = _mk_logger("err")
    rospy.loginfo_throttle = _mk_logger("info")
    rospy.logwarn_throttle = _mk_logger("warn")
    sys.modules["rospy"] = rospy
    return rospy


# ================================================================= A. parse_goal
def test_parse_goal(N):
    print("== A. parse_goal ==")
    check(np.allclose(N.parse_goal([1.0, 2.0, 3.0]), [1, 2, 3]), "列表形式")
    check(np.allclose(N.parse_goal("[1.0, 2.0, 3.0]"), [1, 2, 3]),
          "字符串形式（roslaunch 的 <param value=\"[$(arg ...)]\"/> 会退化成字符串）")
    check(np.allclose(N.parse_goal("[-4.5, 0.5, -27]"), [-4.5, 0.5, -27]),
          "含负数的字符串")


# ================================================================= B. cloud_to_xyz
def test_cloud_to_xyz(N):
    print("== B. cloud_to_xyz ==")
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    pc = N.PointCloud2()
    pc.data = pts.tobytes()
    pc.point_step = 12
    out = N.cloud_to_xyz(pc)
    check(out.shape == (2, 3) and np.allclose(out, pts), "PointCloud2 -> (N,3) float64")


# ================================================================= C. 节点主逻辑
def test_node(N, rospy):
    print("== C. PpoNavNode 观测装配 / 推理 / 发布 ==")
    rospy.params["nav/weights"] = WEIGHTS
    rospy.params["nav/goal"] = [10.0, 0.0, 0.0]
    rospy.params["rate/publish_hz"] = 10.0
    rospy.params["topic/cmd_vel"] = "/uav/cmd_vel"
    node = N.PpoNavNode()

    od = N.Odometry()
    od.pose.pose.position.x = 1.0
    od.pose.pose.position.y = 2.0
    od.pose.pose.position.z = -3.0
    od.twist.twist.linear.x = 0.5
    node._cb_odom(od)

    # 一个位于无人机东北 45°、水平距离 3 m 的障碍点
    d = 3.0 / math.sqrt(2.0)
    p = np.array([[1.0 + d, 2.0 + d, -3.0]], dtype=np.float32)
    pc = N.PointCloud2()
    pc.data = p.tobytes()
    pc.point_step = 12
    node._cb_cloud(pc)

    obs = node.build_obs()
    check(obs.shape == (22,), "观测维度 = 22")
    check(abs(obs[16] - 9.0 / 8.0) < 1e-5, "目标相对位置 x = (10-1)/8 = 1.125（世界系）")
    check(abs(obs[17] - (-2.0 / 8.0)) < 1e-5, "目标相对位置 y = (0-2)/8 = -0.25")
    check(abs(obs[18] - 3.0 / 8.0) < 1e-5, "目标相对位置 z = (0-(-3))/8 = 0.375")
    check(abs(obs[19] - 0.5 / 3.0) < 1e-5, "速度观测 x = 0.5/3")
    check(abs(obs[2] - 3.0 / 12.0) < 1e-4, "扇形 2（+45°）值 = 3/12 = 0.25")
    check(abs(obs[0] - 1.0) < 1e-6 and abs(obs[5] - 1.0) < 1e-6, "其他扇形 = 1（无遮挡）")

    rospy.published[:] = []
    node.step_once()
    check(len(rospy.published) == 1, "step_once 发布 1 条速度指令")
    topic, msg = rospy.published[0]
    check(topic == "/uav/cmd_vel", "发布话题 = /uav/cmd_vel")
    check(abs(msg.linear.x) <= 3.0 and abs(msg.linear.y) <= 3.0,
          "水平速度在 ±3 m/s 内")
    check(abs(msg.linear.z) <= 1.5, "垂直速度在 ±1.5 m/s 内")
    check(msg.linear.x > 0, "目标在东侧 -> 东向速度为正向（世界系语义正确）")


# ================================================================= D. 边界
def test_edge(N):
    print("== D. 边界情况 ==")
    node = N.PpoNavNode()
    od = N.Odometry()
    od.pose.pose.position.z = -3.0
    node._cb_odom(od)
    obs = node.build_obs()
    check(obs.shape == (22,) and np.allclose(obs[:16], 1.0),
          "尚无点云 -> 直方图全 1（视为无遮挡），不崩溃")


# ================================================================= E. body_frame 安全校验
def test_body_frame_guard(N, rospy):
    print("== E. body_frame 安全校验（防止世界系速度被机体系执行）==")
    rospy.params["nav/weights"] = WEIGHTS
    rospy.params["nav/goal"] = [10.0, 0.0, 0.0]
    rospy.params["rate/publish_hz"] = 10.0
    rospy.params["topic/cmd_vel"] = "/uav/cmd_vel"

    rospy.params["control/body_frame"] = True      # 桥接用错 -> 必须报错
    rospy.logged[:] = []
    N.PpoNavNode()
    check(any(lv == "err" for lv, _ in rospy.logged),
          "桥接 body_frame=true 时打印 logerr（否则会静默乱飞）")

    rospy.params["control/body_frame"] = False     # 正确 -> 不应报错
    rospy.logged[:] = []
    N.PpoNavNode()
    check(not any(lv == "err" for lv, _ in rospy.logged),
          "body_frame=false（正确用法）不报 error")

    rospy.params.pop("control/body_frame", None)   # 桥接没起 -> 提示
    rospy.logged[:] = []
    N.PpoNavNode()
    check(any(lv == "warn" for lv, _ in rospy.logged),
          "读不到 control/body_frame（桥接未启动）时打印 logwarn")


def main():
    rospy = _install_stub_modules()
    import ppo_nav_node as N  # noqa: E402  (必须在装好桩之后导入)

    test_parse_goal(N)
    test_cloud_to_xyz(N)
    test_node(N, rospy)
    test_edge(N)
    test_body_frame_guard(N, rospy)

    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for f in FAIL:
            print("  FAIL: %s" % f)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
