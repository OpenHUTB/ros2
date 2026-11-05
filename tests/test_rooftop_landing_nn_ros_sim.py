#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环冒烟测试：用假 carlair_ros_bridge 驱动 rooftop_landing_nn 完成一次屋顶降落。

假桥接层替代真实仿真器，负责：
  * 20 Hz 发布 /uav/odom      —— ENU（z 向上为正，与 carlair_ros_bridge 一致），初始 16 m 高度
  * 10 Hz 发布 /lidar/points  —— ENU，**U 形屋顶**（10x10 m 挖掉一个 4x5 m 的缺口，
    z=8 m）+ 地面（z=0）+ 屋面边缘的立面回波
  * 收到 /uav/goal 后瞬移到目标点上方，并回报 /uav/status =
    "GOAL_DONE <x> <y> <z>"（与真实 carlair_ros_bridge 的格式一致，带三个坐标）
  * 把 /uav/cmd_vel.linear.z 积分进高度，于是节点真的能一层层降下去

为什么用 U 形屋顶：`rooftop_landing` 走的是「凸包 + polylabel 最大内切圆」，
凸包会把缺口**填平**，落点会被选到缺口上方（空中）；本模块直接学高度图，
应当落在 U 形实体区域内、并且离边界有足够余量。测试最后会自己检查这一点：

    落点是否在 U 形内 & 以落点为心 0.6 m 的圆是否完全在屋面内

用法（客户机，需要先 catkin_make）：

    终端 1: roslaunch rooftop_landing_nn main.launch use_rviz:=false
    终端 2: python3 tests/test_rooftop_landing_nn_ros_sim.py
"""
from __future__ import print_function

import math
import sys
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String

# U 形屋顶：外框 [-5,5]x[-5,5]，从 +y 边挖掉 [-2,2]x[0.5,5]
ROOF_Z = 8.0
OUT = (-5.0, -5.0, 5.0, 5.0)
NOTCH = (-2.0, 0.5, 2.0, 5.0)
START_ALT = 16.0
Z_UP = True
RUN_SECONDS = 30.0
MIN_CLEARANCE = 0.6

S = {"alt": START_ALT, "xy": (0.0, 0.0), "cmd_z": 0.0, "goal": None,
     "landing": None, "trace": []}
lock = threading.Lock()


def altitude(z_raw):
    return z_raw if Z_UP else -z_raw


def to_raw(alt):
    return alt if Z_UP else -alt


def in_u(x, y):
    """点是否落在 U 形屋面内。"""
    if not (OUT[0] <= x <= OUT[2] and OUT[1] <= y <= OUT[3]):
        return False
    if NOTCH[0] <= x <= NOTCH[2] and NOTCH[1] <= y <= NOTCH[3]:
        return False
    return True


def clearance_ok(x, y, radius=MIN_CLEARANCE, samples=48):
    """以 (x, y) 为心、radius 为半径的圆是否完全落在 U 形内（采样检验）。"""
    if not in_u(x, y):
        return False
    for k in range(samples):
        a = 2.0 * math.pi * k / samples
        if not in_u(x + radius * math.cos(a), y + radius * math.sin(a)):
            return False
    return True


def build_cloud():
    """U 形屋面 + 地面 + 立面，全部 ENU（z 向上为正）。"""
    xs = np.arange(OUT[0], OUT[2] + 1e-9, 0.2)
    ys = np.arange(OUT[1], OUT[3] + 1e-9, 0.2)
    gx, gy = np.meshgrid(xs, ys)
    roof = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, ROOF_Z)], axis=1)
    keep = ~((roof[:, 0] >= NOTCH[0]) & (roof[:, 0] <= NOTCH[2]) &
             (roof[:, 1] >= NOTCH[1]) & (roof[:, 1] <= NOTCH[3]))
    roof = roof[keep]

    gxs = np.arange(-12.0, 12.0, 0.8)
    gys = np.arange(-12.0, 12.0, 0.8)
    ggx, ggy = np.meshgrid(gxs, gys)
    ground = np.stack([ggx.ravel(), ggy.ravel(), np.zeros(ggx.size)], axis=1)

    zs = np.arange(0.0, ROOF_Z + 1e-9, 0.5)
    walls = []
    for (xa, ya, xb, yb) in ((OUT[0], OUT[1], OUT[2], OUT[1]),
                             (OUT[0], OUT[3], OUT[2], OUT[3]),
                             (OUT[0], OUT[1], OUT[0], OUT[3]),
                             (OUT[2], OUT[1], OUT[2], OUT[3])):
        n = max(int(max(abs(xb - xa), abs(yb - ya)) / 0.5), 1)
        for t in range(n + 1):
            x = xa + (xb - xa) * t / float(n)
            y = ya + (yb - ya) * t / float(n)
            for z in zs:
                walls.append((x, y, z))
    walls = np.asarray(walls, dtype=np.float64)

    pts = np.vstack([roof, ground, walls]).astype(np.float32)
    return pts, roof.shape[0], ground.shape[0], walls.shape[0]


def on_rosout(msg):
    if "rooftop_landing_nn" in (msg.name or "") or "rooftop_nn" in (msg.msg or ""):
        line = (msg.msg or "").strip()
        if line:
            with lock:
                S["trace"].append(line)
            print("[node] %s" % line)
            sys.stdout.flush()


def on_goal(msg):
    with lock:
        S["goal"] = (float(msg.x), float(msg.y), float(msg.z))


def on_landing(msg):
    with lock:
        S["landing"] = (float(msg.x), float(msg.y), float(msg.z))


def on_cmd(msg):
    with lock:
        S["cmd_z"] = float(msg.linear.z)


def main():
    rospy.init_node("fake_carlair_bridge", anonymous=True, disable_signals=True)
    pub_odom = rospy.Publisher("/uav/odom", Odometry, queue_size=1)
    pub_cloud = rospy.Publisher("/lidar/points", PointCloud2, queue_size=1)
    pub_status = rospy.Publisher("/uav/status", String, queue_size=1)

    rospy.Subscriber("/rosout", Log, on_rosout, queue_size=500)
    rospy.Subscriber("/uav/goal", Point, on_goal, queue_size=1)
    rospy.Subscriber("/rooftop_nn/landing_point", Point, on_landing, queue_size=1)
    rospy.Subscriber("/uav/cmd_vel", Twist, on_cmd, queue_size=1)
    rospy.sleep(1.5)

    pts, n_roof, n_ground, n_wall = build_cloud()
    print("[bridge] 合成点云 %d 点（U 形屋面 %d 点 z=%.1f m，地面 %d 点 z=0，立面 %d 点）"
          % (len(pts), n_roof, ROOF_Z, n_ground, n_wall))
    print("[bridge] 初始高度 %.1f m，frame/z_up=%s" % (START_ALT, Z_UP))
    sys.stdout.flush()

    dt = 0.05
    rate = rospy.Rate(1.0 / dt)
    t0 = time.time()
    goal_at = None
    tick = 0
    last_print = 0.0

    while not rospy.is_shutdown() and (time.time() - t0) < RUN_SECONDS:
        now = time.time()
        with lock:
            S["alt"] += S["cmd_z"] * dt
            if S["alt"] < 0.2:
                S["alt"] = 0.2
            g = S["goal"]
            alt = S["alt"]
            xy = S["xy"]

        if g is not None and goal_at is None:
            with lock:
                S["xy"] = (g[0], g[1])
                S["alt"] = max(S["alt"], altitude(g[2]))
                xy, alt = S["xy"], S["alt"]
            goal_at = now
            print("[bridge] 收到目标点 (%.2f, %.2f, raw_z=%+.2f 即 %.2f m)，飞抵其上方"
                  % (g[0], g[1], g[2], altitude(g[2])))
            sys.stdout.flush()

        if goal_at is not None and (now - goal_at) > 0.6:
            pub_status.publish(String("GOAL_DONE %.2f %.2f %.2f" % (g[0], g[1], g[2])))

        odo = Odometry()
        odo.header.stamp = rospy.Time.now()
        odo.header.frame_id = "world"
        odo.pose.pose.position.x = xy[0]
        odo.pose.pose.position.y = xy[1]
        odo.pose.pose.position.z = to_raw(alt)
        odo.pose.pose.orientation.w = 1.0
        pub_odom.publish(odo)

        tick += 1
        if tick % 2 == 0:
            hdr = Header(stamp=rospy.Time.now(), frame_id="world")
            pub_cloud.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

        if now - last_print > 3.0:
            last_print = now
            print("[bridge] alt=%.2f m  cmd_z=%+.2f m/s  status=%s"
                  % (alt, S["cmd_z"], "GOAL_DONE" if goal_at else "-"))
            sys.stdout.flush()
        rate.sleep()

    with lock:
        alt = S["alt"]
        trace = list(S["trace"])
        lp = S["landing"]
        goal = S["goal"]
    joined = " | ".join(trace)

    print("=" * 74)
    print("目标点话题 /uav/goal            : %s" % (goal,))
    print("落点话题 /rooftop_nn/landing_point : %s" % (lp,))
    print("最终高度 %.2f m（屋面 %.1f m，容差 0.35 m）" % (alt, ROOF_Z))
    if lp is not None:
        print("落点是否在 U 形屋面内          : %s" % ("是" if in_u(lp[0], lp[1]) else "否"))
        print("以落点为心 %.1f m 圆是否全在屋面内: %s"
              % (MIN_CLEARANCE, "是" if clearance_ok(lp[0], lp[1]) else "否"))
    print("节点状态机轨迹：")
    for line in trace:
        print("  - %s" % line)
    print("出现过 LANDED : %s" % ("是" if "LANDED" in joined else "否"))
    print("出现过 APPROACH: %s" % ("是" if "APPROACH" in joined else "否"))
    print("出现过 DESCEND : %s" % ("是" if "DESCEND" in joined else "否"))
    ok = ("LANDED" in joined and lp is not None and in_u(lp[0], lp[1])
          and clearance_ok(lp[0], lp[1]))
    print("结论：%s" % ("通过" if ok else "未通过"))
    print("=" * 74)
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
