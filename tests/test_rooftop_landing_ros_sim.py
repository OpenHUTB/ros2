#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环冒烟测试：用假 carlair_ros_bridge 驱动 rooftop_landing 节点完成一次屋顶降落。

假桥接层替代真实仿真器，负责：
  * 20 Hz 发布 /uav/odom      —— ENU（z 向上为正，与 carlair_ros_bridge 一致），初始 15 m 高度
  * 10 Hz 发布 /lidar/points  —— ENU（z 向上为正），8 m 高的 8x8 m 屋顶 + z=0 地面
  * 收到 /uav/goal 后瞬移到目标点上方，并回报 /uav/status =
    "GOAL_DONE <x> <y> <z>"（**与真实 carlair_ros_bridge 的格式一致，带三个坐标**）
  * 把 /uav/cmd_vel.linear.z 积分进高度，于是节点真的能一层层降下去

输出：节点状态机全过程（抓 /rosout）+ 最终高度，用来证明整条链路可用。
"""
from __future__ import print_function

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

ROOF_Z = 8.0            # 屋顶高度（ENU，向上为正）
ROOF_HALF = 4.0         # 屋顶半边长 -> 8 x 8 m
ROOF_CENTER = (8.0, 0.0)
START_ALT = 15.0        # 无人机初始高度
Z_UP = True             # 与 config/rooftop_landing.yaml 的 frame/z_up 一致
RUN_SECONDS = 26.0

S = {
    "alt": START_ALT,
    "xy": (0.0, 0.0),
    "cmd_z": 0.0,
    "goal": None,
    "landing_point": None,
    "trace": [],
}
lock = threading.Lock()


def altitude(z_raw):
    return z_raw if Z_UP else -z_raw


def to_raw(alt):
    return alt if Z_UP else -alt


def build_cloud():
    """8 x 8 m 屋顶面 + 大块地面，都是 ENU（z 向上为正）。"""
    xs = np.arange(ROOF_CENTER[0] - ROOF_HALF, ROOF_CENTER[0] + ROOF_HALF + 1e-9, 0.25)
    ys = np.arange(ROOF_CENTER[1] - ROOF_HALF, ROOF_CENTER[1] + ROOF_HALF + 1e-9, 0.25)
    gx, gy = np.meshgrid(xs, ys)
    roof = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, ROOF_Z)], axis=1)

    gxs = np.arange(-10.0, 26.0, 1.0)
    gys = np.arange(-14.0, 14.0, 1.0)
    ggx, ggy = np.meshgrid(gxs, gys)
    ground = np.stack([ggx.ravel(), ggy.ravel(), np.zeros(ggx.size)], axis=1)

    return np.vstack([roof, ground]).astype(np.float32), roof.shape[0], ground.shape[0]


def on_rosout(msg):
    if "rooftop_landing" in (msg.name or ""):
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
        S["landing_point"] = (float(msg.x), float(msg.y), float(msg.z))


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
    rospy.Subscriber("/rooftop/landing_point", Point, on_landing, queue_size=1)
    rospy.Subscriber("/uav/cmd_vel", Twist, on_cmd, queue_size=1)
    rospy.sleep(1.0)          # 等订阅建立

    pts, n_roof, n_ground = build_cloud()
    print("[bridge] 合成点云 %d 点（屋顶 %d 点 z=%.1f m，%.0fx%.0f m；地面 %d 点 z=0）"
          % (len(pts), n_roof, ROOF_Z, 2 * ROOF_HALF, 2 * ROOF_HALF, n_ground))
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
            S["alt"] += S["cmd_z"] * dt          # 把速度指令积分成真实高度变化
            if S["alt"] < 0.2:
                S["alt"] = 0.2
            g = S["goal"]
            alt = S["alt"]
            xy = S["xy"]

        if g is not None and goal_at is None:
            with lock:
                S["xy"] = (g[0], g[1])
                S["alt"] = max(S["alt"], altitude(g[2]))
                xy = S["xy"]
                alt = S["alt"]
            goal_at = now
            print("[bridge] 收到目标点 (%.2f, %.2f, raw_z=%+.2f 即 %.2f m)，"
                  "飞抵其上方" % (g[0], g[1], g[2], altitude(g[2])))
            sys.stdout.flush()

        if goal_at is not None and (now - goal_at) > 0.6:
            # ⚠️ 必须与真实 carlair_ros_bridge 完全一致：状态词后面带三个坐标
            # （cmd_vel_sub.py 发的是 "GOAL_DONE %.2f %.2f %.2f" % target，其中
            # target 就是**它收到的那个目标点**，不是到达后的实测位置）。
            # 早期这里只发一个裸词 "GOAL_DONE"，于是掩盖了节点用
            # `status == "GOAL_DONE"` 全串比较的 bug —— 假桥接测试通过、真机必挂；
            # 而上报实测位置又会和节点下发的目标点对不上（节点靠坐标分辨
            # 「这条到达状态是不是自己的目标点」）。
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

        if now - last_print > 2.0:
            last_print = now
            print("[bridge] alt=%.2f m  cmd_z=%+.2f m/s  status=%s"
                  % (alt, S["cmd_z"], "GOAL_DONE" if goal_at else "-"))
            sys.stdout.flush()

        rate.sleep()

    with lock:
        alt = S["alt"]
        trace = list(S["trace"])
        lp = S["landing_point"]
        goal = S["goal"]

    joined = " | ".join(trace)
    print("=" * 70)
    print("目标点话题 /uav/goal      : %s" % (goal,))
    print("降落点话题 /rooftop/landing_point : %s" % (lp,))
    print("最终高度 %.2f m（屋顶 %.1f m，容差 0.35 m）" % (alt, ROOF_Z))
    print("节点状态机轨迹：")
    for line in trace:
        print("  - %s" % line)
    print("出现过 LANDED : %s" % ("是" if "LANDED" in joined else "否"))
    print("出现过 APPROACH: %s" % ("是" if "APPROACH" in joined else "否"))
    print("出现过 DESCEND : %s" % ("是" if "DESCEND" in joined else "否"))
    print("=" * 70)


if __name__ == "__main__":
    main()
