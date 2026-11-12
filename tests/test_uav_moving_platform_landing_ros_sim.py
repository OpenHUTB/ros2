#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环冒烟测试：用假 carlair_ros_bridge 驱 uav_moving_platform_landing 完成一次移动平台降落。

假桥接层替代真实仿真器，负责：

* 20 Hz 发布 ``/uav/odom``         —— ENU（z 向上为正，与 carlair_ros_bridge 一致）
* 10 Hz 发布 ``/uav/platform_pose`` —— 平台中心的**真值**，但时间戳比当前时刻旧
  ``latency`` 秒、并叠加高斯噪声（模拟视觉/RPC 取数的实际情况）
* 把 ``/uav/cmd_vel`` 积分进一阶速度滞后模型，于是节点真的能一层层降下去
* 平台按给定的运动（直线 / 圆周）移动，节点只能看到带噪带延迟的观测

跑法（宿主机 RoboStack ROS1 或虚拟机 ROS Noetic 都行）::

    roscore &
    python3 tests/test_uav_moving_platform_landing_ros_sim.py

退出码 0 = 通过；会打印状态机全过程与最终落点误差。
"""
from __future__ import print_function

import math
import sys
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

DECK_SPEED = 0.6          # 平台速度（m/s）
START_ALT = 6.0           # 无人机初始高度（相对平台）
LATENCY = 0.15            # 平台观测延迟（s）
NOISE = 0.03              # 平台观测噪声（m）
RUN_SECONDS = 70.0
Z_UP = True               # 与 config 的 frame/z_up 一致

S = {
    "pos": np.array([0.0, 0.0, START_ALT]),
    "vel": np.zeros(3),
    "cmd": np.zeros(3),
    "state": "",
    "trace": [],
}
lock = threading.Lock()


def deck_pos(t):
    return np.array([DECK_SPEED * t, 0.0, 0.0])


def on_rosout(msg):
    if "moving_landing" in (msg.name or ""):
        line = (msg.msg or "").strip()
        if line:
            print("[node] %s" % line)
            sys.stdout.flush()


def on_cmd(msg):
    with lock:
        S["cmd"] = np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=float)


def on_state(msg):
    import json
    with lock:
        try:
            data = json.loads(msg.data)
        except Exception:                       # pragma: no cover - 话题格式异常
            return
        S["state"] = data.get("state", "")


def main():
    rospy.init_node("fake_carlair_bridge", anonymous=True, disable_signals=True)

    pub_odom = rospy.Publisher("/uav/odom", Odometry, queue_size=1)
    pub_deck = rospy.Publisher("/uav/platform_pose", PoseStamped, queue_size=5)
    rospy.Subscriber("/rosout", Log, on_rosout, queue_size=500)
    rospy.Subscriber("/uav/cmd_vel", Twist, on_cmd, queue_size=1)
    rospy.Subscriber("/uav/moving_landing/state", String, on_state, queue_size=5)
    rospy.sleep(1.5)                            # 等节点起来

    print("[bridge] 平台 %.1f m/s 直线运动，观测延迟 %.2f s、噪声 %.3f m，无人机初始高度 %.1f m"
          % (DECK_SPEED, LATENCY, NOISE, START_ALT))
    sys.stdout.flush()

    rng = np.random.RandomState(7)
    dt = 0.05
    sim_t = 0.0
    last_deck_pub = -1.0
    landed_at = None
    err_at_land = None
    final_state = ""
    t0 = time.time()

    while not rospy.is_shutdown():
        now_wall = time.time() - t0
        sim_t = now_wall
        # 平台真值 + 带噪带延迟的观测
        truth = deck_pos(sim_t)
        if sim_t - last_deck_pub >= 0.1:
            last_deck_pub = sim_t
            # 时间戳必须是 ROS 时钟（与节点里的 rospy.Time.now() 同源），只是比现在旧
            # LATENCY 秒 —— 节点靠这个差值做延迟补偿
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(LATENCY)
            msg.header.frame_id = "world"
            p_meas = deck_pos(max(0.0, sim_t - LATENCY)) + rng.normal(0.0, NOISE, 3)
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = list(p_meas)
            msg.pose.orientation.w = 1.0
            pub_deck.publish(msg)

        # 无人机（一阶速度滞后 + 速度限幅）
        with lock:
            cmd = S["cmd"].copy()
            pos = S["pos"].copy()
            vel = S["vel"].copy()
            state = S["state"]
        tau = np.array([0.35, 0.35, 0.30])
        alpha = np.clip(dt / tau, 0.0, 1.0)
        vel = vel + (cmd - vel) * alpha
        pos = pos + vel * dt
        z = pos[2] if Z_UP else -pos[2]
        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = "world"
        od.pose.pose.position.x = pos[0]
        od.pose.pose.position.y = pos[1]
        od.pose.pose.position.z = z if Z_UP else -z
        od.twist.twist.linear.x = vel[0]
        od.twist.twist.linear.y = vel[1]
        od.twist.twist.linear.z = vel[2] if Z_UP else -vel[2]
        pub_odom.publish(od)
        with lock:
            S["pos"] = pos
            S["vel"] = vel

        # 接地检测（用真值）：相对高度 <= 0.12 m
        z_rel = pos[2] - truth[2] if Z_UP else -pos[2] - truth[2]
        if landed_at is None and z_rel <= 0.12 and state in ("TOUCHDOWN", "LANDED"):
            landed_at = sim_t
            err_at_land = float(math.hypot(pos[0] - truth[0], pos[1] - truth[1]))
            final_state = state
            print("[bridge] 接地：横向误差 %.3f m（平台在 %.2f, 无人机在 %.2f）"
                  % (err_at_land, truth[0], pos[0]))
            sys.stdout.flush()

        if landed_at is not None and sim_t - landed_at > 1.0:
            break
        if sim_t > RUN_SECONDS:
            final_state = state or "TIMEOUT"
            break
        time.sleep(dt)

    print("\n=== 结果 ===")
    print("最终状态：%s" % (final_state or "未终结"))
    err_text = ("%.3f m" % err_at_land) if err_at_land is not None else "未接地"
    print("接地横向误差：%s" % err_text)
    ok = (landed_at is not None and err_at_land is not None and err_at_land <= 0.30)
    print("结论：%s" % ("通过（落点误差 ≤ 0.30 m）" if ok else "失败"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
