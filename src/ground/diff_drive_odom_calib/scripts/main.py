#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""里程计标定主节点（ROS 1）：直行 + 原地旋转两个实验，辨识三个系数。

只通过话题与仿真机器人交互，不直接调用 MuJoCo：
    订阅 /ground_truth（外部测量，相当于现场拉卷尺）、/wheel_ticks（编码器）
    发布 /cmd_vel
换成真车时只要也发这两个话题，本节点一行都不用改。

神经网络滑移补偿的训练与三组对比在 tools/run_experiment.py（离线、可复现）。
"""

import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray

try:
    from diff_drive_odom_calib import config
    from diff_drive_odom_calib.calibration import identify
except ImportError:                                          # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from diff_drive_odom_calib import config
    from diff_drive_odom_calib.calibration import identify


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class CalibNode(object):

    def __init__(self):
        self.r_nominal = float(rospy.get_param("~r_nominal", config.R_NOMINAL))
        self.b_nominal = float(rospy.get_param("~b_nominal", config.B_NOMINAL))
        self.dist = float(rospy.get_param("~straight_distance", config.STRAIGHT_DISTANCE))
        self.v = float(rospy.get_param("~straight_speed", config.STRAIGHT_SPEED))
        self.turns = float(rospy.get_param("~rotation_turns", config.ROTATION_TURNS))
        self.w = float(rospy.get_param("~rotation_speed", config.ROTATION_SPEED))
        self.side = float(rospy.get_param("~square_side", config.SQUARE_SIDE))
        self.warmup = float(rospy.get_param("~warmup", 1.5))

        self.truth = None
        self.ticks = None
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/ground_truth", PoseStamped, self.on_truth, queue_size=50)
        rospy.Subscriber("/wheel_ticks", Float32MultiArray, self.on_ticks, queue_size=50)

    def on_truth(self, msg):
        q = msg.pose.orientation
        self.truth = (msg.pose.position.x, msg.pose.position.y,
                      math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z))

    def on_ticks(self, msg):
        self.ticks = (float(msg.data[0]), float(msg.data[1]))

    def _ready(self, timeout=15.0):
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.truth is not None and self.ticks is not None:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rate.sleep()
        return False

    def _capture(self, segments):
        """执行指令序列，返回逐帧真值增量与编码器增量。

        丢弃开头 warmup 秒：起步加速阶段滑移率高达 30~45%，
        算进标定会让 c_s 偏小（实测 0.966 → 0.914）。
        """
        tw = Twist()
        log = []
        skip = int(round(self.warmup * config.CONTROL_HZ))
        k = 0
        for v, w, dur in segments:
            tw.linear.x, tw.angular.z = v, w
            self.pub.publish(tw)
            rate = rospy.Rate(config.CONTROL_HZ)
            t0 = rospy.Time.now()
            prev = (self.truth, self.ticks)
            while (rospy.Time.now() - t0).to_sec() < dur and not rospy.is_shutdown():
                cur = (self.truth, self.ticks)
                if prev[0] is not None and cur[0] is not None:
                    dx = cur[0][0] - prev[0][0]
                    dy = cur[0][1] - prev[0][1]
                    k += 1
                    if k > skip:
                        log.append(dict(ds_true=math.hypot(dx, dy),
                                        dth_true=wrap(cur[0][2] - prev[0][2]),
                                        dphi_left=cur[1][0] - prev[1][0],
                                        dphi_right=cur[1][1] - prev[1][1]))
                prev = cur
                rate.sleep()
        self.pub.publish(Twist())
        rospy.sleep(0.2)
        return log

    def _err(self, log, c_s, c_theta, k_head):
        r, b = self.r_nominal, self.b_nominal
        x = y = th = tx = ty = tth = 0.0
        for o in log:
            ds = c_s * r * 0.5 * (o["dphi_left"] + o["dphi_right"])
            dth = c_theta * r * (o["dphi_right"] - o["dphi_left"]) / b + k_head * ds
            x += ds * math.cos(th + 0.5 * dth)
            y += ds * math.sin(th + 0.5 * dth)
            th = wrap(th + dth)
            tx += o["ds_true"] * math.cos(tth + 0.5 * o["dth_true"])
            ty += o["ds_true"] * math.sin(tth + 0.5 * o["dth_true"])
            tth = wrap(tth + o["dth_true"])
        return math.hypot(x - tx, y - ty), wrap(th - tth)

    def run(self):
        rospy.loginfo("等待 /ground_truth 与 /wheel_ticks ...")
        if not self._ready():
            rospy.logerr("超时：没收到话题，先启动 sim_node.py？")
            return 1

        rospy.loginfo("① 直行 %.1f m @ %.2f m/s", self.dist, self.v)
        straight = self._capture([(self.v, 0.0, self.dist / self.v)])
        rospy.loginfo("② 原地旋转 %.1f 圈 @ %.2f rad/s", self.turns, self.w)
        rotation = self._capture([(0.0, self.w, self.turns * 2 * math.pi / self.w)])

        prm = identify(straight, rotation, r_nominal=self.r_nominal, b_nominal=self.b_nominal)
        print("\n" + "=" * 58)
        print("辨识结果")
        print("=" * 58)
        print("  c_s     = %.5f   路程系数（平均轮径 + 纵向滑移）" % prm["c_s"])
        print("  k_head  = %+.5f rad/m  直行漂移率" % prm["k_head"])
        print("  c_theta = %.5f   转角系数（轮距 + 转向滑移）" % prm["c_theta"])

        rospy.loginfo("③ 方形路径 %.1f m 验证", self.side)
        turn = (math.pi / 2) / self.w
        segs = []
        for _ in range(4):
            segs.append((self.v, 0.0, self.side / self.v))
            segs.append((0.0, self.w, turn))
        log = self._capture(segs)
        for name, cs, cth, kh in (("标定前", 1.0, 1.0, 0.0),
                                  ("标定后", prm["c_s"], prm["c_theta"], prm["k_head"])):
            e, eth = self._err(log, cs, cth, kh)
            print("  %-6s 终点位置误差 %.3f m   航向误差 %+7.2f°"
                  % (name, e, math.degrees(eth)))
        print("\n  含神经网络的对比见 tools/run_experiment.py")
        return 0


def main():
    rospy.init_node("odom_calib")
    return CalibNode().run()


if __name__ == "__main__":
    sys.exit(main())
