#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑一段方形路径，同时把"里程计 vs 真值"的偏差实时打出来。

用途：在没有键盘 TTY 的场合（CI、远程、录屏）演示 ROS 话题真的在动，
并把"里程计漂移"这件事当场量化出来。

    roslaunch diff_drive_odom_calib sim.launch       # 终端 1
    rosrun  diff_drive_odom_calib demo_drive.py      # 终端 2
"""

import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class DemoDrive(object):

    def __init__(self):
        self.side = float(rospy.get_param("~side", 2.0))
        self.v = float(rospy.get_param("~v", 0.30))
        self.w = float(rospy.get_param("~w", 0.80))
        self.laps = int(rospy.get_param("~laps", 1))
        self.odom = None
        self.truth = None
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=10)
        rospy.Subscriber("/ground_truth", PoseStamped, self.on_truth, queue_size=10)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

    def on_truth(self, msg):
        q = msg.pose.orientation
        self.truth = (msg.pose.position.x, msg.pose.position.y,
                      math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

    def _send(self, v, w, duration):
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
            self.pub.publish(tw)
            rate.sleep()

    def run(self):
        rospy.loginfo("等待 /odom 与 /ground_truth ...")
        rate = rospy.Rate(20)
        t0 = rospy.Time.now()
        while (self.odom is None or self.truth is None) and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > 10:
                rospy.logerr("超时：没有收到话题")
                return 1
            rate.sleep()

        print("")
        print("  方形路径 %d 圈，边长 %.1f m，v=%.2f m/s" % (self.laps, self.side, self.v))
        print("  %8s%26s%26s%10s"
              % ("时间", "里程计 (x, y, yaw)", "真值 (x, y, yaw)", "位置偏差"))
        print("  " + "-" * 72)
        turn_time = (math.pi / 2.0) / self.w
        t_start = rospy.Time.now()
        last = 0.0
        for _lap in range(self.laps):
            for _seg in range(4):
                self._send(self.v, 0.0, self.side / self.v)
                self._report(t_start, last)
                last = (rospy.Time.now() - t_start).to_sec()
                self._send(0.0, self.w, turn_time)
                self._report(t_start, last)
                last = (rospy.Time.now() - t_start).to_sec()
        self._send(0.0, 0.0, 0.3)
        self._report(t_start, last)
        print("  " + "-" * 72)
        err = math.hypot(self.odom[0] - self.truth[0], self.odom[1] - self.truth[1])
        print("  终点位置偏差 %.3f m，航向偏差 %+.1f°"
              % (err, math.degrees(self.odom[2] - self.truth[2])))
        return 0

    def _report(self, t_start, _last):
        if self.odom is None or self.truth is None:
            return
        t = (rospy.Time.now() - t_start).to_sec()
        err = math.hypot(self.odom[0] - self.truth[0], self.odom[1] - self.truth[1])
        print("  %8.1f  (%+6.3f,%+6.3f,%+6.3f)  (%+6.3f,%+6.3f,%+6.3f)  %8.3f m"
              % (t, self.odom[0], self.odom[1], self.odom[2],
                 self.truth[0], self.truth[1], self.truth[2], err))
        sys.stdout.flush()


def main():
    rospy.init_node("demo_drive")
    return DemoDrive().run()


if __name__ == "__main__":
    sys.exit(main())
