#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_base 失败时的兜底导航（不是常驻控制器）。

之前这版是每帧 /scan 就直接发 /uav/cmd_vel，和 move_base 抢同一条话题。
现在改成：

1. 只订阅 /move_base/status，当前目标进入失败态（ABORTED/REJECTED/RECALLED/LOST）
   且状态还是新鲜的时候才动手；move_base 正常规划/执行时本节点一个字都不发。
2. 输出话题是 /simple_nav/cmd_vel，不是 /uav/cmd_vel。
   /uav/cmd_vel 只允许 nav_relay 一个节点发布，它在里面做仲裁。
3. 不是随机转向：订阅 /move_base/goal 记住目标点，用 map->base_link 的 TF
   算目标方位，朝目标走，同时用 /scan 前向扇区避障。

订阅：
    /move_base/status   actionlib_msgs/GoalStatusArray   判断是否失败
    /move_base/goal     move_base_msgs/MoveBaseActionGoal 目标点
    /scan               sensor_msgs/LaserScan            避障
发布：
    /simple_nav/cmd_vel geometry_msgs/Twist              仅接管时输出
"""
import math
import os
import sys

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_logic  # noqa: E402


class SimpleNavigator(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.status_timeout = float(rospy.get_param("~status_timeout", 1.0))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.5))
        self.front_half_width = math.radians(float(rospy.get_param("~front_sector_deg", 60.0)) / 2.0)
        self.side_lo = math.radians(float(rospy.get_param("~side_sector_deg", 60.0)))
        self.side_hi = math.radians(float(rospy.get_param("~side_sector_deg", 120.0)))

        self.cfg = {
            "stop_distance": float(rospy.get_param("~stop_distance", 1.0)),
            "turn_rate": float(rospy.get_param("~turn_rate", 0.6)),
            "max_vx": float(rospy.get_param("~max_vx", 0.5)),
            "max_yaw": float(rospy.get_param("~max_yaw", 0.6)),
            "goal_gain": float(rospy.get_param("~goal_gain", 1.0)),
            "align_slow_rad": float(rospy.get_param("~align_slow_rad", 1.2)),
        }

        self.status_code = None
        self.status_stamp = None
        self.goal = None            # (x, y) in map frame
        self.scan = None
        self.scan_stamp = None
        self.active = False

        self.cmd_pub = rospy.Publisher("/simple_nav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_cb, queue_size=1)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("simple_nav 就绪：只在 move_base 失败时接管（输出到 /simple_nav/cmd_vel）")

    # ---------- 回调 ----------
    def status_cb(self, msg):
        if len(msg.status_list) == 0:
            return
        # 按 actionlib 约定，最后一个元素是最近一次下发的目标
        self.status_code = int(msg.status_list[-1].status)
        self.status_stamp = rospy.Time.now()

    def goal_cb(self, msg):
        p = msg.goal.target_pose.pose.position
        self.goal = (float(p.x), float(p.y))

    def scan_cb(self, msg):
        self.scan = msg
        self.scan_stamp = rospy.Time.now()

    # ---------- 决策 ----------
    def _fresh(self, stamp, timeout):
        return stamp is not None and (rospy.Time.now() - stamp).to_sec() <= timeout

    def _should_takeover(self):
        if self.status_stamp is None or not self._fresh(self.status_stamp, self.status_timeout):
            return False
        if not nav_logic.status_is_failed(self.status_code):
            return False
        return self.goal is not None

    def _goal_bearing(self):
        """目标点在 base_link 下的方位角（左正右负），拿不到 TF 就返回 None。"""
        if self.goal is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0),
                                                 rospy.Duration(0.3))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = nav_logic.yaw_from_quat((q.x, q.y, q.z, q.w))
        ang = math.atan2(self.goal[1] - by, self.goal[0] - bx)
        return nav_logic.normalize_angle(ang - yaw)

    def _ranges_and_angles(self):
        s = self.scan
        return s.ranges, s.angle_min, s.angle_increment, s.range_min

    def tick(self, _event):
        takeover = self._should_takeover()

        if not takeover:
            if self.active:
                # 交还控制权前先发一帧零速，避免带着上一帧速度继续飞
                self.cmd_pub.publish(Twist())
                self.active = False
                rospy.loginfo("move_base 已恢复或目标结束，simple_nav 交回控制权")
            return

        if self.scan is None or not self._fresh(self.scan_stamp, self.scan_timeout):
            # 接管了但没有 scan，原地停着更安全
            self.cmd_pub.publish(Twist())
            return

        ranges, angle_min, angle_inc, range_min = self._ranges_and_angles()
        front = nav_logic.front_min_range(ranges, angle_min, angle_inc,
                                          self.front_half_width, range_min)
        left = nav_logic.sector_min_range(ranges, angle_min, angle_inc,
                                          self.side_lo, self.side_hi, range_min)
        right = nav_logic.sector_min_range(ranges, angle_min, angle_inc,
                                           -self.side_hi, -self.side_lo, range_min)
        bearing = self._goal_bearing()

        vx, wz = nav_logic.decide_command(front, left, right, bearing, self.cfg)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)
        self.active = True
        rospy.logwarn_throttle(2.0,
                               "move_base 失败，simple_nav 接管：front=%s vx=%.2f wz=%.2f",
                               "n/a" if front is None else "%.2fm" % front, vx, wz)

    def spin(self):
        while not rospy.is_shutdown():
            self.tick(None)
            self.rate.sleep()


def main():
    rospy.init_node("simple_nav")
    SimpleNavigator().spin()


if __name__ == "__main__":
    main()
