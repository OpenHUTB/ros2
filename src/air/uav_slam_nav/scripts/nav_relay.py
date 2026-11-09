#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""速度仲裁与转发：唯一向 /uav/cmd_vel 发布速度的节点。

move_base 的 /cmd_vel 和兜底节点的 /simple_nav/cmd_vel 都进到这里，
由本节点选一个转发，再统一做限幅、把 linear.z 清零。
这样 /uav/cmd_vel 只有一个发布者，不会出现两个节点抢控制权。

优先级：兜底（只在它持续输出时）> move_base。
两边都超时就不发指令，交给 bridge 自己的超时悬停。

订阅：
    /cmd_vel             geometry_msgs/Twist   move_base 输出
    /simple_nav/cmd_vel  geometry_msgs/Twist   兜底节点输出（仅接管时）
发布：
    /uav/cmd_vel         geometry_msgs/Twist   桥接包输入

参数（launch 覆盖）：
    ~max_vx        最大线速度 x，默认 0.8 m/s
    ~max_vy        最大线速度 y，默认 0.5 m/s
    ~max_yaw       最大角速度 z，默认 0.6 rad/s
    ~stale_timeout 指令多久没更新就认为失效，默认 0.5 s
    ~rate          转发频率，默认 20 Hz
"""
import os
import sys

import rospy
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_logic  # noqa: E402


class NavRelay(object):
    def __init__(self):
        self.max_vx = float(rospy.get_param("~max_vx", 0.8))
        self.max_vy = float(rospy.get_param("~max_vy", 0.5))
        self.max_yaw = float(rospy.get_param("~max_yaw", 0.6))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 0.5))
        rate_hz = float(rospy.get_param("~rate", 20.0))

        self.mb_cmd = None
        self.mb_stamp = None
        self.fb_cmd = None
        self.fb_stamp = None

        self.pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.mb_cb, queue_size=1)
        rospy.Subscriber("/simple_nav/cmd_vel", Twist, self.fb_cb, queue_size=1)
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("nav_relay 启动：/uav/cmd_vel 由本节点唯一发布")

    def mb_cb(self, msg):
        self.mb_cmd = msg
        self.mb_stamp = rospy.Time.now()

    def fb_cb(self, msg):
        self.fb_cmd = msg
        self.fb_stamp = rospy.Time.now()

    def tick(self, _event=None):
        now = rospy.Time.now()
        src = nav_logic.select_source(now.to_sec(),
                                      None if self.mb_stamp is None else self.mb_stamp.to_sec(),
                                      None if self.fb_stamp is None else self.fb_stamp.to_sec(),
                                      self.stale_timeout)
        if src is None:
            # 没有新鲜指令，安静等 bridge 的超时悬停
            return

        msg = self.fb_cmd if src == "fallback" else self.mb_cmd
        vx, vy, vz, wz = nav_logic.limit_twist(msg.linear.x, msg.linear.y, msg.angular.z,
                                               self.max_vx, self.max_vy, self.max_yaw)
        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.linear.z = vz          # 恒为 0：不主动升降，定高由 bridge 负责
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = wz
        self.pub.publish(out)

    def spin(self):
        while not rospy.is_shutdown():
            self.tick()
            self.rate.sleep()


def main():
    rospy.init_node("nav_relay")
    NavRelay().spin()


if __name__ == "__main__":
    main()
