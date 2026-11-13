#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒立摆能量摆起 + LQR 平衡 ROS 节点（模块入口 main.py）。

订阅 `/cartpole/state`（std_msgs/Float64MultiArray，依次为
[x, x_dot, theta, theta_dot]，theta 从竖直向上方向量起），
按「能量摆起 → 接近竖直后切换 LQR 平衡」输出控制力到
`/cartpole/force`（std_msgs/Float64，N，向右为正）。

运行：
    rosrun cartpole_swingup main.py
    roslaunch cartpole_swingup main.launch
"""
from __future__ import print_function

import rospy
from std_msgs.msg import Float64, Float64MultiArray

from cartpole_model import MODE_BALANCE, MODE_SWINGUP, CartPoleController


class CartPoleSwingUpNode(object):

    def __init__(self):
        self.state_topic = rospy.get_param('~state_topic', '/cartpole/state')
        self.force_topic = rospy.get_param('~force_topic', '/cartpole/force')
        self.state_timeout = rospy.get_param('~state_timeout', 0.5)

        self.controller = CartPoleController(
            k_energy=rospy.get_param('~k_energy', 8.0),
            k_cart_p=rospy.get_param('~k_cart_p', 0.8),
            k_cart_d=rospy.get_param('~k_cart_d', 0.6),
            switch_angle=rospy.get_param('~switch_angle', 0.4),
            switch_rate=rospy.get_param('~switch_rate', 2.0),
            fall_angle=rospy.get_param('~fall_angle', 0.7),
        )

        self.state = None
        self.last_state_time = None
        self.last_mode = None

        self.pub = rospy.Publisher(self.force_topic, Float64, queue_size=10)
        rospy.Subscriber(self.state_topic, Float64MultiArray, self.on_state)
        rospy.loginfo('倒立摆摆起节点已启动：state=%s -> force=%s（LQR 增益 %s）',
                      self.state_topic, self.force_topic,
                      [round(v, 3) for v in self.controller.k_lqr])

    def on_state(self, msg):
        if len(msg.data) < 4:
            return
        self.state = [float(v) for v in msg.data[:4]]
        self.last_state_time = rospy.Time.now()

    def publish_force(self, force):
        self.pub.publish(Float64(data=float(force)))

    def run(self):
        rate = rospy.Rate(rospy.get_param('~control_rate', 50.0))
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.state is None or self.last_state_time is None:
                self.publish_force(0.0)
                rate.sleep()
                continue

            if (now - self.last_state_time).to_sec() > self.state_timeout:
                self.publish_force(0.0)
                rospy.logwarn_throttle(2.0, '超过 %.1f 秒没有收到倒立摆状态，控制力已归零',
                                       self.state_timeout)
                rate.sleep()
                continue

            force = self.controller.update(self.state)
            self.publish_force(force)

            if self.controller.mode != self.last_mode:
                rospy.loginfo('控制模式切换：%s', self.controller.mode)
                self.last_mode = self.controller.mode

            rate.sleep()

        self.publish_force(0.0)


def main():
    rospy.init_node('cartpole_swingup')
    node = CartPoleSwingUpNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.publish_force(0.0)
        rospy.loginfo('节点退出，控制力已归零')


if __name__ == '__main__':
    main()
