#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""让 mybot 在 Gazebo 中按近似正方形轨迹运动。"""

import math

import rospy
from geometry_msgs.msg import Twist


class SquareMotion:
    def __init__(self):
        rospy.init_node("mybot_move_square")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(20)
        self.linear_speed = 0.35
        self.angular_speed = 0.8

    def stop(self):
        self.pub.publish(Twist())

    def publish_for(self, linear_x, angular_z, duration):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.pub.publish(cmd)
            self.rate.sleep()
        self.stop()

    def run(self):
        rospy.sleep(1.0)
        side_time = 2.0 / self.linear_speed
        turn_time = (math.pi / 2.0) / self.angular_speed

        rospy.loginfo("开始走正方形：每边约 2.0 m，共 4 条边")
        for index in range(4):
            rospy.loginfo("第 %d 条边", index + 1)
            self.publish_for(self.linear_speed, 0.0, side_time)
            rospy.sleep(0.4)
            self.publish_for(0.0, self.angular_speed, turn_time)
            rospy.sleep(0.4)

        self.stop()
        rospy.loginfo("正方形轨迹完成")


if __name__ == "__main__":
    node = SquareMotion()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.stop()
