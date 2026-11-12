#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""无人机起飞—悬停—降落 ROS 节点（模块入口 main.py）。

订阅位姿话题（默认 /uav/pose，geometry_msgs/PoseStamped，ENU），
按状态机计算速度指令并发布到 /uav/cmd_vel（geometry_msgs/Twist），
由桥接节点或仿真器执行。

状态机本身在 flight_state_machine.py 中实现（纯 Python，可单独测试），
本文件只负责 ROS 话题收发、参数读取和异常保护。

运行：
    rosrun uav_takeoff_hover_land main.py
    roslaunch uav_takeoff_hover_land main.launch
"""
from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped, Twist

from flight_state_machine import (
    STATE_DONE, STATE_NAMES, TakeoffHoverLandFSM,
)


class TakeoffHoverLandNode(object):

    def __init__(self):
        self.pose_topic = rospy.get_param('~pose_topic', '/uav/pose')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/uav/cmd_vel')
        self.control_rate = rospy.get_param('~control_rate', 20.0)
        self.pose_timeout = rospy.get_param('~pose_timeout', 1.0)

        self.fsm = TakeoffHoverLandFSM(
            takeoff_altitude=rospy.get_param('~takeoff_altitude', 2.0),
            hover_time=rospy.get_param('~hover_time', 5.0),
            position_tolerance=rospy.get_param('~position_tolerance', 0.15),
            max_horizontal_speed=rospy.get_param('~max_horizontal_speed', 1.0),
            max_vertical_speed=rospy.get_param('~max_vertical_speed', 0.5),
            kp_horizontal=rospy.get_param('~kp_horizontal', 0.8),
            kp_vertical=rospy.get_param('~kp_vertical', 1.0),
            land_altitude=rospy.get_param('~land_altitude', 0.05),
            yaw_rate=rospy.get_param('~yaw_rate', 0.0),
            hover_offset=(rospy.get_param('~hover_x', 1.0),
                          rospy.get_param('~hover_y', 0.5)),
        )

        self.position = None
        self.last_pose_time = None

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.on_pose)
        rospy.loginfo('起飞—悬停—降落节点已启动：pose=%s -> cmd=%s',
                      self.pose_topic, self.cmd_topic)

    def on_pose(self, msg):
        p = msg.pose.position
        self.position = (p.x, p.y, p.z)
        self.last_pose_time = rospy.Time.now()

    def publish_cmd(self, command):
        msg = Twist()
        msg.linear.x = command.vx
        msg.linear.y = command.vy
        msg.linear.z = command.vz
        msg.angular.z = command.yaw_rate
        self.pub.publish(msg)

    def publish_stop(self):
        self.pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(self.control_rate)
        last_state = None
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now

            if self.position is None:
                rate.sleep()
                continue

            # 位姿超时：说明桥接或仿真器断开了，先停住并等待恢复
            if (now - self.last_pose_time).to_sec() > self.pose_timeout:
                self.publish_stop()
                rospy.logwarn_throttle(2.0, '超过 %.1f 秒没有收到位姿，已悬停等待',
                                       self.pose_timeout)
                rate.sleep()
                continue

            if dt <= 0.0:
                rate.sleep()
                continue

            command = self.fsm.update(self.position, dt)
            self.publish_cmd(command)

            if command.state != last_state:
                rospy.loginfo('状态切换：%s -> %s（高度 %.2f m）',
                              STATE_NAMES.get(last_state, '无'),
                              STATE_NAMES.get(command.state, command.state),
                              self.position[2])
                last_state = command.state

            if command.state == STATE_DONE:
                rospy.loginfo('降落完成，飞行结束')
                self.publish_stop()
                break

            rate.sleep()

        self.publish_stop()


def main():
    rospy.init_node('uav_takeoff_hover_land')
    node = TakeoffHoverLandNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.publish_stop()
        rospy.loginfo('节点退出，已发布零速度')


if __name__ == '__main__':
    main()
