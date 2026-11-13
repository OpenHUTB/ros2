#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROV 推力分配 ROS 节点（模块入口 main.py）。

订阅期望力/力矩 `/rov/wrench`（geometry_msgs/Wrench，机体坐标系），
用 allocation.py 的伪逆分配算法把六维 wrench 分配到各推进器，
发布：

    /rov/thruster_cmd      std_msgs/Float64MultiArray   各推进器推力（N）
    /rov/wrench_achieved   geometry_msgs/Wrench         实际可达的 wrench

支持推进器故障（参数 ~failed_thruster）和两种饱和策略（~saturation_mode）：
`scale` 按比例缩小、保持合力方向；`clamp` 直接限幅。

运行：
    rosrun rov_thruster_allocation_ros1 main.py
    roslaunch rov_thruster_allocation_ros1 main.launch
"""
from __future__ import print_function

import rospy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray

from allocation import THRUSTERS, allocate


class ThrusterAllocationNode(object):

    def __init__(self):
        self.wrench_topic = rospy.get_param('~wrench_topic', '/rov/wrench')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/rov/thruster_cmd')
        self.achieved_topic = rospy.get_param('~achieved_topic', '/rov/wrench_achieved')
        self.max_thrust = rospy.get_param('~max_thrust', 40.0)
        self.saturation_mode = rospy.get_param('~saturation_mode', 'scale')
        failed = int(rospy.get_param('~failed_thruster', -1))
        self.failed = None if failed < 0 else failed
        self.wrench_timeout = rospy.get_param('~wrench_timeout', 0.5)

        if self.failed is not None and not (0 <= self.failed < len(THRUSTERS)):
            rospy.logwarn('failed_thruster=%d 超出范围，按无故障处理', self.failed)
            self.failed = None

        self.wrench = None
        self.last_wrench_time = None
        self.was_saturated = False

        self.pub_cmd = rospy.Publisher(self.cmd_topic, Float64MultiArray, queue_size=10)
        self.pub_achieved = rospy.Publisher(self.achieved_topic, Wrench, queue_size=10)
        rospy.Subscriber(self.wrench_topic, Wrench, self.on_wrench)

        rospy.loginfo('推力分配节点已启动：wrench=%s -> cmd=%s，max_thrust=%.1f N，模式=%s',
                      self.wrench_topic, self.cmd_topic, self.max_thrust,
                      self.saturation_mode)
        if self.failed is not None:
            rospy.logwarn('推进器 %s 已标记为故障，其余推进器重新分配',
                          THRUSTERS[self.failed]['name'])

    def on_wrench(self, msg):
        self.wrench = [msg.force.x, msg.force.y, msg.force.z,
                       msg.torque.x, msg.torque.y, msg.torque.z]
        self.last_wrench_time = rospy.Time.now()

    def publish_cmd(self, commands):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in commands]
        self.pub_cmd.publish(msg)

    def publish_achieved(self, achieved):
        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = achieved[0], achieved[1], achieved[2]
        msg.torque.x, msg.torque.y, msg.torque.z = achieved[3], achieved[4], achieved[5]
        self.pub_achieved.publish(msg)

    def publish_idle(self):
        self.publish_cmd([0.0] * len(THRUSTERS))
        self.publish_achieved([0.0] * 6)

    def run(self):
        rate = rospy.Rate(rospy.get_param('~control_rate', 20.0))
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.wrench is None or self.last_wrench_time is None:
                self.publish_idle()
                rate.sleep()
                continue

            if (now - self.last_wrench_time).to_sec() > self.wrench_timeout:
                self.publish_idle()
                rospy.logwarn_throttle(2.0, '超过 %.1f 秒没有收到期望 wrench，推进器已归零',
                                       self.wrench_timeout)
                rate.sleep()
                continue

            result = allocate(self.wrench, max_thrust=self.max_thrust,
                              failed=self.failed, mode=self.saturation_mode)
            self.publish_cmd(result['commands'])
            self.publish_achieved(result['achieved'])

            if result['saturated'] != self.was_saturated:
                if result['saturated']:
                    if self.saturation_mode == 'scale':
                        rospy.logwarn('推进器饱和，期望 wrench 按 %.2f 比例缩小',
                                      result['scale'])
                    else:
                        rospy.logwarn('推进器饱和，已按最大推力限幅')
                self.was_saturated = result['saturated']

            rate.sleep()

        self.publish_idle()


def main():
    rospy.init_node('rov_thruster_allocation_ros1')
    node = ThrusterAllocationNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.publish_idle()
        rospy.loginfo('节点退出，推进器指令已归零')


if __name__ == '__main__':
    main()
