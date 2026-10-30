#!/usr/bin/env python3
"""ROS Noetic 超车变道仿真节点（演示用）。

订阅：
    /lead_distance  (std_msgs/Float64) 前车纵向位置（m，以本车起点为 0）
    /lead_speed     (std_msgs/Float64) 前车速度（m/s）
发布：
    /cmd_vel        (geometry_msgs/Twist) linear.x=纵向速度，angular.z=横向速度

运行：
    rosrun overtake main.py
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from vehicle_model import VehicleModel
from overtake_controller import OvertakeController


class OvertakeNode:
    def __init__(self):
        rospy.init_node('overtake_node')
        self.host = VehicleModel(x=0.0, y=0.0, v=15.0)
        self.ctrl = OvertakeController()
        self.lead_x = 80.0
        self.lead_v = 8.0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/lead_distance', Float64, self.cb_distance)
        rospy.Subscriber('/lead_speed', Float64, self.cb_speed)
        self.rate = rospy.Rate(20)

    def cb_distance(self, msg):
        self.lead_x = msg.data

    def cb_speed(self, msg):
        self.lead_v = msg.data

    def run(self):
        rospy.loginfo('overtake_node started')
        while not rospy.is_shutdown():
            a, vy, state, info = self.ctrl.compute(
                self.host.x, self.host.y, self.host.v,
                self.lead_x, self.lead_v, 1.0 / 20.0)
            self.host.update(a, vy, 1.0 / 20.0)

            cmd = Twist()
            cmd.linear.x = self.host.v
            cmd.angular.z = vy
            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(1.0,
                f"state={state} v={self.host.v:.2f} y={self.host.y:.2f}")
            self.rate.sleep()


if __name__ == '__main__':
    try:
        OvertakeNode().run()
    except rospy.ROSInterruptException:
        pass
