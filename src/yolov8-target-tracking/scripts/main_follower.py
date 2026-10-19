#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray 

class TargetFollower:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/target_position", Float32MultiArray, self.callback, queue_size=1)
        rospy.loginfo("目标跟随节点启动，正在监听目标位置...")

    def callback(self, msg):
        error_x = msg.data[0]
        error_y = msg.data[1]

        twist = Twist()
        twist.angular.z = -float(error_x) / 300.0
        twist.linear.x = -float(error_y) / 300.0

        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("main_follower")
    node = TargetFollower()
    rospy.spin()
