#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

if __name__ == '__main__':
    rospy.init_node('virtual_point')
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
    rate = rospy.Rate(10)
    t = 0.0
    rospy.loginfo("虚拟小红点启动，打开rviz加Odometry显示就能看到点绕圈移动")
    while not rospy.is_shutdown():
        x = math.cos(t)
        y = math.sin(t)
        theta = t + math.pi/2
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(*[0,0,math.sin(theta/2), math.cos(theta/2)])
        pub_odom.publish(odom)
        t += 0.02
        rate.sleep()
