#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('virtual_point')
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)
    t = 0.0
    rospy.loginfo("虚拟小红点启动，打开rviz加Odometry显示就能看到点绕圈移动")
    while not rospy.is_shutdown():
        x = math.cos(t)
        y = math.sin(t)
        theta = t + math.pi/2

        # 广播 odom -> base_link 的TF，让rviz的Fixed Frame有坐标系可用，不再报No TF data
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = Quaternion(*[0, 0, math.sin(theta/2), math.cos(theta/2)])
        tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(*[0, 0, math.sin(theta/2), math.cos(theta/2)])
        pub_odom.publish(odom)
        t += 0.02
        rate.sleep()
