#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
import tf2_ros
class OdomPublisher:
    def __init__(self):
        self.model_name = rospy.get_param('~model_name', 'mecanum_robot')
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb)
        rospy.loginfo("里程计发布节点启动")
    def cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        twist = msg.twist[idx]
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose
        odom.twist.twist = twist
        self.pub_odom.publish(odom)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)
if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    node = OdomPublisher()
    rospy.spin()
