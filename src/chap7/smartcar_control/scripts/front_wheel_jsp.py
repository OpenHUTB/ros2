#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('front_wheel_jsp')
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
rate = rospy.Rate(50)

msg = JointState()
msg.name = ['right_front_wheel_to_bridge', 'left_front_wheel_to_bridge']
msg.position = [0.0, 0.0]
msg.velocity = [0.0, 0.0]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
