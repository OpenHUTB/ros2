#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
if __name__ == '__main__':
    rospy.init_node('mecanum_ctrl')
    pub = rospy.Publisher('/mecanum_group_controller/command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(50)
    vx = 0.3
    vy = 0.0
    wz = 0.0
    while not rospy.is_shutdown():
        fl =  vx - vy - wz
        fr =  vx + vy + wz
        rl =  vx + vy - wz
        rr =  vx - vy + wz
        msg = Float64MultiArray()
        msg.data = [fl, fr, rl, rr]
        pub.publish(msg)
        rate.sleep()
