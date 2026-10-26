#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64

# 2R mechanical arm parameters
L1 = 0.3
L2 = 0.25

def inverse_kinematics(x, y):
    d = (x*x + y*y - L1*L1 - L2*L2) / (2.0*L1*L2)
    d = max(-1.0, min(1.0, d))
    theta2 = math.acos(d)
    theta1 = math.atan2(y, x) - math.atan2(L2*math.sin(theta2),
                                           L1 + L2*math.cos(theta2))
    return theta1, theta2

def main():
    rospy.init_node("two_dof_arm_circle")
    pub1 = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=10)
    pub2 = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=10)
    rospy.sleep(1)

    cx, cy = 0.38, 0.20
    r, omega = 0.08, 0.5

    rate = rospy.Rate(50)
    t = 0.0
    while not rospy.is_shutdown():
        x = cx + r*math.cos(omega*t)
        y = cy + r*math.sin(omega*t)
        th1, th2 = inverse_kinematics(x, y)
        pub1.publish(th1)
        pub2.publish(th2)
        t += 0.02
        rate.sleep()

if __name__ == "__main__":
    main()
