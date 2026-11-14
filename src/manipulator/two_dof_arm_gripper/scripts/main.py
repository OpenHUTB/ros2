#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node("arm_push_block")
    pub1 = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=10)
    pub2 = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=10)
    rospy.sleep(1)
    rate = rospy.Rate(50)

    print("Step 1: arm straight, facing +x")
    for _ in range(100):
        pub1.publish(0.0)
        pub2.publish(0.0)
        rate.sleep()
    rospy.sleep(1)

    print("Step 2: rotate clockwise (push block)")
    for i in range(300):
        angle1 = 0.0 - 1.0 * i / 300.0
        pub1.publish(angle1)
        pub2.publish(0.0)
        rate.sleep()
    rospy.sleep(1)

    print("Step 3: rotate back")
    for i in range(150):
        angle1 = -1.0 + 1.0 * i / 150.0
        pub1.publish(angle1)
        pub2.publish(0.0)
        rate.sleep()
    rospy.sleep(1)
    print("Done!")

    while not rospy.is_shutdown():
        pub1.publish(0.0)
        pub2.publish(0.0)
        rate.sleep()

if __name__ == "__main__":
    main()
