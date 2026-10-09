#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def gesture_callback(msg):
    gesture = msg.data
    rospy.loginfo(f"收到手势: {gesture}")
    
    # 在这里写对接 AirSim 的代码
    if gesture == "open":
        # 起飞指令
        rospy.loginfo("执行：起飞")
    elif gesture == "fist":
        # 降落指令
        rospy.loginfo("执行：降落")
    elif gesture == "ok":
        # 紧急停止
        rospy.loginfo("执行：紧急停止")

def main():
    rospy.init_node('drone_control_node', anonymous=True)
    rospy.Subscriber('/gesture_command', String, gesture_callback)
    rospy.spin()

if __name__ == '__main__':
    main()