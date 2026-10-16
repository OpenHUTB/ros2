#!/usr/bin/env python3
import airsim
import rospy
from std_msgs.msg import String

def gesture_callback(msg):
    gesture = msg.data
    rospy.loginfo(f"收到手势: {gesture}")
    
    # 在这里写对接 AirSim 的代码
    if gesture == "open":
    # 起飞指令
       client = airsim.MultirotorClient()
       client.confirmConnection()
       client.enableApiControl(True)
       client.takeoffAsync().join()
       rospy.loginfo(">>> 无人机已真实起飞！ <<<")
   elif gesture == "fist":
    # 降落指令
       client = airsim.MultirotorClient()
       client.confirmConnection()
       client.enableApiControl(True)
       client.landAsync().join()
       rospy.loginfo(">>> 无人机已真实降落！ <<<")
    elif gesture == "ok":
    # 紧急停止
       client = airsim.MultirotorClient()
       client.confirmConnection()
       client.enableApiControl(True)
       client.armDisarm(False)  # 紧急锁桨
       rospy.loginfo(">>> 执行：紧急停止！ <<<")

def main():
    rospy.init_node('drone_control_node', anonymous=True)
    rospy.Subscriber('/gesture_command', String, gesture_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
