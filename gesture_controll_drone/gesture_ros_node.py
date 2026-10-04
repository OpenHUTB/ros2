#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
# 导入你原来的相机和手势检测代码
import sys
sys.path.append('/home/user/hutb_ws/src/nn/src/gesture_controll_drone')  # 虚拟机的路径，之后改
from camera_utils import Camera
from gesture_detection import GestureDetector  # 假设你的手势识别在这个文件里

def main():
    # 1. 初始化 ROS 节点
    rospy.init_node('gesture_control_node', anonymous=True)
    
    # 2. 创建一个发布者，向外发布手势识别结果
    gesture_pub = rospy.Publisher('/gesture_command', String, queue_size=10)
    
    # 3. 初始化摄像头和手势检测（沿用你今晚跑通的代码）
    camera = Camera()
    camera.initialize()
    detector = GestureDetector()
    
    rate = rospy.Rate(10)  # 10Hz 频率
    
    rospy.loginfo("手势控制节点已启动！")
    
    while not rospy.is_shutdown():
        frame = camera.read_frame()
        if frame is not None:
            # 得到手势识别结果，比如 "open", "fist", "ok"
            gesture_result = detector.detect(frame)  
            
            # 发布给 ROS 系统
            if gesture_result:
                gesture_pub.publish(gesture_result)
                rospy.loginfo(f"发布手势: {gesture_result}")
                
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass