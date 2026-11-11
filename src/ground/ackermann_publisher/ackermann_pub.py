#!/usr/bin/env python3
# ackermann_pub.py
import rospy
from ackermann_msgs.msg import AckermannDrive

if __name__ == "__main__":
    # 1. 初始化节点
    rospy.init_node("simple_ackermann_publisher")
    # 2. 创建发布器：话题名、消息类型、队列长度
    pub = rospy.Publisher("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, queue_size=10)
    # 3. 设置发布频率 10Hz
    rate = rospy.Rate(10)

    rospy.loginfo("Ackermann发布节点启动，开始发送控制指令")
    while not rospy.is_shutdown():
        # 构造阿克曼消息
        drive_msg = AckermannDrive()
        drive_msg.speed = 2.0               # 速度 m/s
        drive_msg.steering_angle = 0.2      # 转向角 rad，正数向右，负数向左
        drive_msg.steering_angle_velocity = 0
        drive_msg.acceleration = 0
        drive_msg.jerk = 0

        pub.publish(drive_msg)
        rospy.loginfo(f"发送: speed={drive_msg.speed}, steer={drive_msg.steering_angle}")
        rate.sleep()
