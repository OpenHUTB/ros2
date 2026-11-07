#!/usr/bin/env python3
import os
import rospy

if __name__ == '__main__':
    try:
        rospy.init_node('gazebo_robot_steering_main')
        print("正在启动 Gazebo 3D 环境与 rqt 控制面板...")
        os.system("roslaunch gazebo_robot_steering robot_steering.launch")
    except rospy.ROSInterruptException:
        pass
