#!/usr/bin/env python3
import subprocess

if __name__ == '__main__':
    print("正在启动 Gazebo 3D 环境与 rqt 控制面板...")
    try:
        subprocess.call(["roslaunch", "gazebo_robot_steering", "robot_steering.launch"])
    except KeyboardInterrupt:
        pass
