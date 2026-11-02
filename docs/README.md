# 基于Gazebo+ROS移动机器人仿真实验

## 1. 项目概述
本项目基于ROS机器人操作系统与Gazebo物理仿真平台，以TurtleBot3移动机器人为载体，在自定义六边形迷宫环境中完成两大功能：
1. 利用激光雷达获取环境距离信息，实现机器人自主障碍物检测与避障导航。
2. 基于Gmapping算法，依靠激光雷达数据完成SLAM同步定位与二维栅格地图构建，在RViz中可视化地图。

本项目在Ubuntu20.04 + ROS Noetic虚拟机环境开发，使用Gazebo作为物理仿真器，RViz作为数据可视化工具。

## 2. 实验环境
- 操作系统：Ubuntu 20.04
- ROS版本：ROS Noetic
- 仿真平台：Gazebo 11
- 可视化工具：RViz
- 机器人模型：TurtleBot3（burger）
- 依赖功能包：`turtlebot3`、`gmapping`、`rospy`、`geometry_msgs`、`sensor_msgs`

安装依赖命令：
```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-gmapping ros-noetic-rviz ros-noetic-gazebo-ros-pkgs

###
``` 
## 3. 核心代码实现
 
### 3.1 激光雷达自主避障节点 laser_obstacle_avoid.py
``` python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('laser_obstacle_avoid_node')
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.twist_msg = Twist()
        self.rate = rospy.Rate(10)
        self.safe_distance = 0.6

    def laser_callback(self, scan_data):
        # 读取机器人正前方激光测距数据
        front_data = scan_data.ranges[340:360] + scan_data.ranges[0:20]
        front_min_dist = min(front_data)

        if front_min_dist < self.safe_distance:
            # 前方检测到障碍物，停止前进并右转
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.4
        else:
            # 无障碍物，直行
            self.twist_msg.linear.x = 0.2
            self.twist_msg.angular.z = 0.0
        self.vel_pub.publish(self.twist_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = ObstacleAvoidance()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序结束")
```
### 3.2 迷宫场景启动文件 maze_world.launch
``` python
<?xml version="1.0"?>
<launch>
  <!-- 设置TurtleBot3模型 -->
  <param name="turtlebot3_model" value="burger"/>

  <!-- 启动Gazebo，加载自定义迷宫世界 -->
  <include file="$(find gazebo_ros)/launch/gazebo.launch">
    <arg name="world_name" value="$(find your_package)/worlds/maze.world"/>
  </include>

  <!-- 在Gazebo中生成TurtleBot3机器人 -->
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_turtlebot3" args="-urdf -model turtlebot3_burger -param robot_description" />
</launch>
```
### 3.3 Gmapping SLAM建图启动文件 gmapping_slam.launch
``` python
<?xml version="1.0"?>
<launch>
  <!-- 启动gmapping SLAM节点 -->
  <node pkg="gmapping" type="slam_gmapping" name="gmapping" output="screen">
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="odom"/>
    <param name="map_frame" value="map"/>
    <param name="scan_topic" value="scan"/>
    <param name="maxUrange" value="5.0"/>
    <param name="minimumRange" value="0.1"/>
  </node>
  <!-- 启动RViz可视化 -->
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find your_package)/rviz/gmapping.rviz"/>
</launch>
```
    
### 4. 完整运行步骤
 
打开4个独立终端，依次执行
 
1.终端1；设置机器人模型，启动Gazebo迷宫仿真

export TURTLEBOT3_MODEL=burger

roslaunch your_package maze_world.launch

2. 终端2：运行自主避障Python程序

cd catkin_ws && source devel/setup.bash

rosrun your_package laser_obstacle_avoid.py

3. 终端3：启动Gmapping SLAM建图

roslaunch your_package gmapping_slam.launch

4. 操作说明：
机器人自动在迷宫移动，依靠激光雷达避开绿色障碍物；
RViz窗口实时显示激光点云与正在构建的二维栅格地图；
探索完成后，可以执行下面命令保存地图：

rosrun map_server map_saver -f maze_map

### 效果图：
https://github.com/Xavier-101666/ros2/commit/08f3c44d69146a1651cf44074d43aff4aef345e3
