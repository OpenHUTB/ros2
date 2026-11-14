项目：基于八叉树的快递小车自动避障寻路仿真
环境：Ubuntu20.04 ROS Noetic Gazebo Octomap

1. 打开文件夹方式
图形界面：双击delivery_robot文件夹浏览源码
终端打开：nautilus delivery_robot

2. 安装依赖
sudo apt update
sudo apt install ros-noetic-octomap ros-noetic-octomap-server ros-noetic-octomap-ros ros-noetic-pointcloud-to-laserscan

3. 部署
将delivery_robot放到 ~/delivery_ws/src
cd ~/delivery_ws
catkin_make
source devel/setup.bash
chmod +x src/delivery_robot/scripts/octree_delivery_planner.py

4. 运行仿真
roslaunch delivery_robot delivery_octree.launch

5. RViz可视化八叉树（新开终端）
cd ~/delivery_ws
source devel/setup.bash
rviz
Add Octomap，topic=/octomap

6. 停止程序：Ctrl + C

