# TurtleBot3 自主导航

基于 ROS Noetic 的 TurtleBot3 (Burger) Gazebo 仿真自主导航功能包。

## 功能

- map_server 加载静态地图
- AMCL 自适应蒙特卡洛定位
- move_base + DWA 局部规划器导航
- RViz 可视化（2D Pose Estimate / 2D Nav Goal）

## 依赖

- Ubuntu 20.04 + ROS Noetic
- turtlebot3, turtlebot3_gazebo, turtlebot3_bringup
- dwa_local_planner

```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-bringup
sudo apt install ros-noetic-dwa-local-planner
```

## 编译与运行

```bash
cd ~/nav_ws
catkin_make
source devel/setup.bash

# 终端 1：启动 Gazebo 仿真世界
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 终端 2：启动导航
export TURTLEBOT3_MODEL=burger
rosparam set use_sim_time true
roslaunch turtlebot3_auto_nav main.launch
```

## 文件结构

```
turtlebot3_auto_nav/
├── launch/
│   └── main.launch          # 一键启动导航 + RViz
├── maps/
│   ├── map.pgm              # 静态地图（Gazebo turtlebot3_world 场景）
│   └── map.yaml             # 地图元数据
├── param/
│   ├── costmap_common_params.yaml
│   ├── dwa_local_planner_params.yaml
│   ├── global_costmap_params.yaml
│   └── local_costmap_params.yaml
├── rviz/
│   └── turtlebot3_nav.rviz  # RViz 显示配置
├── CMakeLists.txt
└── package.xml
```

详细教程与参数说明见 [docs/ground/turtlebot3_auto_nav.md](../../../docs/ground/turtlebot3_auto_nav.md)。
