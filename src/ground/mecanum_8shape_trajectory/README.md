# mecanum_8shape_trajectory

麦克纳姆轮全向移动平台运动学建模与8字轨迹追踪仿真。

## 运行环境

- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11

## 运行步骤

1. 将本包放入 catkin_ws/src/
2. 编译：catkin_make
3. 基础仿真：roslaunch mecanum_8shape_trajectory base_sim.launch
4. 8字追踪：roslaunch mecanum_8shape_trajectory trajectory_track.launch

## 入口

- main.py：直线前进测试入口（符合仓库main开头规范）
