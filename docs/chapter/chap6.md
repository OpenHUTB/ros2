# 第6章 机器人 SLAM 与自主导航

本章介绍在 Gazebo Classic 11 中，为一台带 RPLIDAR 激光雷达的 mrobot 移动机器人搭建仿真环境，并完成 SLAM 建图与自主导航。

本章配套完整功能包：

```text
src/chap6/catkin_ws/src/mrobot_gazebo/        # 仿真环境：房间世界、Gazebo 机器人模型与插件
src/chap6/catkin_ws/src/mrobot_description/   # arbotix 假仿真使用的简化机器人 URDF 模型
src/chap6/catkin_ws/src/mrobot_bringup/       # arbotix 假机器人启动配置（精简版）
src/chap6/catkin_ws/src/mrobot_navigation/    # gmapping 与 move_base 启动配置、代价地图、rviz、地图与随机导航脚本
src/chap6/catkin_ws/src/mrobot_teleop/        # 键盘遥控节点（Python3）
```

教程内容：

- 理解 SLAM（同步定位与建图）与激光雷达数据结构 LaserScan；
- 安装 Gazebo 官方模型库；
- 使用 Building Editor 手绘房间并保存为 `.world` 世界；
- 加载带激光雷达的 mrobot 机器人；
- 排查 Noetic 下 xacro 宏前缀、重复包、关节状态等常见问题；
- 使用 gmapping（RBPF 粒子滤波）进行激光 SLAM 建图，并用 map_saver 保存地图；
- 基于 arbotix 假仿真，使用 fake_localization + move_base 实现地图内定位、路径规划、实时避障与自主导航。

## 教程小节

- [6.1 仿真环境搭建](chap6/slam_01_environment_setup.md)
- [6.2 gmapping 激光 SLAM 建图](chap6/slam_02_gmapping.md)
- [6.3 导航功能包与实时避障](chap6/slam_03_navigation.md)
