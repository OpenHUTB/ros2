# 第6章 机器人 SLAM 与自主导航

本章介绍在 Gazebo Classic 11 中，为一台带 RPLIDAR 激光雷达的 mrobot 移动机器人搭建仿真环境，并完成 SLAM 建图与自主导航。

本章配套完整功能包：

```text
src/chap6/catkin_ws/src/mrobot_gazebo/
```

教程内容：

- 理解 SLAM（同步定位与建图）与激光雷达数据结构 LaserScan；
- 安装 Gazebo 官方模型库；
- 使用 Building Editor 手绘房间并保存为 `.world` 世界；
- 加载带激光雷达的 mrobot 机器人；
- 排查 Noetic 下 xacro 宏前缀、重复包、关节状态等常见问题；
- （后续）使用 gmapping / Hector / Cartographer 进行 SLAM 建图；
- （后续）使用 AMCL + move_base 实现地图内自主导航。

## 教程小节

- [6.1 仿真环境搭建](chap6/slam_01_environment_setup.md)
