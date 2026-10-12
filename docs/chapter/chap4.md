# 第4章 机器人仿真

本章介绍如何在 Gazebo Classic 11 中搭建一个可运行的 ROS 移动机器人平台，并完成从模型、传感器到运动控制的首次仿真。

本章配套完整功能包：

```text
src/chap4/mybot_gazebo/
```

教程内容：

- 检查 Ubuntu、ROS Noetic、Gazebo 和插件依赖；
- 创建 Catkin 工作空间并编译 `mybot_gazebo`；
- 使用 URDF/xacro 建立两轮差速机器人；
- 配置差速驱动、里程计、激光雷达和 RGB 相机；
- 编写 world 和 launch 文件；
- 启动 Gazebo 并检查话题、TF 和传感器；
- 使用手动速度命令和自动正方形轨迹验证机器人；
- 调整参数、排查故障，并扩展到 SLAM 和导航。

[进入 Gazebo 机器人平台搭建与首次仿真](chap4/gazebo_platform_simulation.md)
