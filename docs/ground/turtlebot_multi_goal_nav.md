# 多目标点导航（turtlebot_multi_goal_nav）

## 简介

`turtlebot_multi_goal_nav` 是一个**多目标点顺序导航**模块（无独立激光避障节点），
用于地面移动机器人（如 TurtleBot3）。机器人从当前位置出发，按顺序访问一组目标点，
到达阈值内即切换下一个目标点，全部目标点到达后自动停车；正前方障碍过近时，
内嵌简单避障逻辑（停车原地右转）。

* 源码：`src/ground/turtlebot_multi_goal_nav/`（入口 `main.py`）
* 语言 / 框架：Python3 + ROS 1（`rospy`）
* 话题：订阅 `/odom`（`nav_msgs/Odometry`）、`/scan`（`sensor_msgs/LaserScan`），发布 `/cmd_vel`（`geometry_msgs/Twist`）

## 目标点

室外场景的 5 个目标点，按顺序访问（坐标单位为米）：(1.5, 0.0)、(1.5, 3.0)、
(-2.5, 3.0)、(-2.5, -1.5)、(0.0, 0.0)，最后一个目标点为出发点，形成闭环巡检。

## 工作方式

每个控制周期（10 Hz）：

1. 由 `/odom` 计算当前位姿 `(x, y, yaw)`；
2. 计算到当前目标点的距离与角度误差（角度误差归一化到 `[-π, π]`）；
3. 读取 `/scan` 正前方向（`340:360 + 0:20`）最近有效距离 `min_dist`；
4. 决策：
   * `min_dist < safe_dist`（0.4 m）：**停车，原地右转避障**（内嵌，`angular.z = 0.4`）；
   * 距目标 `< goal_tol`（0.2 m）：**到达，切换到下一目标点并停车**；
   * 角度误差 `> 0.15 rad`：**原地转向**对准目标方向（`angular.z = 0.3 * angle_err`）；
   * 否则：**直行**驶向目标点（`linear.x = 0.2`）；
5. 全部目标点到达后停车并退出。

## 文件结构

```
src/ground/turtlebot_multi_goal_nav/
├── main.py                      # 模块入口
├── package.xml                  # ROS1 catkin 包描述
├── CMakeLists.txt               # 构建 / 安装配置
├── launch/
│   └── turtlebot_multi_goal_nav.launch   # launch 启动文件
└── README.md                    # 运行环境与运行步骤
```

## 运行环境与步骤

运行环境与运行步骤详见模块内 [`README.md`](https://github.com/OpenHUTB/ros2/blob/master/src/ground/turtlebot_multi_goal_nav/README.md)。

快速启动：

```bash
# 1) 编译（若尚未编译本包）
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 启动仿真 / 底盘（提供 /odom、/scan 与 /cmd_vel），以 TurtleBot3 为例
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 3) 启动本模块
roslaunch turtlebot_multi_goal_nav turtlebot_multi_goal_nav.launch
```

## 运行效果

机器人在仿真中依次经过 5 个目标点并自动停车的运行效果（仿真录制动图）：

![turtlebot_multi_goal_nav 运行效果](../img/ground/turtlebot_multi_goal_nav.gif)

## 使用的大模型声明

本模块的 `main.py` 为作业原始代码；`package.xml`、`CMakeLists.txt`、`launch`、
`README.md`、本文档由大模型辅助整理生成，作者已逐项核对并对提交内容负全部责任。
