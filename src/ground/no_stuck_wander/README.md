# no_stuck_wander 防卡死漫游

基于激光雷达（`/scan`）的防卡死漫游模块。机器人正常情况下向前漫游，
当正前方障碍过近时进入「绕障 / 脱困」状态机，避免撞墙或陷入墙角卡死。

## 运行环境

* 操作系统：Ubuntu 20.04 / 22.04
* ROS 版本：ROS 1（Noetic），模块使用 `rospy`
* Python 版本：3.8 / 3.10 / 3.11
* 依赖：`rospy`、`sensor_msgs`、`geometry_msgs`（见 `package.xml`）
* 需要提供话题：`/scan`（`sensor_msgs/LaserScan`）、`/cmd_vel`（`geometry_msgs/Twist`）
  * 推荐使用 TurtleBot3 Gazebo 仿真（自带 `/scan` 与 `/cmd_vel`），或搭载激光雷达的真实底盘

## 运行步骤

1. 将本包放到 catkin 工作空间的 `src` 目录下（本仓库已内置在 `src/ground/no_stuck_wander/`）：

   ```bash
   cd ~/catkin_ws/src
   cp -r <仓库路径>/src/ground/no_stuck_wander ./
   cd ~/catkin_ws && catkin_make
   source ~/catkin_ws/devel/setup.bash
   ```

2. 启动仿真 / 底盘（以 TurtleBot3 为例）：

   ```bash
   export TURTLEBOT3_MODEL=burger
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```

3. 启动本模块（launch 启动）：

   ```bash
   roslaunch no_stuck_wander no_stuck_wander.launch
   ```

   也可直接运行入口 `main.py`：

   ```bash
   rosrun no_stuck_wander main.py
   # 或
   python3 $(rospack find no_stuck_wander)/main.py
   ```

4. 观察机器人：无障碍时直行漫游，遇到障碍自动绕障，极端情况下先直退再原地转，
   脱困后强制前进一小段，避免卡死。

## 运行效果

机器人在仿真中漫游、绕障与脱困的运行效果（Webots 仿真录制）：

![no_stuck_wander 运行效果](../../../docs/img/ground/no_stuck_wander.gif)

## 使用的大模型声明

本模块的 `main.py` 为作业原始代码；`package.xml`、`CMakeLists.txt`、`launch`、
`README.md`、文档页由大模型辅助整理生成，作者已逐项核对并对提交内容负全部责任。
