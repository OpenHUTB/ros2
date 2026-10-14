# turtlebot_multi_goal_nav 多目标点导航

**多目标点顺序导航**模块（不含激光避障节点）。地面移动机器人（TurtleBot3 等）
从当前位置出发，依次访问一组目标点，到达阈值内即切换下一个目标点，
全部目标点到达后自动停车。

## 运行环境

* 操作系统：Ubuntu 20.04 / 22.04
* ROS 版本：ROS 1（Noetic），模块使用 `rospy`
* Python 版本：3.8 / 3.10 / 3.11
* 依赖：`rospy`、`nav_msgs`、`geometry_msgs`、`tf`（见 `package.xml`）
* 需要提供话题：`/odom`（`nav_msgs/Odometry`）、`/cmd_vel`（`geometry_msgs/Twist`）
  * 推荐使用 TurtleBot3 Gazebo 仿真（自带 `/odom` 与 `/cmd_vel`），或搭载里程计的真实底盘

## 目标点

室外场景的 5 个目标点，按顺序访问（坐标单位为米）：

| 序号 | 目标点 (x, y) |
| --- | --- |
| 1 | (1.5, 0.0) |
| 2 | (1.5, 3.0) |
| 3 | (-2.5, 3.0) |
| 4 | (-2.5, -1.5) |
| 5 | (0.0, 0.0) |

## 运行步骤

1. 将本包放到 catkin 工作空间的 `src` 目录下（本仓库已内置在 `src/ground/turtlebot_multi_goal_nav/`）：

   ```bash
   cd ~/catkin_ws/src
   cp -r <仓库路径>/src/ground/turtlebot_multi_goal_nav ./
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
   roslaunch turtlebot_multi_goal_nav turtlebot_multi_goal_nav.launch
   ```

   也可直接运行入口 `main.py`：

   ```bash
   rosrun turtlebot_multi_goal_nav main.py
   # 或
   python3 $(rospack find turtlebot_multi_goal_nav)/main.py
   ```

4. 观察机器人：依次经过 5 个目标点，全部到达后自动停车。

## 运行效果

> 运行效果截图待补充：请将你的真实运行截图（png，<3 MB）放置到
> `docs/img/ground/turtlebot_multi_goal_nav.png` 后，更新下方图片链接。

![turtlebot_multi_goal_nav 运行效果](../../../docs/img/ground/turtlebot_multi_goal_nav.png)

## 使用的大模型声明

本模块的 `main.py` 为作业原始代码；`package.xml`、`CMakeLists.txt`、`launch`、
`README.md`、文档页由大模型辅助整理生成，作者已逐项核对并对提交内容负全部责任。
