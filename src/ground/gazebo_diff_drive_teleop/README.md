# Gazebo 差速小车键盘控制 / Gazebo Differential-Drive Robot Keyboard Teleop

基于 ROS2 Jazzy + Gazebo Harmonic 实现差速驱动小车：通过 `gz_ros2_control` + `diff_drive_controller` 把 `/cmd_vel` 速度指令映射为左右轮转速，实现小车前进、后退、转向，并提供键盘遥控与自动巡航两种运行方式。

A differential-drive robot in ROS2 Jazzy + Gazebo Harmonic: `gz_ros2_control` + `diff_drive_controller` maps `/cmd_vel` commands to left/right wheel speeds. Two modes are provided — keyboard teleop and auto-cruise.

## 运行环境 / Environment
- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic（`gz sim`）
- 依赖：ros_gz、ros2_control、ros2_controllers、gz_ros2_control、robot_state_publisher、xacro

## 运行步骤 / Usage
```bash
# 1. 安装依赖
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-robot-state-publisher ros-jazzy-xacro

# 2. 将本模块放入 ROS2 工作空间 src 下（例如 ~/ros2_ws/src/homework/.../）

# 3. 一键运行整个模块（自动巡航演示，小车沿圆形轨迹行驶）
bash main.sh

# 4. 手动键盘控制模式
bash main.sh manual   # 仅启动仿真
# 另开一个终端运行键盘节点：
ros2 run gazebo_diff_drive_teleop teleop_keyboard
```

键盘按键 / Keys：`w/x` 前进/后退，`a/d` 左转/右转，`s` 停止，`+/-` 调整线速度，`q` 退出。

## 验证 / Verification
```bash
ros2 topic list          # 查看 /cmd_vel /odom /tf 等话题
ros2 topic info /cmd_vel # 类型为 geometry_msgs/Twist
ros2 topic echo /odom    # 查看里程计数据
```

> 若小车不响应键盘，先 `ros2 topic list` 看控制器实际订阅的 cmd_vel 话题名；若为 `/diff_drive_controller/cmd_vel`，则运行时加 remap：`ros2 run gazebo_diff_drive_teleop teleop_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel`。

## 说明 / Note
代码与文档使用 AI 大模型辅助编写，已核对并确认，对提交内容负责。
