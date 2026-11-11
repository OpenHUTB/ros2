# Gazebo 差速小车键盘控制与精确运动

基于 ROS2 Jazzy + Gazebo Harmonic 的差速驱动小车，通过 `gz_ros2_control` + `diff_drive_controller` 把 `/cmd_vel` 指令映射为左右轮转速。在原有键盘遥控和自动巡航之外，本版本增加基于里程计的相对精确运动。

## 运行环境 / Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic（`gz sim`）
- 依赖：`ros_gz`、`ros_gz_bridge`、`ros2_control`、`ros2_controllers`、`gz_ros2_control`、`robot_state_publisher`、`xacro`

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-robot-state-publisher ros-jazzy-xacro
```

## 一键运行

```bash
# 自动巡航演示（默认）
bash main.sh

# 里程计闭环精确运动
bash main.sh precision

# 仅启动仿真，另开终端运行键盘节点
bash main.sh manual
ros2 run gazebo_diff_drive_teleop teleop_keyboard
```

`bash main.sh precision` 会在后台启动 Gazebo 和控制器，等待 `/odom` 就绪后在前台运行精确运动节点；按 `q` 退出时自动清理仿真进程。

无显示器环境：

```bash
bash main.sh precision gz_args:='-r -s'
```

## 精确运动功能

`precision_moves` 用里程计闭环完成固定的相对运动，不依赖外部定位或导航栈：

| 按键 | 功能 |
| --- | --- |
| `1` / `2` | 前进 / 后退 0.5 m |
| `3` / `4` | 左转 / 右转 90° |
| `5` | 转 180° |
| `6` | 正方形轨迹演示（0.5 m + 90°，重复四次） |
| `0` / `s` / 空格 | 取消当前动作 |
| `w/x/a/d` 或方向键 | 手动驾驶 |
| `q` / `Ctrl-C` | 退出并自动停车 |

直线运动使用投影到起始航向的里程计位移作为反馈，接近目标时减速；转向运动累加里程计偏航角并做比例控制，误差进入容差后停车。连续动作通过队列依次执行。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `/odom` | `nav_msgs/Odometry` | 差速控制器里程计（由 bridge 转发） |
| `/precision_moves/status` | `std_msgs/String` | 当前动作状态 |
| `/precision_moves/markers` | `visualization_msgs/MarkerArray` | 动作方向标记 |

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `distance_step` | `0.5` | 单次相对移动距离（m） |
| `turn_angle` | `90.0` | 单次相对转向角度（度） |
| `distance_tolerance` | `0.01` | 直线停车容差（m） |
| `angle_tolerance` | `0.02` | 转向停车容差（rad） |
| `linear_gain` | `1.8` | 直线比例增益 |
| `angular_gain` | `2.2` | 转向比例增益 |
| `linear_speed` | `0.25` | 最大线速度（m/s） |
| `angular_speed` | `1.0` | 最大角速度（rad/s） |

## 验证

```bash
ros2 topic echo /precision_moves/status
ros2 topic echo /odom
```

本次真实 Gazebo 测试：前进 0.5 m 实际约 0.542 m（误差 +0.042 m），左转 90° 实际约 91.0°（误差 +1.0°），四边正方形闭合误差约 0.044 m。效果图见 `docs/ground/gazebo_diff_drive_teleop.md`。

## 说明

代码与文档使用 AI 大模型辅助编写，已核对并确认，对提交内容负责。
