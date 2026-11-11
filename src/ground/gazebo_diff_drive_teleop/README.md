# Gazebo 差速小车键盘控制与里程计闭环精确运动

用单键执行固定距离、固定角度和正方形轨迹，依靠 `/odom` 反馈自动停车。

## 运行环境

- Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic
- 依赖：`ros_gz_sim`、`ros_gz_bridge`、`ros2_control`、`ros2_controllers`、`gz_ros2_control`、`robot_state_publisher`、`xacro`

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-robot-state-publisher ros-jazzy-xacro
```

## 快速运行

```bash
bash main.sh                 # 自动巡航
bash main.sh manual          # 仅启动仿真
bash main.sh precision # 本项目功能
```

无界面环境：

```bash
bash main.sh precision gz_args:='-r -s'
```

## 功能按键

| 按键 | 功能 |
| --- | --- |
| `1` / `2` | 前进 / 后退 0.5 m |
| `3` / `4` | 左转 / 右转 90° |
| `5` | 转 180° |
| `6` | 正方形演示（0.5 m + 90°，重复四次） |
| `0` / `s` / 空格 | 取消当前动作 |
| `+` / `-` | 调整线速度 |

## 实现要点

- `precision_moves.py`：相对距离/角度动作、里程计闭环、正方形动作队列。
- `controller_bridge.py`：把 `Twist /cmd_vel` 转成 `TwistStamped`，并转发 `/odom`。
- `main.sh precision`：后台启动仿真，前台运行键盘节点，退出时清理进程组。

## 实测结果

真实 Gazebo 运行：前进 0.5 m 实测 0.542 m（误差 +0.042 m），左转 90° 实测约 91.0°（误差 +1.0°），正方形闭合误差约 0.044 m。

## 参数与话题

完整参数表、话题/TF、算法公式、复现步骤和故障排查见 `docs/ground/gazebo_diff_drive_teleop.md`。

## 说明

代码与文档使用 AI 大模型辅助编写，已核对并确认，对提交内容负责。
