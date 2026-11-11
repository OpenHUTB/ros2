# Gazebo 差速小车键盘控制与示教回放

基于 ROS2 Jazzy + Gazebo Harmonic 的差速驱动小车：通过 `gz_ros2_control` + `diff_drive_controller` 把 `/cmd_vel` 指令映射为左右轮转速，并提供键盘遥控、自动巡航和“示教一次、自动回放”三种运行方式。

本模块同时适配 Jazzy 的控制器接口：launch 会桥接 Gazebo `/clock`，并把标准 `geometry_msgs/Twist` 转换为控制器使用的 `TwistStamped`，因此 `teleop_keyboard`、`auto_drive` 和 `teach_replay` 都使用统一的 `/cmd_vel`、`/odom` 接口。

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

# 示教录制与自动回放
bash main.sh teach

# 仅启动仿真，另开终端运行键盘节点
bash main.sh manual
ros2 run gazebo_diff_drive_teleop teleop_keyboard
```

`bash main.sh teach` 会在后台启动 Gazebo 和控制器，等待 `/odom` 就绪后在前台运行示教节点；按 `q` 退出时会自动清理仿真进程。

无显示器环境可以只启动 Gazebo 服务端：

```bash
bash main.sh teach gz_args:='-r -s'
```

## 键盘遥控

`teleop_keyboard` 的按键：

| 按键 | 功能 |
| --- | --- |
| `w` / `↑` | 前进 |
| `x` / `↓` | 后退 |
| `a` / `←` | 左转 |
| `d` / `→` | 右转 |
| `s` / 空格 | 停止 |
| `+` / `-` | 调整线速度 |
| `q` / `Ctrl-C` | 退出并自动停车 |

## 示教录制与回放

`teach_replay` 先用键盘把小车开一遍，节点记录 `/odom` 轨迹；按 `p` 后使用纯追踪控制器沿原路返回，支持循环往返和手动接管。

| 按键 | 功能 |
| --- | --- |
| `w/x/a/d` 或方向键 | 手动驾驶 |
| `r` | 开始 / 结束录制（重新开始会清空旧轨迹） |
| `p` | 开始 / 暂停回放 |
| `l` | 循环往返开关 |
| `c` | 清空轨迹 |
| `s` / 空格 | 停止；回放中暂停回放 |
| `+` / `-` | 调整线速度 |
| `q` / `Ctrl-C` | 退出并自动停车 |

推荐流程：按 `r` → 开一段折线或 S 形 → 按 `r` 结束 → 按 `p`，小车会从较近的端点沿示教路径开回另一端。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 键盘、自动巡航或回放的速度指令 |
| `/odom` | `nav_msgs/Odometry` | 差速控制器里程计（由 bridge 转发） |
| `/teach_replay/path` | `nav_msgs/Path` | 示教路径，可在 RViz2 中添加 Path 显示 |
| `/teach_replay/markers` | `visualization_msgs/MarkerArray` | 起点、终点和前视目标点 |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | 控制器原生速度输入（由 bridge 写入） |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | 控制器原生里程计（由 bridge 转发） |

## 验证

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic hz /odom
ros2 topic echo /teach_replay/path --once
```

`teach_replay` 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `linear_speed` | `0.3` | 最大线速度（m/s） |
| `angular_speed` | `1.0` | 最大角速度（rad/s） |
| `record_min_distance` | `0.05` | 轨迹采样最小间距（m） |
| `record_min_angle` | `0.10` | 轨迹采样最小转角（rad） |
| `lookahead_distance` | `0.25` | 纯追踪前视距离（m） |
| `goal_tolerance` | `0.12` | 端点判定阈值（m） |

本次真实 Gazebo 运行结果：示教路径 39 个点，回放约 10.04 s，终点距起点 0.110 m（阈值 0.12 m）。详细说明见 `docs/ground/gazebo_diff_drive_teleop.md`。

## 说明

代码与文档使用 AI 大模型辅助编写，已核对并确认，对提交内容负责。
