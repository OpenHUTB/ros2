# Gazebo 差速小车键盘控制与电量管理

基于 ROS2 Jazzy + Gazebo Harmonic 的差速驱动小车，通过 `gz_ros2_control` + `diff_drive_controller` 把 `/cmd_vel` 指令映射为左右轮转速。在原有键盘遥控和自动巡航之外，本版本增加电量模拟和低电量自动返航。

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

# 电量管理与低电量自动返航
bash main.sh battery

# 仅启动仿真，另开终端运行键盘节点
bash main.sh manual
ros2 run gazebo_diff_drive_teleop teleop_keyboard
```

`bash main.sh battery` 会在后台启动 Gazebo 和控制器，等待 `/odom` 就绪后在前台运行电量管理节点；按 `q` 退出时自动清理仿真进程。

无显示器环境：

```bash
bash main.sh battery gz_args:='-r -s'
```

## 电量管理功能

`battery_teleop` 根据里程计实际速度计算耗电：

- 行驶距离和转向都会消耗电量，静止时有轻微自放电；
- 电量低于 `low_battery_threshold` 后自动返回充电点；
- 到点后进入 `charging` 状态，电量恢复后回到手动模式；
- 可随时手动接管，或按 `h` 重新返航。

| 按键 | 功能 |
| --- | --- |
| `w/x/a/d` 或方向键 | 手动驾驶 |
| `s` / 空格 | 停止；返航中暂停返航 |
| `h` | 手动返回充电点 |
| `b` | 演示用，强制把电量降到阈值以下 |
| `+` / `-` | 调整线速度 |
| `q` / `Ctrl-C` | 退出并自动停车 |

返航控制使用里程计闭环：先转向充电点，再按距离比例前进，接近目标时减速并调整航向。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `/odom` | `nav_msgs/Odometry` | 差速控制器里程计（由 bridge 转发） |
| `/battery_state` | `sensor_msgs/BatteryState` | 模拟电池状态 |
| `/battery_return/status` | `std_msgs/String` | `manual` / `returning` / `charging` |
| `/battery_return/markers` | `visualization_msgs/MarkerArray` | 充电点标记 |

launch 同时桥接 Gazebo `/clock`，并把 `Twist` 速度指令转换为控制器所需的 `TwistStamped`。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `initial_battery` | `100.0` | 初始电量（%） |
| `low_battery_threshold` | `25.0` | 低电量阈值（%） |
| `charged_threshold` | `80.0` | 充电完成阈值（%） |
| `drain_per_meter` | `8.0` | 每米耗电（%） |
| `drain_per_radian` | `2.0` | 每弧度转向耗电（%） |
| `charge_rate` | `8.0` | 充电速率（%/s） |
| `home_x` / `home_y` / `home_yaw` | `0` | 充电点位姿 |

## 验证

```bash
ros2 topic echo /battery_state
ros2 topic echo /battery_return/status
ros2 topic echo /battery_return/markers
```

本次真实 Gazebo 验证：小车驶离充电点约 0.67 m 后按 `b` 强制低电量，节点自动返回充电点，进入 `charging` 状态，电量从 6.3% 开始回升。效果图见 `docs/ground/gazebo_diff_drive_teleop.md`。

## 说明

代码与文档使用 AI 大模型辅助编写，已核对并确认，对提交内容负责。
