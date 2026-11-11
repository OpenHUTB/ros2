# Gazebo 差速小车键盘控制与电量管理/低电量返航

## 1. 项目概述

本项目在 ROS 2 Jazzy + Gazebo Harmonic 中搭建两轮差速驱动小车，使用 `gz_ros2_control` 作为 Gazebo 与 `ros2_control` 之间的硬件接口，使用 `diff_drive_controller` 把速度指令换算成左右轮转速。

模块包含以下运行方式：

- **自动巡航**：`auto_drive` 持续发布固定速度，演示差速小车的圆弧运动。
- **键盘遥控**：`teleop_keyboard` 发布 `/cmd_vel`，手动驾驶小车。
- **电量管理与低电量自动返航**：模拟电池消耗，电量低于阈值后自动返回充电点，到点进入充电状态，电量恢复后回到手动模式。

本项目的重点是电池能量模型、低电量状态机和基于里程计的返航控制。所有实验都使用真实的 Gazebo 物理仿真和 `/odom` 里程计数据，不使用离线伪造轨迹。

## 2. 差速运动学基础

设左右轮半径均为 `r = 0.06 m`，轮距 `L = 0.36 m`，左右轮角速度为 `ωL`、`ωR`，则车体线速度 `v` 和角速度 `ω` 为：

```text
v = r (ωL + ωR) / 2
ω = r (ωR - ωL) / L
```

反过来，给定 `/cmd_vel` 的 `linear.x = v` 和 `angular.z = ω`，控制器计算左右轮目标转速：

```text
ωL = (v - ω L / 2) / r
ωR = (v + ω L / 2) / r
```

因此：

- `v > 0, ω = 0`：直行；
- `v < 0, ω = 0`：倒车；
- `v = 0, ω > 0`：原地左转；
- `v > 0, ω > 0`：左转弯。

差速底盘不能直接横移，转弯半径由 `R = v / ω` 决定。

## 3. 系统架构与控制链路

```text
键盘
  │
  ▼
功能节点（teleop_keyboard / auto_drive / battery_teleop）
  │  geometry_msgs/Twist
  ▼
/cmd_vel
  │
  ▼
controller_bridge
  │  geometry_msgs/TwistStamped
  ▼
/diff_drive_controller/cmd_vel
  │
  ▼
diff_drive_controller ──▶ gz_ros2_control ──▶ Gazebo 差速小车
  │
  └── /diff_drive_controller/odom ──▶ controller_bridge ──▶ /odom
```

Gazebo Harmonic 的 `/clock` 通过 `ros_gz_bridge` 桥接到 ROS 2。控制器启用了 `use_sim_time`，如果没有 `/clock`，控制器不会更新里程计，功能节点也就无法闭环。

`controller_bridge` 解决了 Jazzy 的一个接口差异：

- Jazzy 的 `diff_drive_controller` 订阅 `geometry_msgs/TwistStamped`；
- 框架中的键盘节点和自动巡航节点发布 `geometry_msgs/Twist`；
- bridge 把 Twist 转成带时间戳的 TwistStamped，并把控制器里程计转发为标准 `/odom`。

## 4. 目录结构

```text
gazebo_diff_drive_teleop/
├── main.sh                                  # 一键编译/启动
├── README.md
├── package.xml / setup.py / setup.cfg
├── config/diff_drive_controller.yaml        # 控制器参数
├── launch/gazebo_diff_drive_teleop.launch.py
├── urdf/diff_drive_robot.urdf.xacro         # 车体、轮子、ros2_control
└── gazebo_diff_drive_teleop/
    ├── __init__.py
    ├── teleop_keyboard.py                   # 键盘遥控
    ├── auto_drive.py                        # 自动巡航
    ├── battery_teleop.py                  # 本项目新增功能
    └── controller_bridge.py                 # Twist/TwistStamped 与 odom 桥
```

## 5. 运行环境与依赖

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12
- 依赖：`ros_gz_sim`、`ros_gz_bridge`、`ros2_control`、`ros2_controllers`、`gz_ros2_control`、`robot_state_publisher`、`xacro`

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-robot-state-publisher ros-jazzy-xacro
```

## 6. 构建与运行

在模块所在工作空间根目录执行：

```bash
bash main.sh                 # 自动巡航
bash main.sh manual          # 仅启动仿真
bash main.sh battery # 本项目功能
```

也可以手动构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select gazebo_diff_drive_teleop
source install/setup.bash
ros2 launch gazebo_diff_drive_teleop gazebo_diff_drive_teleop.launch.py demo:=false
```

只启动 Gazebo 服务端、不打开 GUI：

```bash
bash main.sh battery gz_args:='-r -s'
```

`main.sh` 的后台/前台分工是有意的：`ros2 launch` 不会把终端 TTY 传给子节点，因此键盘节点必须由 `main.sh` 在前台通过 `ros2 run` 启动，才能在终端读取按键。

## 7. 键盘操作

通用驾驶按键：

| 按键 | 功能 |
| --- | --- |
| `w` / `↑` | 前进 |
| `x` / `↓` | 后退 |
| `a` / `←` | 左转 |
| `d` / `→` | 右转 |
| `s` / 空格 | 停止 |
| `q` / `Ctrl-C` | 退出（恢复终端并发送零速度） |

本项目功能按键：

| 按键 | 功能 |
| --- | --- |
| `s` / 空格 | 停止；返航中暂停返航 |
| `h` | 手动返回充电点 |
| `b` | 演示用，强制把电量降到阈值以下 |
| `+` / `-` | 调整线速度 |

## 8. 电量模型与低电量自动返航

### 8.1 电池模型

节点根据 `/odom` 的实际线速度和角速度积分耗电：

```text
drain = idle_drain + drain_per_meter * |v| + drain_per_radian * |w|
battery = max(0, battery - drain * dt)
```

进入充电状态后：

```text
battery = min(100, battery + charge_rate * dt)
```

电池是仿真量，不读取真实电源；参数 `initial_battery`、`low_battery_threshold`、`charged_threshold` 分别控制初始电量、返航触发阈值和充电完成阈值。

### 8.2 状态机

```text
manual ──电量 <= low_threshold──▶ returning ──到达充电点──▶ charging
  ▲                                  │                         │
  └──────── 手动接管/按 s ◀──────────┘                         │
  └────────────────── 电量 >= charged_threshold ◀─────────────┘
```

- `manual`：键盘驾驶，持续耗电。
- `returning`：自动转向充电点，按距离比例前进，接近时减速；到达位置后对齐 `home_yaw`。
- `charging`：原地停止并充电。电量达到 `charged_threshold` 后自动回到 manual。
- 手动按键会取消自动返航；此时 `_auto_return_armed` 置为 false，防止低电量状态每个周期反复抢回控制权。按 `h` 可以重新武装并启动返航。

### 8.3 返航控制

返航使用与差速运动学一致的位姿闭环：

```text
distance = hypot(home.x - x, home.y - y)
desired_yaw = atan2(home.y - y, home.x - x)
heading_error = normalize(desired_yaw - yaw)
```

- 航向误差大于 0.8 rad：原地转向；
- 否则线速度与距离成正比，角速度与航向误差成正比；
- 距离小于 0.35 m 时进一步降速；
- 位置进入 `goal_tolerance` 后，再对齐充电点的目标航向，然后进入充电状态。

### 8.4 演示与安全

- 按 `b` 会把电量强制设为 `low_threshold - 1%`，用于快速演示低电量返航，不影响真实耗电逻辑。
- 退出时恢复终端并连续发送零速度。
- 低电量返航是单机器人、基于里程计的行为，没有避障和全局定位。

## 9. 参数说明

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `initial_battery` | 100.0 % | 初始电量 |
| `low_battery_threshold` | 25.0 % | 低电量阈值 |
| `charged_threshold` | 80.0 % | 充电完成阈值 |
| `drain_per_meter` | 8.0 %/m | 每米耗电 |
| `drain_per_radian` | 2.0 %/rad | 每弧度转向耗电 |
| `idle_drain_per_second` | 0.15 %/s | 静止自放电 |
| `charge_rate` | 8.0 %/s | 充电速率 |
| `home_x` / `home_y` / `home_yaw` | 0 / 0 / 0 | 充电点位姿 |
| `goal_tolerance` | 0.12 m | 位置到达阈值 |
| `yaw_tolerance` | 0.12 rad | 航向到达阈值 |
| `return_linear_gain` | 1.2 | 返航线速度增益 |
| `return_angular_gain` | 2.0 | 返航角速度增益 |

## 10. 话题、TF 与状态

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `/odom` | `nav_msgs/Odometry` | 差速控制器里程计 |
| `/battery_state` | `sensor_msgs/BatteryState` | 仿真电池电压、电流、百分比和状态 |
| `/battery_return/status` | `std_msgs/String` | `manual` / `returning` / `charging` |
| `/battery_return/markers` | `visualization_msgs/MarkerArray` | 充电点和返航目标标记 |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | 控制器原生输入 |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | 控制器原生里程计 |

`/battery_state` 中：

- `percentage` 为 0~1；
- `power_supply_status` 在充电时为 `CHARGING`，否则为 `DISCHARGING`；
- `voltage`、`current`、`charge`、`capacity` 是按仿真电量计算的展示值。

TF 关系与基础模块一致：`odom -> base_footprint -> base_link -> wheels/caster`。

## 11. 实测结果与复现方法

实验方法：在无界面 Gazebo 中启动完整控制链路，通过伪终端向前台节点注入按键；订阅 `/odom` 和功能话题，记录真实位姿/电量/状态后计算指标。这样既复现了人工键盘操作，又避开了 GUI 截图对测量精度的影响。

```bash
# 终端 1：启动仿真
bash main.sh battery gz_args:='-r -s'

# 终端 2：观察话题
ros2 control list_controllers
ros2 topic hz /odom
ros2 topic echo /clock --once
```

真实 Gazebo 测试：小车从充电点驶出约 0.67 m 后按 `b` 强制低电量，节点自动返回充电点，进入 `charging` 状态，电量从 6.3% 开始回升。

| 阶段 | 结果 |
| --- | --- |
| 手动驶离 | 距充电点约 0.67 m |
| 强制低电量 | 电量降到阈值以下，状态切为 `returning` |
| 自动返航 | 进入充电点 0.12 m 范围，状态切为 `charging` |
| 充电 | 电量从 6.3% 开始上升 |

图中左侧为手动驶离（蓝）和自动返回（橙）轨迹，绿色方块为充电点；右侧为电量曲线，阴影区域为充电阶段。

![电量管理与自动返航运行数据 真实运行数据](../img/ground/gazebo_diff_drive_teleop_battery_return.png)

## 12. 故障排查

| 现象 | 排查方向 |
| --- | --- |
| 小车不动 | 用 `ros2 control list_controllers` 确认 `diff_drive_controller` 为 `active`；确认 `/clock` 有数据 |
| 没有 `/odom` | 检查 `clock_bridge` 和 `controller_bridge` 是否启动；`ros2 topic info /diff_drive_controller/odom` |
| 键盘节点读不到按键 | 必须在交互式终端中运行；`main.sh` 已经用前台 `ros2 run` 启动，不要把它放到后台 |
| 控制器提示 Velocity command timed out | 确认 `/cmd_vel` 仍在发布；bridge 是否存活 |
| 功能节点启动后立即退出 | 查看终端是否提示“标准输入不是交互式终端”；改用 `main.sh` 或交互式终端 |
| `colcon build` 报重复包名 | `main.sh` 已使用 `--base-paths src`，避免扫描工作空间外的备份目录 |
| 电量不下降 | 确认 `/odom` 的 `twist.twist.linear.x` 在变化；电量按实际里程计速度计算，不是按按键次数计算 |
| 低电量后不返航 | 确认 `_auto_return_armed` 没有被手动接管关闭；按 `h` 重新启动返航，或按 `b` 重新触发低电量 |

## 13. 已知限制与后续工作

- 位姿全部来自轮式里程计，长时间运行会有累积漂移；没有使用激光雷达、相机或全局定位。
- 仿真场景为平面，未考虑坡道、打滑、悬挂和轮胎形变。
- 电池模型是软件仿真，参数不代表真实电池化学特性；返航依赖里程计，未考虑障碍物和充电桩视觉对接。
- 测试在单台 Ubuntu 24.04 + Jazzy 机器上完成；合并前仍需第二台机器复现。
- 后续可以加入 IMU/EKF 融合、`slam_toolbox` 或 Nav2，把当前功能扩展到真实定位与导航。

## 14. 与仓库已有模块的区别

- `gazebo_diff_drive_teleop` 原模块只做速度控制，不管理能量。
- 仓库中的 `cmd_vel_guard` 是速度安全监督，不涉及电量。
- 本项目把电量消耗、低电量决策、返航控制和充电状态串成完整状态机，并发布标准 `BatteryState` 话题。

## 15. 说明

代码与文档使用 AI 大模型辅助编写。作者已核对代码、参数和实验数据，并对提交内容负责。

