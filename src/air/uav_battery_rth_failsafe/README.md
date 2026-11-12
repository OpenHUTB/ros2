# 无人机电池电量监控与自动返航 (RTH) 迫降

订阅电池电量与无人机位置, 按分级阈值触发预警、自动返航到起飞点、就近迫降。

与 `uav_mission_safety_manager`(通用任务状态机)的区别: 本模块**专注电源管理与失效保护**,
不介入任务流程, 只在电量危险时接管位置目标。

## 1. 仿真平台

本模块按仓库推荐的两套仿真底座设计, **不使用 Gazebo**:

| 平台 | 底座包 | ROS 版本 | 说明 |
|---|---|---|---|
| MuJoCo(推荐主平台) | `src/air/uav_mujoco_sim` | Humble | 纯 CPU, 与本包同为 ament_python / Humble, 话题可直接映射 |
| AirSim(备选) | `src/air/drone_airsim_bringup` | Noetic | CarlaAir / AirSim 1.8.1, 跨版本接入需 ros1_bridge |

模块本身不依赖任何仿真器, 只依赖两条话题: 电量(`std_msgs/Float32`)、位置(`geometry_msgs/PointStamped`),
因此换底座时**不改代码**, 只改话题名参数(见第 5 节)。

## 2. 状态分级

| 状态 | 电量条件 | 行为 |
|---|---|---|
| NORMAL | > 25% | 不干预 |
| WARNING | 15% ~ 25% | 日志预警; 若估算返航电量不足则提前返航 |
| RETURN_HOME | 8% ~ 15% | 目标切到 home 点上方 3m |
| EMERGENCY_LAND | <= 8% | 切到最近预设安全点(或当前位置)垂直迫降 |

电池话题断流超过 `battery_timeout`(默认 3s)时按失联处理, 直接进入 `EMERGENCY_LAND`。

## 3. 话题

| 方向 | 默认话题 | 参数名 | 类型 |
|---|---|---|---|
| 订阅 | `/drone/battery` | `battery_topic` | `std_msgs/Float32` (0~100) |
| 订阅 | `/drone/position` | `position_topic` | `geometry_msgs/PointStamped` |
| 发布 | `/drone/setpoint_position` | `setpoint_topic` | `geometry_msgs/PointStamped` |
| 发布 | `/drone/failsafe_state` | `state_topic` | `std_msgs/String` |

`NORMAL` / `WARNING` 状态**不发布** setpoint, 不干扰正在执行任务的节点;
只有 `RETURN_HOME` 与 `EMERGENCY_LAND` 才接管位置目标。

## 4. 构建与运行

```bash
# 在 ROS2 工作区根目录
colcon build --packages-select uav_battery_rth_failsafe
source install/setup.bash

# 方式一: 直接运行节点
ros2 run uav_battery_rth_failsafe battery_rth_node

# 方式二: 带默认参数的 launch
ros2 launch uav_battery_rth_failsafe uav_battery_rth.launch.py
```

命令行覆盖参数:

```bash
ros2 run uav_battery_rth_failsafe battery_rth_node --ros-args \
  -p home_x:=0.0 -p home_y:=0.0 -p home_z:=0.0 \
  -p warn_percent:=25.0 -p rth_percent:=15.0 -p emergency_percent:=8.0
```

## 5. 与仿真底座对接

两个底座都不发布"电池电量"话题(MuJoCo 与 AirSim 的物理模型里没有电量状态),
需要在仿真侧注入一路电量、并做好位置与目标的话题映射。

### 5.1 MuJoCo(`uav_mujoco_sim`)

| 本模块需要 | 底座提供 | 处理方式 |
|---|---|---|
| 位置 `PointStamped` | `/uav/odom` (`nav_msgs/Odometry`) | 取 `pose.pose.position` 转 `PointStamped` 后再 remap |
| 目标 `PointStamped` | `/uav/goal` (`geometry_msgs/Point`) | 取 `point` 转 `Point` 后再 remap |
| 电量 `Float32` | 无 | 用脚本按放电曲线发布, 见下 |

```bash
# 模拟电量从 100% 线性放电到 5%(每 2 秒掉 1 个百分点)
ros2 topic pub -r 0.5 /drone/battery std_msgs/msg/Float32 "{data: 95.0}"

# 启动本模块并对齐 MuJoCo 话题
ros2 run uav_battery_rth_failsafe battery_rth_node --ros-args \
  -p position_topic:=/uav/position_from_odom \
  -p setpoint_topic:=/uav/goal_from_setpoint
```

### 5.2 AirSim(`drone_airsim_bringup`)

| 本模块需要 | 底座提供 | 处理方式 |
|---|---|---|
| 位置 `PointStamped` | `/drone/pose` (`geometry_msgs/PoseStamped`, NED) | 需 ros1_bridge 跨到 Humble, 并做 NED→ENU 转换 |
| 目标 `PointStamped` | `/drone/cmd_vel` (`Twist`, NED 速度) | 本模块输出的是位置目标, 需位置控制器转速度 |
| 电量 `Float32` | 无 | 同上, 脚本注入 |

AirSim 底座是 ROS Noetic, 与本包(Humble)跨版本, 需要 `ros1_bridge` 才能互通;
如果课程环境是纯 Humble, 建议直接用 MuJoCo 底座。

## 6. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `warn_percent` | 25.0 | 预警阈值(%) |
| `rth_percent` | 15.0 | 返航阈值(%) |
| `emergency_percent` | 8.0 | 迫降阈值(%) |
| `power_per_meter` | 0.15 | 每米距离估算耗电百分比 |
| `cruise_speed_mps` | 3.0 | 返航巡航速度(m/s), 用于耗时估算 |
| `battery_timeout` | 3.0 | 电池话题断流判据(s) |
| `home_x` / `home_y` / `home_z` | 0.0 | 起飞点坐标(ENU) |
| `safe_zones` | "" | 分号分隔的迫降点 `"x,y,z;x,y,z"`, 留空则原地迫降 |
| `battery_topic` | `/drone/battery` | 电量话题名 |
| `position_topic` | `/drone/position` | 位置话题名 |
| `setpoint_topic` | `/drone/setpoint_position` | 目标点话题名 |
| `state_topic` | `/drone/failsafe_state` | 状态话题名 |

## 7. 单元测试

纯计算部分拆到 `rth_math.py`(`decide_state` / `dist_3d`), 不依赖 ROS, 可直接跑:

```bash
python3 src/air/uav_battery_rth_failsafe/test_rth.py
```

覆盖 6 项: 四状态阈值跳转、预警但返航余量不足时提前返航、三维距离计算。

## 8. 验证范围(诚实说明)

- 已实测: 节点级验证。注入电量与位置话题后, 四状态按时跳转、`failsafe_state` 正确发布、
  仅在 `RETURN_HOME` / `EMERGENCY_LAND` 时接管 setpoint、电池断流 3s 进入迫降。
- 未实测: 与 MuJoCo / AirSim 底座的完整闭环飞行(第 5 节的话题转换节点尚未随包提供)。
  该部分属于教学演示级的接入说明, 更换底座时按第 5 节映射即可, 不需要改动本模块代码。
