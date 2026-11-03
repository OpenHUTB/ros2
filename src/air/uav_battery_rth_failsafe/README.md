# 无人机电池电量监控与自动返航 (RTH) 迫降

订阅电池电量与无人机位置, 按分级阈值触发预警、自动返航到起飞点、就近迫降。

与 `uav_mission_safety_manager`(通用任务状态机)的区别: 本模块**专注电源管理与失效保护**,
不介入任务流程, 只在电量危险时接管位置目标。

## 运行环境

- Ubuntu 20.04 + ROS 2 Humble
- AirSim(或提供 `/drone/battery`、`/drone/position` 的桥接)

## 状态分级

| 状态 | 电量条件 | 行为 |
|---|---|---|
| NORMAL | > 25% | 不干预 |
| WARNING | 15% ~ 25% | 日志预警; 若估算返航电量不足则提前返航 |
| RETURN_HOME | 8% ~ 15% | 目标切到 home 点上方 3m |
| EMERGENCY_LAND | <= 8% | 切到最近预设安全点(或当前位置)垂直迫降 |

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/drone/battery` | `std_msgs/Float32` (0~100) |
| 订阅 | `/drone/position` | `geometry_msgs/PointStamped` |
| 发布 | `/drone/setpoint_position` | `geometry_msgs/PointStamped` |
| 发布 | `/drone/failsafe_state` | `std_msgs/String` |

## 运行

```bash
cd src/air/uav_battery_rth_failsafe
python3 main.py
# 或
ros2 launch launch/uav_battery_rth.launch.py
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `warn_percent` | 25.0 | 预警阈值 |
| `rth_percent` | 15.0 | 返航阈值 |
| `emergency_percent` | 8.0 | 迫降阈值 |
| `power_per_meter` | 0.15 | 每米距离估算耗电百分比 |
| `home_x/y/z` | 0/0/0 | 起飞点坐标 |
| `safe_zones` | "" | 分号分隔的迫降点 "x,y,z;x,y,z" |

## 验证范围

- 节点启动后按阈值发布 failsafe_state
- 电量降低时位置目标切换到 home/迫降点
- 完整闭环需配合 AirSim 电池话题与位置控制桥接验证
