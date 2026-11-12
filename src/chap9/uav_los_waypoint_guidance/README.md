# uav_los_waypoint_guidance

无人机 **LOS 视线制导**航点跟踪，输出机体系速度指令。

## 要点

- `segment_tracking` 求相对航段的纵向进度与有向横向误差 `e`
- `psi_d = psi_path - atan2(e, Delta)`，把横向偏差直接折进期望航向
- 机体系速度 `(v cos(err), v k sin(err))`，偏航角速度 = `yaw_gain * err` 并限幅
- 控制器由独立 timer 驱动（`control_rate_hz`），odom 只更新状态，odom 断了不会卡死
- 参数走 ROS 2 标准 YAML（节点名下的 `ros__parameters`），命令行 `-p` 覆盖同样生效

## 话题

| 话题 | 类型 | 方向 |
|---|---|---|
| `/uav/odom` | nav_msgs/Odometry | 订阅 |
| `/uav/cmd_vel` | geometry_msgs/Twist | 发布（机体系速度） |
| `/uav/los_state` | std_msgs/String | 发布（诊断） |

## 环境（实测）

- Ubuntu 20.04 + ROS 2 Foxy，Python 3.8.10
- 无 Gazebo 依赖：仿真在包内 `uav_sim` 节点完成

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select uav_los_waypoint_guidance
source install/setup.bash
ros2 launch uav_los_waypoint_guidance main.launch.py
# 换参数文件
ros2 launch uav_los_waypoint_guidance main.launch.py params_file:=/path/to/params.yaml
# 命令行覆盖单个参数（标准 ROS 2 参数机制）
ros2 run uav_los_waypoint_guidance main --ros-args -p speed:=2.0 -p yaw_gain:=1.5
```

## 参数（config/params.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `speed` | 1.5 | 巡航速度 (m/s) |
| `lookahead` | 2.0 | LOS 前视距离 (m) |
| `accept_radius` | 0.8 | 航点接受半径 (m) |
| `log_period` | 0.5 | 日志/状态发布周期 (s) |
| `control_rate_hz` | 20.0 | 控制循环频率 (Hz) |
| `yaw_gain` | 1.2 | 航向误差到偏航角速度的增益 |
| `max_yaw_rate` | 0.8 | 偏航角速度限幅 (rad/s) |
| `waypoints` | … | 成对的 x, y 航点 |

## 测试

```bash
PYTHONPATH=src/chap9/uav_los_waypoint_guidance python3 -m pytest \
  src/chap9/uav_los_waypoint_guidance/test -q
```

## 许可证

MIT
