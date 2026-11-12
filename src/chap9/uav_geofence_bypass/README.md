# uav_geofence_bypass

无人机 **禁飞区膨胀 + 可见图最短绕行**。

## 要点

- `inflate_polygon` 按安全裕度膨胀凸多边形（默认 0.6 m）
- `segment_clear` 判断线段是否穿入内部，允许与顶点相切
- 以起点、膨胀顶点、终点建可见图，Dijkstra 求最短路
- 禁飞区、起点、终点、裕度都在 `config/params.yaml` 里，不用改代码

## 话题

| 话题 | 类型 | 方向 |
|---|---|---|
| `/uav/odom` | nav_msgs/Odometry | 订阅 |
| `/uav/goal` | geometry_msgs/PoseStamped | 订阅（可动态改目标） |
| `/uav/safe_goal` | geometry_msgs/PoseStamped | 发布 |
| `/uav/geofence_state` | std_msgs/String | 发布（诊断） |

## 环境（实测）

- Ubuntu 20.04 + ROS 2 Foxy，Python 3.8.10
- 无 Gazebo 依赖：仿真在包内 `uav_sim` 节点完成

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select uav_geofence_bypass
source install/setup.bash
ros2 launch uav_geofence_bypass main.launch.py
# 换一套禁飞区：
ros2 launch uav_geofence_bypass main.launch.py params_file:=/path/to/params.yaml
```

## 参数（config/params.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `margin` | 0.6 | 禁飞区膨胀裕度 (m) |
| `accept_radius` | 0.3 | 航点切换半径 (m) |
| `log_period` | 1.0 | 日志周期 (s) |
| `start` | [0, 0] | 起点 |
| `goal` | [15, 0] | 目标 |
| `zones` | 2 个四边形 | 每个禁飞区按 x1,y1,x2,y2… 排列 |

## 测试

```bash
PYTHONPATH=src/chap9/uav_geofence_bypass python3 -m pytest \
  src/chap9/uav_geofence_bypass/test -q
```

## 许可证

MIT
