# uav_slam_nav

无人机 SLAM 建图 + 导航包。**ROS2 Humble (ament_python)**，通过 `ros1_bridge` 接 Noetic 上的 `carlair_ros_bridge`，和仓库里 `uav_battery_rth_failsafe`、`fire_detection_drone` 等包一致。

## 链路

```
Noetic 侧:
  AirSim -> carlair_ros_bridge -> /lidar/points, /uav/odom, /uav/cmd_vel
                                    |
                              ros1_bridge 跨版本
                                    |
Humble 侧:
  /lidar/points -> points_to_scan -> /scan -> slam_toolbox -> /map (map->odom)
                                     \-> nav2 (planner/controller/costmap) -> /cmd_vel
  /uav/odom -> odom_tf -> odom->base_link TF
  simple_nav: nav2 停发 /cmd_vel 超过 1.5s 时接管，输出 /simple_nav/cmd_vel
  nav_relay: /uav/cmd_vel 的唯一发布者，仲裁 nav2 / simple_nav，限幅，z 恒 0
```

## 文件

```
uav_slam_nav/
  nav_logic.py          纯计算（投影、扇区测距、兜底决策、仲裁），不 import rclpy
  points_to_scan_node.py 点云 -> /scan，用 tf2 查 cloud->base_link 变换
  odom_tf_node.py       /uav/odom -> odom->base_link TF
  simple_nav_node.py    nav2 失败时的兜底
  nav_relay_node.py     /uav/cmd_vel 唯一发布者
launch/slam_nav.launch.py
config/slam_toolbox.yaml
config/nav2_params.yaml
test/test_uav_slam_nav_local.py
```

## 控制权

- `/uav/cmd_vel` 只有 `nav_relay` 一个发布者。
- `simple_nav` 输出到 `/simple_nav/cmd_vel`，不直接发 `/uav/cmd_vel`。
- 仲裁规则（`nav_logic.select_source`）：兜底新鲜（<0.5s）> nav2 新鲜 > 都不发（让 bridge 超时悬停）。
- `simple_nav` 只在 nav2 的 `/cmd_vel` 停发 1.5s 以上、且最近收到过 `/goal_pose` 时才接管；nav2 恢复后自动闭嘴。
- 兜底用 tf2 查目标点相对 base_link 的方位角朝目标修角，前向 120° 扇区 1m 内有障碍就停车，往空间大的一侧转。

## 坐标系

标准导航栈 TF 树：`map -> odom -> base_link`。

- `slam_toolbox` 发 `map -> odom`。
- `odom_tf` 把 `/uav/odom` 发成 `odom -> base_link`。
- launch 里加了一个 `odom -> world` 的静态单位变换，把 bridge 点云里的 `world` frame 接到树上。
- nav2 配置里 global costmap 用 `map`、local costmap 用 `odom`，`robot_base_frame` 统一 `base_link`（上一版 global_frame 一个 map 一个 world 的问题已修）。

## 参数

- 点云切片：`height_lo=-0.3m`、`height_hi=3.0m`，`range_max=30m`，1°/beam。
- nav2：全局 costmap 30×30m、局部 12×12m，resolution 0.1，inflation 全局 1.2m / 局部 0.8m。
- 速度限幅：`max_vx=0.8`、`max_vy=0.5`、`max_yaw=0.6`，z 恒 0（定高交给 bridge）。
- 兜底：`stop_distance=1.0m`、`turn_rate=0.6rad/s`、`nav_stale_s=1.5`。

## 跑

```bash
# Noetic 终端
roslaunch carlair_ros_bridge bringup.launch
roslaunch ros1_bridge ros1_bridge.launch

# Humble 终端
ros2 launch uav_slam_nav slam_nav.launch.py
# RViz 里 Add -> Map / LaserScan / TF，点 "2D Goal Pose" 给目标
```

## 测试

```bash
python3 test/test_uav_slam_nav_local.py
```

22 个用例，不依赖 ROS，CI 直接跑。覆盖点云投影、扇区测距、兜底决策、仲裁、TF 矩阵，以及三条静态断言（`simple_nav` 不许直发 `/uav/cmd_vel`、`nav_relay` 是 `/uav/cmd_vel` 唯一发布者、`points_to_scan` 用 tf2）。
