# 地面无人车 SLAM 建图与导航（ugv_slam_nav）

任务(3)的地面载具实现：turtlebot3 在 Gazebo 里跑 slam_toolbox 建图，nav2 导航，本包补 odom→TF、nav2 超时兜底和唯一 cmd_vel 仲裁。

## 链路

```
Gazebo turtlebot3 -> /scan (2D lidar), /odom
                  \-> /scan -> slam_toolbox -> /map
                           \-> nav2 -> /nav2/cmd_vel
odom_tf_node: /odom -> odom->base_footprint TF
simple_nav_node: 收到 /goal_pose 后，前向 120° 扇区避障自己走（nav2 超时兜底）
nav_relay_node: 仲裁 nav2 / simple_nav，限幅 vx 0.5 / yaw 1.0，唯一发 /cmd_vel
```

## 启动

```bash
# 终端1：turtlebot3 仿真
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端2：本包
ros2 launch ugv_slam_nav main.launch.py
# 或
ros2 run ugv_slam_nav main
```

RViz 里 Add Map / LaserScan / TF，点 2D Goal Pose 给目标。

## 节点

| 节点 | 作用 |
|---|---|
| `ugv_odom_tf` | /odom 发成 odom→base_footprint TF |
| `ugv_simple_nav` | nav2 停发 1.5s 且收到 /goal_pose 时接管，前向扇区避障 |
| `ugv_nav_relay` | 仲裁 nav2 / simple_nav，限幅，唯一 /cmd_vel 发布者 |

## 参数

- `config/slam_toolbox.yaml`：建图分辨率 0.05，最大激光 8m
- `config/nav2_params.yaml`：global costmap=map，local=odom，base=base_footprint，vx≤0.5

## 测试

```bash
python3 test/test_ugv_slam_nav_local.py
```

13 个用例，不依赖 ROS，CI 直接跑。覆盖 clamp、relay 仲裁/限幅、扇区避障（空scan/障碍近/巡航/减速带/nan/侧面忽略）。
