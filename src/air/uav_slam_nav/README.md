# uav_slam_nav

第三次作业，SLAM 建图 + 无人机避障导航。

## 干啥的

- slam_toolbox 异步建图，实时出 `/map`
- 把 bridge 发的世界系点云投影成 base_link 下的 `/scan`
- move_base 做全局/局部规划
- `nav_relay.py` 是 `/uav/cmd_vel` **唯一**的发布者：把 move_base 的速度限幅、清零 z 后转发给 bridge
- `simple_nav.py` 是 move_base 失败时的兜底，只在 `/move_base/status` 显示当前目标失败时接管

## 演示

![demo](docs/slam_nav_demo.gif)

## 怎么跑

先开 Windows 上的 AirSim 仿真器，然后在虚拟机里：

```bash
roslaunch carlair_ros_bridge main.launch        # 先连上仿真器
roslaunch uav_slam_nav slam_nav.launch
```

> 这个包是 ROS1 (Noetic/catkin)，所以用 `roslaunch`。原因是它下游连着
> `carlair_ros_bridge`：bridge 用 `rospy` 发 `/lidar/points`、`/uav/odom`，收
> `/uav/cmd_vel`，是 catkin 包；ROS1 和 ROS2 的话题不能直接互通，仓库里也没有
> `ros1_bridge`，所以链路这一端必须是 ROS1。`src/air` 下现在 22 个 catkin 包、
> 27 个 ament 包，本身就是混着的，`uav_keyboard_control`、`uav_landing_monitor`
> 也都是 catkin，本包跟它们保持一致。

## 话题和 TF

| 话题 | 方向 | 说明 |
|------|------|------|
| `/lidar/points` | in | bridge 发的世界系 PointCloud2 |
| `/scan` | out | points_to_scan 转出来的 LaserScan，base_link 系 |
| `/uav/odom` | in | bridge 发的无人机里程计 |
| `/map` | out | slam_toolbox 建的栅格图 |
| `/cmd_vel` | in | move_base 输出 |
| `/move_base/status` | in | 判断当前目标是否失败 |
| `/move_base/goal` | in | 兜底用的目标点 |
| `/simple_nav/cmd_vel` | 内部 | 兜底节点输出，只有接管时才发 |
| `/uav/cmd_vel` | out | **只有 nav_relay 发**，仲裁后的最终速度 |

TF 链路：
- `world -> base_link`：由 carlair_ros_bridge 发（跟 `/uav/odom` 同源）
- `map -> world`：由 slam_toolbox 发（校正漂移）

## 控制权怎么分配

之前 `simple_nav` 是每帧 `/scan` 就发一次 `/uav/cmd_vel`，跟 move_base 抢同一条
话题，两边同时发速度，机体会抖。现在改成单一发布者：

```
move_base ──/cmd_vel────────────┐
                                ├─ nav_relay（仲裁 + 限幅 + z 清零）── /uav/cmd_vel ── bridge
simple_nav ──/simple_nav/cmd_vel┘
```

规则（`nav_logic.select_source`）：
- 兜底有输出且新鲜（0.5s 内）→ 用兜底
- 否则 move_base 有输出且新鲜 → 用 move_base
- 都不新鲜 → 不发任何速度，交给 bridge 自己的超时悬停

`simple_nav` 的接管条件（`nav_logic.status_is_failed`）：`/move_base/status`
最后一个目标的 status 是 ABORTED(4) / REJECTED(5) / RECALLED(8) / LOST(9)，且这条
状态是 1s 内的。move_base 正常规划或执行时它一个字都不发；交还控制权时会先补一帧
零速再停。

兜底的动作也不是随机转向：它记住 `/move_base/goal` 的目标点，用 `map -> base_link`
的 TF 算目标方位，朝目标修角（偏差大于 1.2 rad 时只转不走），同时看 `/scan` 前向
120° 扇区，1 m 内有障碍就停车并往空间大的一侧转。

## 关键参数

**点云切片（points_to_scan.py）**：
- `height_lo = -0.3 m`，`height_hi = 3.0 m`：base_link 系下 z 方向只取这个区间，滤掉地面和楼顶
- `range_min = 0.3 m`，`range_max = 30 m`

**SLAM（slam_toolbox）**：
- `resolution = 0.1 m`
- `map_update_interval = 1.0 s`
- `minimum_time_interval = 0.1 s`
- `transform_tolerance = 0.2 s`

**代价地图（config/costmap.yaml）**：
- footprint：0.35 m 半边长的正方形（`[[0.35,0.35],[0.35,-0.35],...]`），按无人机外廓加了点裕度
- 全局 costmap：rolling_window 30×30 m，inflation_radius 1.2 m
- 局部 costmap：rolling_window 12×12 m，inflation_radius 0.8 m

**速度限幅与定高（nav_relay.py）**：
- `max_vx = 0.8 m/s`，`max_vy = 0.5 m/s`，`max_yaw = 0.6 rad/s`
- linear.z 直接清零：nav_relay 不发任何升降指令，保持当前高度
- **定高由 carlair_ros_bridge 负责**：bridge 内部有高度保持控制器，收到 z=0 的速度指令后维持当前飞行高度，不会下沉
- move_base 停发超过 0.5 s 后 bridge 自己会悬停（超时保护）

**兜底（simple_nav.py）**：
- `stop_distance = 1.0 m`，`turn_rate = 0.6 rad/s`
- `max_vx = 0.5 m/s`，`max_yaw = 0.6 rad/s`
- `status_timeout = 1.0 s`，`scan_timeout = 0.5 s`

## 测试

纯逻辑抽到了 `scripts/nav_logic.py`（不 import rospy），`tests/` 里直接跑：

```bash
python3 src/air/uav_slam_nav/tests/test_uav_slam_nav_local.py
```

40 个用例，覆盖：
- 点云投影：高度切片、最近点取 bin、超 range 丢弃、姿态旋转、位置平移
- 扇区测距：前向扇区只算 ±60°，侧向扇区 (±60°, ±120°)
- 兜底决策：前向 1 m 内有障碍停车转向、没有目标点时不动、方位偏差大时只转不走
- 速度仲裁：兜底优先、兜底失效回落到 move_base、两边都超时就不发
- 静态回归：`simple_nav` 不许直接发 `/uav/cmd_vel`、必须订阅 `/move_base/status`

CI 里 `Byte-compile slam_nav scripts` 之后会跑这一步。

## 运行验证

launch 启动后关键节点都起来了：

```
$ rosnode list
/move_base
/nav_relay
/points_to_scan
/simple_nav
/slam_toolbox
```

话题频率：

```
$ rostopic hz /map
average rate: 1.18 Hz

$ rostopic hz /scan
average rate: 9.7 Hz

$ rostopic hz /uav/odom
average rate: 20.1 Hz
```

RViz 里加 Map、LaserScan、TF 三个 display，能看到地图随无人机移动慢慢长出来。gif 里录的就是这个过程。

`/uav/cmd_vel` 的发布者确认只有一个：

```
$ rostopic info /uav/cmd_vel
Publishers:
 * /nav_relay
Subscribers:
 * /cmd_vel_sub
```

兜底那条分支没在仿真里刻意把 move_base 跑失败，只用单元测试覆盖了决策和仲裁。

## 环境说明

我这边仿真器是跑在 Windows 宿主机上的 AirSim，ROS 代码跑在 Ubuntu 虚拟机里，虚拟机通过 TCP 连 `10.0.2.2:41451` 跟 Windows 通信。跨主机会有几十毫秒延迟，物理引擎也跟纯 Linux 版 AirSim 有点差别，轨迹会稍微飘一点，但功能逻辑是一样的。
