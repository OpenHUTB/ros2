# uav_slam_nav —— AirSim 无人机建图与自主导航（ROS2 Humble）

无人机在 AirSim 场景中用**前视深度相机**边飞边建图（占据栅格），地图上做 **A\***
路径规划，再用**纯追踪**控制器沿路径自主飞行。全流程由 4 个 ROS2 节点组成，
通过 `/map`、`/plan`、`/uav/cmd_vel` 三个标准话题串联，可在 rviz 里实时观察。

对应课程任务（3）：在虚拟环境中实现机器人建图与导航，导航与 SLAM 同步仿真。

## 1. 环境要求

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04（VirtualBox 虚拟机） |
| ROS | ROS2 Humble |
| 仿真器 | AirSim（Unreal Blocks 环境），运行在 Windows 宿主机 |
| Python 依赖 | `numpy`、`airsim`（`pip3 install airsim numpy`） |

网络拓扑：虚拟机通过 VirtualBox NAT 访问宿主机 `10.0.2.2:41451`（AirSim 的 RPC 端口）。
如果用的是桥接网卡或双系统，把 `airsim_ip` 参数改成宿主机的局域网 IP 即可。

## 2. 目录结构

```
src/uav_slam_nav/
├── package.xml                     # ament_python 包描述
├── setup.py / setup.cfg            # 安装配置，入口 main = uav_slam_nav.main:main
├── resource/uav_slam_nav           # ament 索引占位文件
├── config/params.yaml              # 4 个节点的默认参数
├── launch/main.launch.py   # 一条 launch 起全部节点
├── uav_slam_nav/
│   ├── main.py                     # 总入口，--role 选择节点组合
│   ├── airsim_source.py            # AirSim <-> ROS2 数据源
│   ├── mapping.py                  # 深度反投影 + 占据栅格 + /map
│   ├── planner.py                  # A* 规划 + /plan
│   ├── controller.py               # 纯追踪 + /uav/cmd_vel
│   └── grid_utils.py               # 坐标变换、反投影、Bresenham（无 ROS 依赖）
└── README.md
```

## 3. 构建

```bash
cd ~/ros2
colcon build --packages-select uav_slam_nav --symlink-install
source install/setup.bash
```

## 4. 运行

### 4.1 一条命令跑全套（推荐）

```bash
# 宿主机（Windows）：先启动 AirSim
#   D:\airsim\Blocks\WindowsNoEditor\Blocks.exe

# 虚拟机（Ubuntu）
ros2 launch uav_slam_nav main.launch.py
```

launch 会依次拉起 `airsim_source`、`mapping`、`planner`、`controller` 四个节点，
全部挂在 `uav` 命名空间下，参数取自 `config/params.yaml`。

### 4.2 分步运行（调试时用）

```bash
# 终端 1：数据源
ros2 run uav_slam_nav main --role source

# 终端 2：建图
ros2 run uav_slam_nav main --role mapping

# 终端 3：规划
ros2 run uav_slam_nav main --role planner

# 终端 4：控制
ros2 run uav_slam_nav main --role controller
```

四个节点也可以合成一个进程跑（`--role all`），内部用 `MultiThreadedExecutor`，
AirSim 的 RPC 阻塞不会拖住建图和规划。

### 4.3 起飞、给目标、看地图

```bash
# 起飞（服务名在 uav 命名空间下）
ros2 service call /uav/takeoff std_srvs/srv/Trigger {}

# 在 rviz 里看地图，或直接命令行确认
ros2 topic echo /map --once --field info

# 发一个目标点（地图坐标系，单位米）
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 8.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}}"

# 看规划出的路径与导航状态
ros2 topic echo /plan --once --field poses
ros2 topic echo /nav_status

# 降落
ros2 service call /uav/land std_srvs/srv/Trigger {}
```

rviz2 里把 Fixed Frame 设成 `map`，添加 `Map`（话题 `/map`）、`Path`（话题 `/plan`）、
`TF` 三个显示项即可看到建图与规划过程。

## 5. 话题与服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav/pose` | `geometry_msgs/PoseStamped` | 发布 | 无人机位姿，AirSim 原始 NED 系（x 北 / y 东 / z 向下） |
| `/uav/image` | `sensor_msgs/Image` (bgr8) | 发布 | 前视 RGB 图 |
| `/uav/depth` | `sensor_msgs/Image` (32FC1) | 发布 | 前视深度图，单位米 |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 机体系速度（x 前 / y 右 / z 下）+ 偏航角速度 |
| `/map` | `nav_msgs/OccupancyGrid` | 发布 | 占据栅格，100 占据 / 0 自由 / -1 未知 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 订阅 | 规划目标点 |
| `/plan` | `nav_msgs/Path` | 发布 | A* 规划结果 |
| `/nav_status` | `std_msgs/String` | 发布 | 导航状态（tracking / arrived / idle） |
| `/uav/takeoff` | `std_srvs/Trigger` | 服务 | 起飞 |
| `/uav/land` | `std_srvs/Trigger` | 服务 | 降落 |

TF 树：`map -> odom -> airsim_ned`，三者在本模块中重合（`/uav/pose` 本身就是世界系位姿）。

## 6. 关键参数

完整列表与注释见 `config/params.yaml`，最常用的几个：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `airsim_ip` | `10.0.2.2` | AirSim 所在主机地址 |
| `resolution` | `0.25` | 栅格分辨率（米/格） |
| `map_size` | `50.0` | 地图边长（米） |
| `max_depth` | `15.0` | 参与建图的最远深度 |
| `inflation_radius` | `0.6` | 规划时的障碍膨胀半径 |
| `lookahead` | `1.5` | 纯追踪前视距离 |
| `cruise_height` | `3.0` | 巡航高度（米） |

## 7. 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| 日志反复打印“连接 AirSim 失败” | 宿主机 AirSim 没启动，或 `airsim_ip` 不对；VirtualBox NAT 下必须是 `10.0.2.2` |
| `/map` 一直是空的 | 没起飞、深度图没数据，或高度过滤把点全滤掉了；把 `min_height` 调到 0、`max_height` 调到 10 试试 |
| 规划返回“不可达” | 目标点在障碍里，或还没建出通路；先在 rviz 里看看 `/map`，换一个目标点 |
| 无人机原地打转 | `kp_yaw` 太大或 `lookahead` 太小；先调 `lookahead` 到 2.0 再降 `kp_yaw` |

## 8. 运行效果

![建图与自主导航演示](../../img/uav_slam_nav_demo.gif)

> 录制方式：Windows 上用 ScreenToGif 录 AirSim 窗口 + rviz 窗口，控制在 10 MB 以内，
> 放到仓库 `docs/img/uav_slam_nav_demo.gif`。
