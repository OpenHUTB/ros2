# carlair_ros_bridge

CarlaAir（OpenHUTB 空地一体仿真）/ AirSim 与 **ROS Noetic** 之间的无人机桥接层。

统一完成 **NED → ENU** 坐标换算，向上暴露标准 ROS 接口，使感知、规划、控制、端到端各模块
无需关心仿真器内部坐标系与 RPC 细节。

## 运行架构

```
Windows（有显卡）                      Ubuntu 20.04 虚拟机
┌────────────────────────┐  TCP 2000  ┌──────────────────────────────┐
│ OpenHUTB / CarlaAir    │◀──────────▶│ ROS Noetic                   │
│  ./CarlaAir.sh Town10HD│  TCP 41451 │  carlair_ros_bridge          │
│   · CARLA：车辆/天气/LiDAR          │   · /uav/odom  /tf           │
│   · AirSim：无人机/相机/雷达         │   · /uav/cmd_vel  /uav/goal  │
└────────────────────────┘            └──────────────────────────────┘
```

## 接口一览

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/uav/odom` | `nav_msgs/Odometry` | 位姿与速度（ENU），默认 20 Hz |
| 发布 | `/tf` | `tf2_msgs/TFMessage` | `world → base_link` |
| 发布 | `/camera/image_raw` | `sensor_msgs/Image` | 前视彩色图，`bgr8`，640×480，默认 20 Hz |
| 发布 | `/camera/depth` | `sensor_msgs/Image` | 平面深度图，`32FC1`，单位米 |
| 发布 | `/camera/seg` | `sensor_msgs/Image` | 语义分割伪彩色图，`bgr8`（默认关闭） |
| 发布 | `/lidar/points` | `sensor_msgs/PointCloud2` | XYZ float32 世界系（ENU）点云，默认 10 Hz |
| 发布 | `/uav/status` | `std_msgs/String` | `READY / HOVER / VELOCITY / GOAL / GOAL_DONE` |
| 订阅 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 速度指令，默认机体系（x 前, y 左, z 上） |
| 订阅 | `/uav/goal` | `geometry_msgs/Point` | 目标点（ENU 世界系），阻塞飞抵后悬停 |

**安全保护**：速度指令自动限幅；超过 `rate/cmd_timeout`（默认 0.5 s）没有新指令即自动悬停。

**传感器说明**：

- 三路相机图像合并为**一次** `simGetImages` RPC，避免每路一次网络往返，是 20 Hz 的关键；
- 解码只依赖 numpy：AirSim 返回的 `image_data_uint8` 是 BGRA 平面数组，去掉 alpha 即 BGR，
  正好对应 ROS 的 `bgr8`，因此虚拟机内不需要安装 OpenCV；
- `DepthPlanar`(ImageType=1) 给出的是**沿光轴的平面深度**（米），与透视深度的关系为
  `Z_planar = Z_perspective · cos θ`；
- 点云坐标系由 `settings.json` 的 `DataFrame` 决定：本包使用 `VehicleInertialFrame`
  （载具惯性系，NED 世界系），与 `/uav/odom` 同源，建图/规划无需额外外参；
  若改为 `SensorLocalFrame`，需把 `sensor/lidar_frame` 同步改为 `sensor_local`，
  桥接层会用 `LidarData.pose` 把点云变换到惯性系后再转 ENU；
- 点云可选环形滤波（`lidar/min_range`、`max_range`）与体素降采样（`lidar/voxel_leaf`）。

## 依赖安装

```bash
# 1) ROS 侧连接仿真器的 Python 客户端（纯 Python，虚拟机上装即可）
pip3 install airsim

# 2) 传感器配置放到【Windows】的 AirSim 目录（仿真器读这里，不是虚拟机）
#    Windows: C:\Users\<你>\Documents\AirSim\settings.json
cp config/settings.json  <Windows>/Documents/AirSim/settings.json
```

## 编译

```bash
cd ~/catkin_ws
cp -r <本包> src/air/
catkin_make          # 或 catkin build
source devel/setup.bash
```

## 运行

```bash
# Windows 侧先启动仿真（等待 "Ready! Both servers are running."）
./CarlaAir.sh Town10HD

# 虚拟机侧：一键启动（含环境自检）
roslaunch carlair_ros_bridge main.launch

# 只做环境自检
rosrun carlair_ros_bridge main.py _sim/host:=192.168.94.1
```

自检通过时输出：

```
==> carlair_ros_bridge 环境自检
    目标仿真器: 192.168.94.1:41451 (vehicle='Drone1')
    [通过] airsim 包可用  -- 已导入
    [通过] 连接仿真器 192.168.94.1:41451  -- confirmConnection 成功
    [通过] 读取位姿  -- pos_enu=(0.00, 0.00, -1.20) yaw=90.0°
    [通过] 相机取帧 front_rgb  -- 尺寸 640x480
    [通过] 激光雷达点云 lidar1  -- 点数 6329, x∈[-83.2, 90.5] y∈[-23.2, 93.9] z∈[-29.4, -5.8]
自检结果: 5/5 通过
```

## 验证与调试

```bash
rostopic hz /uav/odom                 # 应约 20 Hz
rostopic echo -n1 /uav/odom           # 看位姿
rostopic echo /uav/status             # 看状态机（HOVER/VELOCITY/GOAL）

# 键盘控制（任务①）
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/uav/cmd_vel

# 传感器（任务②③的数据源）
rostopic hz /camera/image_raw         # 应约 20 Hz
rostopic hz /camera/depth             # 应约 20 Hz
rostopic hz /lidar/points             # 应约 10 Hz
rostopic echo -n1 /camera/image_raw/encoding    # bgr8
rostopic echo -n1 /lidar/points/width           # 当前帧点数

# 可视化
rosrun rviz rviz        # 添加 Image(/camera/image_raw) 与 PointCloud2(/lidar/points, 颜色 Height)

# 只关掉传感器节点（省虚拟机 CPU）
roslaunch carlair_ros_bridge main.launch publish_image:=false publish_lidar:=false

# 点云降到 0.2 m 体素，便于建图与可视化
roslaunch carlair_ros_bridge main.launch voxel_leaf:=0.2

# 直接下发目标点（任务②③的基础）
rostopic pub -1 /uav/goal geometry_msgs/Point "{x: 30.0, y: 10.0, z: -8.0}"
```

> `z` 为 ENU 高度（向上为正）。示例中 `-8.0` 表示飞到 AirSim NED 的 8 m 高度。

## 常见问题

| 现象 | 原因与处理 |
|---|---|
| `连接仿真器` 失败 | Windows 侧未启动 / 主机 IP 不对。`ping 192.168.94.1`；首次启动 CarlaAir 需等 2–5 分钟 |
| 相机取帧失败 | `settings.json` 未放到 Windows 的 `Documents/AirSim/`，或相机名与配置不一致 |
| 点云为空 | `lidar1` 未启用；确认 settings.json 中 `SensorType: 6` 且 `Enabled: true` |
| 点云与里程计错位 | `settings.json` 的 `DataFrame` 与 `sensor/lidar_frame` 不一致：`VehicleInertialFrame` ↔ `vehicle_inertial`，`SensorLocalFrame` ↔ `sensor_local` |
| 图像/点云频率偏低 | 虚拟机 CPU 不足或分辨率过大；可 `voxel_leaf:=0.2` 降采样、`publish_seg:=false`，或降低 settings.json 的 Width/Height |
| RViz 报 "No tf data" | Image/PointCloud2 的 `Fixed Frame` 设为 `world`（点云）或 `base_link`（图像） |
| 无人机不响应指令 | 桥接会自动 `enableApiControl` + `armDisarm`；若手动测试需先执行这两步 |
| 位姿方向不对 | 检查是否误用 CARLA 坐标系；本包统一输出 ENU，NED→ENU 由 `sim_client` 完成 |

## 坐标系换算（核心公式）

```
位置:  ENU = C · NED,   C = [[0,1,0],[1,0,0],[0,0,-1]]
       即 x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
姿态:  R_enu = C · R_ned      （同一物理姿态在两个世界系下的表示）
速度:  v_enu = C · v_ned
```

恒等姿态（机头朝北）在 ENU 下的偏航角为 **+90°**，因为 ENU 以「东」为 0°。
单元测试见仓库 `tests/test_frames.py`（7 项，全部通过）。

## 本地测试（无需 ROS / 仿真器 / GPU）

```bash
python3 tests/test_frames.py          # 坐标换算        7 项
python3 tests/test_bridge_local.py    # 桥接状态机与工程文件  31 项
python3 tests/test_sensors_local.py   # 图像/点云与工程文件   72 项
```

`test_sensors_local.py` 用桩替代 `rospy` 与 `sensor_msgs`，覆盖：

- BGRA→BGR 解码、深度图 float32 还原、尺寸不匹配的异常分支；
- `sensor_msgs/Image` 的 `step = width × channels × itemsize` 与 `data` 长度；
- NED→ENU 轴变换、`SensorLocalFrame` 位姿还原（世界点 → 雷达局部系 → 反解回世界系）；
- 环形滤波、体素降采样、`PointCloud2` 字节布局（`point_step=12`、字段偏移 0/4/8、小端 float32）；
- 两个节点 `step_once()` 的发布行为与参数一致性。
