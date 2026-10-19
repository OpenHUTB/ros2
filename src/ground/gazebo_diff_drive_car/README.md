# gazebo_diff_drive_car

> Gazebo 差速驱动小车物理仿真与多传感器闭环控制 ｜ **ROS 2 Humble** + **Gazebo 11**
>
> 机器人操作系统及应用课程作业 —— 物理仿真、键盘控制与传感器感知闭环

## 模块概述

本模块在 Gazebo 中用 URDF 搭建一台差速驱动小车，实现三项功能：

| 功能 | 实现文件 |
|---|---|
| 物理仿真 + 键盘运动控制 | `gazebo_diff_drive_car/teleop_keyboard.py` |
| 激光雷达感知 → 控制闭环 | `gazebo_diff_drive_car/laser_avoid.py` |
| 深度相机感知 → 控制闭环 | `gazebo_diff_drive_car/depth_camera_avoid.py` |

三者共用同一套底盘与统一入口，通过 `mode` 参数切换。

与同目录其他模块的关系：`carla_keyboard_control`（CARLA 平台）、
`mujoco_diff_drive_slip_compensation`（MuJoCo 平台）等分别使用各自的仿真器；
本模块基于 **Gazebo + ROS 2**，与 `turtlebot3_lidar_avoid` 同属 ROS 2 栈，
区别在于本模块使用**自建差速底盘**（含雷达与深度相机两套传感器闭环）。

## 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS | Humble |
| 仿真平台 | Gazebo 11 |
| 构建系统 | colcon（ament_python） |
| Python | 3.10 |

### 依赖安装

```bash
sudo apt install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    python3-numpy
```

## 编译

```bash
mkdir -p ~/ros2_ws/src
cp -r /path/to/src/ground/gazebo_diff_drive_car ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gazebo_diff_drive_car
source install/setup.bash
```

## 运行

### 统一入口

```bash
./main.sh                 # 键盘遥控（默认）
./main.sh laser           # 激光雷达避障
./main.sh camera          # 深度相机避障
./main.sh idle            # 仅加载仿真环境
./main.sh laser nogui     # 无图形界面（服务器 / CI）
```

等价写法：

```bash
ros2 launch gazebo_diff_drive_car main.launch.py mode:=laser
ros2 run    gazebo_diff_drive_car car_main --ros-args -p mode:=laser
```

### 键盘操作

启动 `teleop` 模式后，在运行终端按键：

| 按键 | 功能 | 按键 | 功能 |
|---|---|---|---|
| `w` / `s` | 前进 / 后退 | `a` / `d` | 左转 / 右转 |
| `q` / `e` | 前进 + 左转 / 右转 | `z` / `c` | 后退 + 左转 / 右转 |
| `空格` | 紧急停止 | `x` | 退出 |
| `+` / `-` | 调整线速度上限 | | |

采用增量式控制：松开按键后车辆保持当前速度，需按空格停车。

### 观察运行

```bash
ros2 topic list                      # 应含 scan / cmd_vel / odom / camera/...
ros2 topic hz scan                   # 激光雷达约 10 Hz
ros2 topic hz cmd_vel                # 控制指令频率
ros2 topic echo cmd_vel              # 查看速度指令
```

## 话题与坐标

**订阅**

| 话题 | 类型 | 使用节点 |
|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | `laser_avoid` |
| `camera/depth/image_raw` | `sensor_msgs/Image`（32FC1） | `depth_camera_avoid` |
| `odom` | `nav_msgs/Odometry` | `odom_recorder` |

**发布**

| 话题 | 类型 | 发布者 |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | 各控制节点 |
| `odom` | `nav_msgs/Odometry` | Gazebo 差速驱动插件 |
| `scan` | `sensor_msgs/LaserScan` | Gazebo 射线传感器插件 |
| `camera/image_raw`、`camera/depth/image_raw`、`camera/points` | 相机相关 | Gazebo 相机插件 |

**TF 树**

```
odom
 └── base_footprint
      └── base_link
           ├── left_wheel_link / right_wheel_link
           ├── front_caster_link / rear_caster_link
           ├── laser_link
           └── camera_link
                └── camera_optical_link
```

## 参数

主要参数（完整见 `config/params.yaml`）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mode` | `teleop` | 运行模式 |
| `safe_distance` | 1.0 / 1.2 m | 进入避障阈值（雷达 / 相机） |
| `clear_distance` | 1.4 / 1.6 m | 退出避障阈值（滞回） |
| `danger_distance` | 0.45 / 0.5 m | 紧急后退阈值 |
| `cruise_speed` | 0.35 / 0.30 m/s | 巡航线速度 |
| `depth_percentile` | 25.0 | 深度统计百分位 |
| `floor_value_ratio` | 0.30 | 地板值判定占比 |

运行时覆盖：

```bash
ros2 run gazebo_diff_drive_car laser_avoid_node --ros-args -p safe_distance:=1.2
```

## 算法要点

### 激光雷达避障

1. **感知**：360° 扫描划分五个扇区，各取最小距离；用邻域连续性校验
   剔除车体自遮挡产生的孤立近距回波。
2. **决策**：三态状态机（巡航 / 避障 / 紧急后退），进入与退出避障用
   不同阈值形成滞回，抑制状态抖动。
3. **控制**：由左右可通行度之差决定转向（势场法），并施加最小转向力度
   约束规避局部极小值；线速度随前方距离线性衰减。

### 深度相机避障

1. **感知**：深度图分左 / 中 / 右三条带，取第 25 百分位深度；
   用直方图峰值检测剔除 Gazebo 深度相机对无回波区域输出的固定
   「地板值」（实测占比可达 66%，不清除会锁死统计结果）。
2. **决策**：与雷达版本同构的三态状态机。
3. **控制**：势场法转向 + 速度衰减 + 原地点转超时保护。

## 目录结构

```
gazebo_diff_drive_car/
├── main.py                          模块入口（课程 main.* 约定）
├── main.sh / main.bat               Linux / Windows 启动脚本
├── package.xml / setup.py / setup.cfg   ROS 2 包定义
├── requirements.txt                 Python 依赖说明
├── launch/main.launch.py            ROS 2 launch 入口
├── urdf/car.urdf.xacro              车辆模型（含 ROS 2 Gazebo 插件）
├── worlds/car_world.world           仿真场景
├── config/params.yaml               参数配置
├── rviz/car.rviz                    RViz 配置
├── resource/gazebo_diff_drive_car   ament_index 资源索引
└── gazebo_diff_drive_car/
    ├── main.py                      统一入口节点（按 mode 调度）
    ├── laser_avoid.py               激光雷达避障
    ├── depth_camera_avoid.py        深度相机避障
    ├── teleop_keyboard.py           键盘控制
    └── odom_recorder.py             轨迹记录与性能评价
```

## 许可证

MIT License
