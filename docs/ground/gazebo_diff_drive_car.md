# 地面载具物理仿真与键盘运动控制（Gazebo 版）

> **OpenHUTB ROS 机器人任务 —— 地面载具（无人车）方向**
>
> 仿真平台：**Gazebo 11** ｜ 机器人：**差速驱动无人车**
> 系统环境：Ubuntu 20.04 + ROS Noetic

---

## 本模块简介

本模块在 **Gazebo 11** 物理仿真环境中构建一台差速驱动无人车，
实现任务书要求的三个功能：

| 功能 | 任务书原文 | 实现文件 |
|---|---|---|
| **物理仿真 + 键盘运动控制** | 任务 (1)，5 分 | `main.py` + `scripts/teleop_keyboard.py` |
| **激光雷达感知 → 控制闭环** | 任务 (2)，5 分 | `scripts/laser_avoid.py` |
| **深度相机感知 → 控制闭环** | 任务 (2)，5 分 | `scripts/depth_camera_avoid.py` |

### 与同目录其他模块的关系

`src/ground/` 下的模块构成**多仿真平台对比**：

| 模块 | 仿真平台 | 说明 |
|---|---|---|
| `carla_keyboard_control` | Carla | 车辆键盘控制（Carla 平台） |
| `carla_perception_control` | Carla | 传感器感知与轨迹跟踪 |
| `mujoco_car_project` | MuJoCo | 车辆物理仿真 |
| **`gazebo_diff_drive_car`** | **Gazebo 11** | **本模块：差速驱动无人车 + 双传感器闭环** |

不同仿真平台、不同传感器的并列实现，对应任务书
「**支持多种路径规划与控制算法的对比**」的要求。

---

## 快速开始

### 1. 环境准备

```bash
sudo apt install -y \
    ros-noetic-gazebo-ros ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher ros-noetic-teleop-twist-keyboard \
    ros-noetic-xacro ros-noetic-rviz python3-numpy python3-matplotlib
```

### 2. 编译

```bash
cd ~/catkin_ws
cp -r /path/to/src/ground/gazebo_diff_drive_car src/
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 3. 运行

```bash
# 方式一：使用模块自带入口脚本
./main.sh              # 键盘遥控（默认）
./main.sh laser        # 激光雷达避障
./main.sh camera       # 深度相机避障
./main.sh laser nogui  # 无图形界面

# 方式二：直接使用 roslaunch
roslaunch gazebo_diff_drive_car main.launch mode:=teleop
roslaunch gazebo_diff_drive_car main.launch mode:=laser
roslaunch gazebo_diff_drive_car main.launch mode:=camera
```

---

## 功能演示

### 键盘运动控制

![键盘遥控模式](images/G_teleop_a.png)

*键盘遥控模式下的仿真场景。蓝色车体、黑色驱动轮、
车头红色激光雷达清晰可见。*

### 激光雷达避障

![激光雷达避障](images/H_laser_a.png)

*激光雷达避障模式。可见激光束在圆柱、方块与墙面上的回波扇形，
以及障碍物背后的扫描阴影区。*

### 深度相机避障

![深度相机避障](images/I_camera_a.png)

*深度相机避障模式运行场景。*

### 三种模式的轨迹对比

![轨迹对比](../ground/data/trajectory_plot.png)

*三种模式的轨迹对比。灰色为场地围墙，红色圆形与蓝色方块为障碍物。*

---

## 实测性能

### 三种模式对比

| 模式 | 行驶路程 | 净位移 | **路径效率** | 平均速度 | 碰撞 |
|---|---|---|---|---|---|
| `teleop`（键盘） | 6.34 m | 6.31 m | **0.997** | 0.108 m/s | — |
| `laser`（雷达） | 18.22 m | 9.07 m | **0.498** | 0.319 m/s | **0** |
| `camera`（相机） | 12.82 m | 9.73 m | **0.758** | 0.219 m/s | **0** |

**路径效率** = 净位移 / 实际路程，越接近 1 表示路径越直。

- `teleop` 效率 0.997 → 近乎完美直线，验证差速运动学正确
- `laser` 效率 0.498 → 大幅绕行，体现全向感知的激进避障
- `camera` 效率 0.758 → 中度绕行，前向视野下的保守策略

### 关键指标

| 指标 | 实测值 |
|---|---|
| `/cmd_vel` 控制频率 | **19.9 ~ 20.3 Hz**（目标 20 Hz） |
| `/scan` 激光频率 | ~10 Hz（360 光束，360° 全向） |
| `/camera/depth/image_raw` | 12.7 ~ 14.1 Hz（640×480，32FC1） |
| 仿真实时因子（无 GUI） | **0.98 ~ 1.00** |
| 仿真实时因子（有 GUI） | 0.58 ~ 0.85 |

---

## 文档目录

| 章节 | 内容 |
|---|---|
| [任务一：物理仿真与键盘运动控制](gazebo_diff_drive_car/teleop.md) | 车体模型、离地高度设计、增量式控制 |
| [任务二：激光雷达感知与避障](gazebo_diff_drive_car/laser_avoid.md) | 扇区划分、自遮挡过滤、FSM + 势场法 |
| [任务二：深度相机感知与视觉避障](gazebo_diff_drive_car/camera_avoid.md) | ROI 统计、地板值剔除、双传感器对比 |
| [系统架构](gazebo_diff_drive_car/architecture.md) | 统一入口设计、安全看门狗、TF 树 |
| [差速驱动运动学](gazebo_diff_drive_car/kinematics.md) | 正逆运动学推导、参数敏感性分析 |
| [人工势场与有限状态机](gazebo_diff_drive_car/algorithm.md) | 势场构造、局部极小值规避、滞回抑制 |
| [传感器数据处理](gazebo_diff_drive_car/sensor_processing.md) | 无效值处理、数据同步 |
| [实验设置与操作步骤](gazebo_diff_drive_car/experiments.md) | 完整可复现的操作流程 + 截图 |
| [性能评价与算法对比](gazebo_diff_drive_car/evaluation.md) | 7 项量化指标 + 可视化分析 |
| [工程问题排查记录](gazebo_diff_drive_car/troubleshooting.md) | **8 个真实 bug 的完整定位过程** |
| [场景与模型参数](gazebo_diff_drive_car/parameters.md) | 全部可调参数汇总 |

---

## 技术亮点

### 1. 识别并解决两个真实的传感器缺陷

| 问题 | 现象 | 解决方案 |
|---|---|---|
| **激光雷达自遮挡** | 正前方出现恒定近距离虚假回波（0.36 m），车辆误判被堵死 | 邻域连续性校验：真实障碍会连续遮挡 ≥5 束，自遮挡只影响孤立几束 |
| **深度图地板值** | 无回波区域输出固定常数 **1.005 m，占比达 66%**，统计量被完全锁定 | 直方图峰值检测：占比 > 30% 的深度值判定为无效标记并剔除 |

**第二个问题的隐蔽性极高** —— 1.005 m 是一个看起来完全合理的深度读数。
修复后，相机模式从「30 秒移动 0.08 m」变为「30 秒移动 7.40 m」。

### 2. 完整的安全机制

- **速度看门狗**：1 秒无新指令则强制停车，算法崩溃时车辆不会失控
- **滞回抑制**：进入避障阈值 1.0 m、退出阈值 1.4 m，消除状态抖动
- **原地点转超时保护**：避免因传感器异常导致车辆无限打转

### 3. 统一入口 + 模式调度

任务书要求「每个模块必须支持 launch 启动，入口为 `main.xxx`」。
本模块采用**单一入口 + 模式参数**设计：

```
main.sh <mode>
   └─→ main.launch
         └─→ main.py 读取 mode 参数
               └─→ 动态实例化对应算法节点
```

好处：所有功能共享同一套底盘接口与安全策略；切换功能只改参数；
算法节点崩溃不影响其他模式。

---

## 目录结构

```
gazebo_diff_drive_car/
├── main.py                          统一入口节点（选项调度 + 安全看门狗）
├── main.sh                          Linux 启动脚本
├── main.bat                         Windows 启动脚本
├── README.md                        模块说明
├── package.xml                      ROS 包定义
├── CMakeLists.txt                   编译配置
├── requirements.txt                 Python 依赖说明
│
├── launch/
│   └── main.launch                  ★ 统一启动入口
├── urdf/
│   └── car.xacro                    车辆模型（差速底盘 + 雷达 + 深度相机）
├── worlds/
│   └── car_world.world              仿真场景（12×12 m + 障碍物）
├── scripts/
│   ├── teleop_keyboard.py           键盘运动控制
│   ├── laser_avoid.py               激光雷达避障
│   ├── depth_camera_avoid.py        深度相机避障
│   ├── odom_recorder.py             轨迹记录与性能评价
│   ├── analyze_trajectory.py        轨迹分析与算法对比
│   └── plot_trajectory.py           轨迹绘图
├── config/
│   └── params.yaml                  参数配置参考
└── rviz/
    └── car.rviz                     RViz 可视化配置
```

---

## 话题接口

**订阅**

| 话题 | 类型 | 使用节点 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `laser_avoid.py` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | `depth_camera_avoid.py` |
| `/odom` | `nav_msgs/Odometry` | `odom_recorder.py` |

**发布**

| 话题 | 类型 | 发布节点 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `main.py`（经看门狗统一发布） |
| `/odom_recorder/trail` | `visualization_msgs/Marker` | `odom_recorder.py` |

**TF 树**

```
odom
 └── base_footprint
      └── base_link
           ├── left_wheel_link
           ├── right_wheel_link
           ├── front_caster_link
           ├── rear_caster_link
           ├── laser_link
           └── camera_link
                └── camera_optical_link
```

---

## 许可证

MIT License
