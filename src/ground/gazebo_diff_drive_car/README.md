# gazebo_diff_drive_car

> OpenHUTB ROS 机器人任务 —— 地面载具（无人车）方向
>
> **Gazebo 11** 差速驱动无人车物理仿真 ｜ Ubuntu 20.04 + ROS Noetic

---

## 模块概述

本模块在 **Gazebo 11** 物理仿真环境中构建一台差速驱动无人车，
完成 OpenHUTB ROS 机器人任务书中的以下要求：

| 任务书要求 | 分值 | 实现 |
|---|---|---|
| (1) 物理仿真 + 键盘运动控制 | 5 | `scripts/teleop_keyboard.py` |
| (2) 传感器感知 → 运动控制闭环（**雷达**） | 5 | `scripts/laser_avoid.py` |
| (2) 传感器感知 → 运动控制闭环（**深度相机**） | 5 | `scripts/depth_camera_avoid.py` |
| 多算法 / 多传感器对比 | 加分 | 激光与深度相机两套独立闭环 |
| 性能评价 | 加分 | `scripts/odom_recorder.py` + `scripts/analyze_trajectory.py` |

---

## 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS |
| ROS | Noetic Ninjemys |
| Gazebo | 11.13.0 |
| Python | 3.8 |
| 依赖库 | numpy, matplotlib |

### 依赖安装

```bash
sudo apt install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-xacro \
    ros-noetic-rviz \
    python3-numpy \
    python3-matplotlib
```

---

## 编译

本模块为 **catkin (ROS1)** 包。

```bash
# 1. 放入 catkin 工作区
mkdir -p ~/catkin_ws/src
cp -r /path/to/gazebo_diff_drive_car ~/catkin_ws/src/
cd ~/catkin_ws

# 2. 编译
source /opt/ros/noetic/setup.bash
catkin_make

# 3. 载入环境
source devel/setup.bash
```

> **说明**：本模块为 **ROS1 (catkin)** 包，与仓库中其他 ROS2 (ament/colcon) 模块并存。
> catkin 与 colcon 分别处理各自格式的包，互不干扰，**无需额外标记**。
>
> ⚠️ **不要**在本模块目录放置 `COLCON_IGNORE` 文件：
> 该标记会被 catkin 一并识别，导致本包**被 catkin 完全跳过而不编译**
> （表现为 `devel/share/` 下找不到本包，`devel/lib/<pkg>/` 不生成）。
> 这是本项目实际踩过的坑，详见
> `docs/ground/gazebo_diff_drive_car/troubleshooting.md`。

---

## 运行

### 方式一：模块入口脚本（推荐）

```bash
./main.sh                 # 键盘遥控模式（默认）
./main.sh laser           # 激光雷达避障
./main.sh camera          # 深度相机避障
./main.sh idle            # 仅加载仿真环境
./main.sh laser nogui     # 激光避障 + 无图形界面（服务器/CI）
```

### 方式二：roslaunch

```bash
roslaunch gazebo_diff_drive_car main.launch mode:=teleop
roslaunch gazebo_diff_drive_car main.launch mode:=laser
roslaunch gazebo_diff_drive_car main.launch mode:=camera
```

### 可选参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mode` | `teleop` | `teleop` / `laser` / `camera` / `idle` |
| `gui` | `true` | 是否启动 Gazebo 图形界面 |
| `rviz` | `false` | 是否启动 RViz |
| `x` `y` `yaw` | `-5.0` `-4.0` `0.0` | 车辆初始位姿 |
| `record` | `true` | 是否记录轨迹并输出性能报告 |

---

## 键盘操作

启动 `mode:=teleop` 后，在运行终端按键：

| 按键 | 功能 | 按键 | 功能 |
|---|---|---|---|
| `w` | 前进 | `s` | 后退 |
| `a` | 左转 | `d` | 右转 |
| `q` | 前进 + 左转 | `e` | 前进 + 右转 |
| `z` | 后退 + 左转 | `c` | 后退 + 右转 |
| `空格` | 紧急停止 | `x` | 退出程序 |
| `+` | 提高速度上限 | `-` | 降低速度上限 |

> 采用**增量式**速度控制：松开按键后车辆保持当前速度，需按空格停车。

---

## 实测性能

### 三种模式对比

| 模式 | 行驶路程 | 净位移 | 路径效率 | 平均速度 | 碰撞 |
|---|---|---|---|---|---|
| `teleop` | 6.34 m | 6.31 m | **0.997** | 0.108 m/s | — |
| `laser` | 18.22 m | 9.07 m | **0.498** | 0.319 m/s | **0** |
| `camera` | 12.82 m | 9.73 m | **0.758** | 0.219 m/s | **0** |

### 关键指标

| 指标 | 实测值 |
|---|---|
| `/cmd_vel` 频率 | 19.9 ~ 20.3 Hz |
| `/scan` 频率 | ~10 Hz（360 束） |
| 深度图频率 | 12.7 ~ 14.1 Hz（640×480） |
| 实时因子（无 GUI） | 0.98 ~ 1.00 |

---

## 目录结构

```
gazebo_diff_drive_car/
├── main.py                  统一入口节点
├── main.sh                  Linux 启动脚本
├── main.bat                 Windows 启动脚本
├── package.xml              ROS 包定义
├── CMakeLists.txt           编译配置
├── requirements.txt         Python 依赖
├── launch/main.launch       统一启动入口
├── urdf/car.xacro           车辆模型
├── worlds/car_world.world   仿真场景
├── scripts/                 算法节点
├── config/params.yaml       参数配置
└── rviz/car.rviz            RViz 配置
```

---

## 关键设计

### 统一入口 + 模式调度

```
main.sh <mode>  →  main.launch  →  main.py（读取 mode）
                                        ↓
                              动态实例化算法节点
```

所有功能共享同一套底盘接口与安全策略（速度看门狗）。

### 安全机制

- **速度看门狗**：1 秒无新指令则强制停车，算法崩溃时车辆不会失控
- **滞回抑制**：进入避障阈值 1.0 m / 退出 1.4 m，消除状态抖动
- **原地点转超时保护**：避免传感器异常导致车辆无限打转

### 传感器缺陷处理

本模块在开发中识别并解决了两个真实的仿真传感器缺陷：

1. **激光雷达自遮挡** → 邻域连续性校验过滤孤立近距回波
2. **深度图地板值**（无回波区域输出固定常数 1.005 m，占比 66%）
   → 直方图峰值检测并剔除

详见 `docs/ground/gazebo_diff_drive_car/troubleshooting.md`。

---

## 相关文档

完整技术文档位于仓库 `docs/ground/gazebo_diff_drive_car/`，包括：

- 算法原理与公式推导
- 源码逐模块解析
- 完整实验步骤与截图
- 性能评价与算法对比
- 工程问题排查记录

---

## 许可证

MIT License
