# 基于 Gazebo 雷达感知的移动机器人自主避障仿真

> **ROS 1 Noetic** + **Gazebo 11** + **Mbot**。ROS 2 Humble + TurtleBot3 Waffle 版本见 [`turtlebot3_lidar_avoid`](./ground/turtlebot3_lidar_avoid.md)。

## 1. 项目概述

本项目基于 ROS 1 Noetic 与 Gazebo 物理仿真引擎，实现移动机器人（Mbot）在未知静态环境中的自主避障。节点订阅 `/scan` 激光雷达数据，通过**扇区划分 + 阈值判断**的方式输出 `/cmd_vel` 速度指令。

与仓库中已有的 `turtlebot3_lidar_avoid`（ROS 2 Humble + TurtleBot3 Waffle 版本）相比，本模块定位为 **ROS 1 Noetic 下的精简实现**：

| 对比项 | turtlebot3_lidar_avoid | 本模块 mbot_gazebo_lidar_avoid |
| :--- | :--- | :--- |
| 中间件 | ROS 2 Humble（ament_python） | ROS 1 Noetic（catkin + rospy） |
| 机器人 | TurtleBot3 Waffle（标准仿真镜像） | Mbot（课程自建差速车） |
| 状态机 | 巡航 / 避障 / 紧急三态 + 滞回 | 直行 / 避障 / 紧急停车三态（简化） |
| 参数配置 | `params.yaml` + ROS 2 参数服务器 | 源码顶部常量，单文件易入门 |
| 工程完备性 | 含 config / setup.py / launch.py / main.sh | 单 `main.py` + `main.launch` |

> 两个模块**互补不重复**，分别服务于 ROS 1 / ROS 2 两种课程环境。本模块适用于仍在 Ubuntu 20.04 + ROS Noetic 环境下做课程作业的同学。

## 2. 环境搭建与依赖安装

### 2.1 基础环境

| 项目 | 版本 |
| :--- | :--- |
| 操作系统 | Ubuntu 20.04 |
| ROS 版本 | ROS Noetic |
| 仿真引擎 | Gazebo 11 |
| 开发语言 | Python 3 |

### 2.2 安装依赖

```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
pip3 install numpy
```

## 3. 项目文件结构

```text
src/mbot_gazebo_lidar_avoid/
├── launch/
│   └── main.launch                 # 一键启动避障节点
├── scripts/
│   └── main.py                     # 核心避障节点（含安全保护）
├── test/
│   └── test_main.py                # 异常输入单元测试（离线运行）
├── CMakeLists.txt                  # catkin 编译规则
└── package.xml                     # 包元信息
```

## 4. 运行步骤

### 4.1 编译工作空间

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4.2 启动仿真与节点

终端 1，启动 Gazebo 仿真（使用 TurtleBot3 场景示意；Mbot 场景替换为对应 launch 文件）：

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

终端 2，启动避障节点：

```bash
roslaunch mbot_gazebo_lidar_avoid main.launch
```

## 5. 核心算法

### 5.1 数据流

```
/scan (360束 LaserScan)
   │
   ▼
① NaN/Inf 过滤 + 扇区取最小有限值
   │
   ▼
② 前方 < 0.25m → 紧急停车
   前方 < 0.80m → 减速 + 朝空间宽侧转向
   其他         → 直行 0.3 m/s
   │
   ▼
/cmd_vel (geometry_msgs/Twist)
```

### 5.2 扇区划分

| 扇区 | `ranges` 索引 | 角度 | 用途 |
| :--- | :--- | :--- | :--- |
| 前方 | `[0:15]` + `[345:360]` | ±15° | 能否直行 |
| 左侧 | `[15:60]` | +15° ~ +60° | 转向方向 |
| 右侧 | `[300:345]` | −60° ~ −15° | 转向方向 |

### 5.3 源码关键片段 (`scripts/main.py`)

```python
# 每帧先过滤 NaN/Inf，某扇区全非法视为空旷安全
def _valid_min(values):
    filtered = [v for v in values if v is not None and math.isfinite(v)]
    return min(filtered) if filtered else None

# 决策三态
if front < DANGER_DISTANCE:      # < 0.25m → 紧急停车
    linear.x = 0.0; angular.z = 0.0
elif front < SAFE_DISTANCE:      # < 0.80m → 避障转向
    linear.x = 0.05
    angular.z = 1.0 if left > right else -1.0
else:                            # 直行
    linear.x = 0.30; angular.z = 0.0
```

### 5.4 安全保护

| 保护类型 | 触发条件 | 行为 |
| :--- | :--- | :--- |
| NaN/Inf 过滤 | 扫描中有 NaN / ±Inf | 跳过非法值；某扇区全部非法则视为空旷 |
| 异常长度检测 | `len(msg.ranges) != 360` | 停车并告警 |
| 全非法保护 | 三个扇区全部找不到有限值 | 停车并告警 |
| `/scan` 超时 | 1.5 秒未收到扫描 | 停车并告警（10Hz watchdog） |
| 退出停车 | `Ctrl+C` / `rosnode kill` | `rospy.on_shutdown` 发零速度 |

## 6. 实验指标

| 指标项 | 数值 |
| :--- | :--- |
| 传感器类型 | 2D 激光雷达 `/scan` |
| 避障触发安全距离 | 0.80 m |
| 紧急停车距离 | 0.25 m |
| 正常直行速度 | 0.30 m/s |
| 避障时线速度 | 0.05 m/s |
| 避障角速度 | ±1.0 rad/s |

## 7. 异常输入测试

离线单元测试（不依赖 ROS）已随仓库提供：

```bash
python3 test/test_main.py
```

覆盖的输入场景：

- 正常扫描 → 决策正确
- 全部 NaN / 全部 Inf → 返回 None（触发停车分支）
- 部分扇区 NaN / Inf → 其余扇区正常参与决策
- 扫描长度 ≠ 360 → 触发异常长度保护

## 8. 许可证

MIT License。
