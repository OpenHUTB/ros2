# turtlebot3_lidar_avoid

> TurtleBot3 激光雷达自主避障 ｜ **ROS 2 Humble** + **Gazebo 11**
>
> 机器人操作系统及应用课程作业 —— 传感器感知与运动控制闭环

## 模块概述

本模块在 Gazebo 中加载 TurtleBot3（Waffle）机器人，订阅其机载激光雷达，
实现未知环境下的自主避障巡航：

```
激光雷达 /scan ──▶ 扇区划分 ──▶ 有限状态机 ──▶ 势场法转向 ──▶ /cmd_vel
   (感知)          (最近距离)     (巡航/避障/紧急)   (方向+速度)    (控制)
```

与同目录其他模块的关系：`carla_keyboard_control`（CARLA 平台）、
`mujoco_car_project`（MuJoCo 平台）等模块面向各自仿真器；
本模块是 **ROS 2 + TurtleBot3** 组合下的激光雷达避障实现，
并与 `gazebo_diff_drive_car`（ROS 1 Noetic + 自建差速车）构成
ROS 1 / ROS 2 两个版本的对照。

## 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS | Humble |
| 仿真平台 | Gazebo 11 |
| 机器人 | TurtleBot3 Waffle |
| Python | 3.10 |

> [!NOTE] ROS 2 与 Gazebo 发行版兼容性说明
> - **基准环境**：本模块原生基于 **Ubuntu 22.04 + ROS 2 Humble + Gazebo 11 (Classic)** 开发与测试，节点发布标准 `geometry_msgs/msg/Twist` 控制指令。
> - **高版本（ROS 2 Jazzy / Rolling + Gazebo Harmonic）兼容提示**：在 Ubuntu 24.04（ROS 2 Jazzy）中，Gazebo 桥接节点（`ros_gz_bridge`）默认监听带有时间戳的 `geometry_msgs/msg/TwistStamped`。若在 Jazzy 环境下直接运行，需配置 bridge 桥接类型或适配为 `TwistStamped`，否则 Gazebo 无法响应纯 `Twist` 指令。

### 依赖安装

```bash
sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-gazebo-ros-pkgs
```

## 编译

本模块为 **ROS 2 (ament_python)** 包：

```bash
mkdir -p ~/ros2_ws/src
cp -r /path/to/src/ground/turtlebot3_lidar_avoid ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_lidar_avoid
source install/setup.bash
```

## 运行

### 1. 启动仿真环境

新开终端并启动 Gazebo 仿真环境（**必须先指定机器人型号环境变量**，否则会报错 `KeyError: 'TURTLEBOT3_MODEL'`）：

```bash
# 1. 设置机器人型号（可选 burger / waffle / waffle_pi）
export TURTLEBOT3_MODEL=burger

# 2. 启动标准 TurtleBot3 仿真世界场景
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

> **可选快捷方式**：若使用预置特定课程环境的虚拟机镜像，也可使用镜像自带的一键脚本（如 `~/start-gazebo-turtlebot-ros-humble.sh`）。标准独立环境中推荐使用上述官方 `ros2 launch` 命令。

### 2. 启动避障节点

```bash
# 方式一：模块入口脚本（推荐）
./main.sh
./main.sh /out/scan        # 若雷达话题为 /out/scan

# 方式二：ROS 2 launch
ros2 launch turtlebot3_lidar_avoid main.launch.py
ros2 launch turtlebot3_lidar_avoid main.launch.py scan_topic:=/out/scan

# 方式三：直接运行入口文件
python3 main.py

# 方式四：运行已注册节点
ros2 run turtlebot3_lidar_avoid avoid_node
```

> **注意雷达话题**：TurtleBot3 标准场景为 `/scan`；
> 部分课程镜像场景使用 `/out/scan`。若节点订阅不到数据，先用
> `ros2 topic list | grep scan` 确认实际名称，再通过参数指定。

### 3. 观察运行

```bash
ros2 topic hz /scan          # 雷达频率（约 5~10 Hz）
ros2 topic echo /cmd_vel     # 速度指令
ros2 node list               # 应出现 /turtlebot3_lidar_avoid
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scan_topic` | `/scan` | 激光雷达话题 |
| `cmd_vel_topic` | `/cmd_vel` | 速度指令话题 |
| `safe_distance` | 0.60 m | 进入避障的距离阈值 |
| `clear_distance` | 0.90 m | 退出避障的距离阈值（滞回） |
| `danger_distance` | 0.28 m | 进入紧急后退的阈值 |
| `cruise_speed` | 0.15 m/s | 巡航线速度 |
| `avoid_speed` | 0.08 m/s | 避障时最大线速度 |
| `back_speed` | −0.05 m/s | 紧急后退速度 |
| `min_turn` | 0.50 rad/s | 最小角速度 |
| `max_turn` | 1.20 rad/s | 最大角速度 |
| `turn_gain` | 1.50 | 左右距离差到角速度的增益 |
| `emergency_hold` | 0.8 s | 紧急状态最短保持时间 |

运行时覆盖参数：

```bash
ros2 run turtlebot3_lidar_avoid avoid_node --ros-args -p safe_distance:=0.7
```

完整参数说明见 `config/params.yaml`。

## 目录结构

```
turtlebot3_lidar_avoid/
├── main.py                            模块入口（课程 main.* 约定）
├── main.sh / main.bat                 Linux / Windows 启动脚本
├── package.xml / setup.py / setup.cfg ROS 2 包定义
├── requirements.txt                   Python 依赖说明
├── launch/main.launch.py              ROS 2 launch 文件
├── config/params.yaml                 参数配置参考
├── resource/turtlebot3_lidar_avoid    ament_index 资源索引
└── turtlebot3_lidar_avoid/
    ├── __init__.py
    └── avoid.py                       ★ 避障算法实现
```

## 算法说明

### 感知：扇区最小距离

把 360° 扫描按角度划分为三个扇区，各取最小距离：

| 扇区 | 角度范围 | 用途 |
|---|---|---|
| 前方 | −30° ~ +30° | 决定能否直行 |
| 左侧 | +30° ~ +90° | 转向方向选择 |
| 右侧 | −90° ~ −30° | 转向方向选择 |

取最小值而非均值：均值会被远处空旷区域稀释，导致对近处障碍不敏感。

### 决策：三态有限状态机

| 状态 | 进入条件 | 行为 |
|---|---|---|
| `CRUISE` | 前方 > `clear_distance` | 直行 |
| `AVOID` | 前方 < `safe_distance` | 减速并转向 |
| `EMERGENCY` | 前方 < `danger_distance` | 后退并转向 |

进入与退出避障使用不同阈值（`safe_distance` / `clear_distance`），
形成**滞回**，避免距离在阈值附近波动时状态高频切换。

### 控制：势场法转向

转向方向由左右两侧可通行度之差决定：

    Δ = 左侧距离 − 右侧距离
    ω = turn_gain × clamp(Δ / 2.0, −1, +1)

并施加两个约束：

- **最小转向力度**：`|ω| < min_turn` 时强制取 `±min_turn`，
  规避势场法的局部极小值问题（避免贴着障碍面平移而不转开）；
- **线速度衰减**：随前方障碍接近程度线性衰减，越近越慢。

## 相对初版单点避障的改进

课程实验初版只读取正前方单束（`ranges[180]`），存在明显局限：
无法感知侧向障碍，在墙角或斜向障碍场景容易卡住。本模块的改进：

| 项 | 初版 | 本模块 |
|---|---|---|
| 感知范围 | 正前方单束 | 前 / 左 / 右三扇区 |
| 转向决策 | 固定右转 | 朝较空旷一侧转 |
| 速度控制 | 固定值 | 随障碍距离衰减 |
| 状态抖动 | 无滞回 | 进入/退出双阈值 |
| 墙角脱困 | 易卡住 | 紧急后退 + 原地转向 |
| 参数 | 硬编码 | ROS 2 参数，可运行时覆盖 |

## 许可证

MIT License
