# vehicle_pygame_teleop

物理仿真由 CARLA 0.9.16 提供；本包只生成控制指令（键盘 → 平滑 Twist，30Hz）。

## 控制链路

```text
键盘 → vehicle_pygame_teleop（指令平滑，30Hz）
     → /carla/ego_vehicle/twist                (geometry_msgs/Twist)
     → carla_twist_to_control                  (本仓库 src/ground/ros-bridge)
     → /carla/ego_vehicle/vehicle_control_cmd  (carla_msgs/CarlaEgoVehicleControl)
     → CARLA 0.9.16 中的自车（物理仿真）
```

## 与已有「手动控制」示例的区别

| 对比项 | 已有「手动控制」示例 | 本模块 `vehicle_pygame_teleop` |
| --- | --- | --- |
| 控制入口 | ros-bridge 内置手动驾驶（在 CARLA 窗口按 `B`） | 独立 ROS 节点，键盘窗口操作 |
| 指令接口 | 无对外接口 | 发布标准 `/carla/ego_vehicle/twist`，可编程、可复用 |
| 指令平滑 | 无 | 加减速与转向均平滑，车辆不跳变 |
| 退出行为 | - | ESC / 关闭窗口 / Ctrl+C 均自动发布零速度 |
| 适用场景 | 日常体验 CARLA | 作为感知、规划、端到端任务的遥控与调试基座 |

## 运行环境

- Ubuntu 20.04 + ROS Noetic（Python 3）
- pygame
- CARLA 0.9.16（运行在有 GPU 的宿主机上）
- carla ROS bridge（本仓库 `src/ground/ros-bridge`，含 `carla_twist_to_control`）

## 1. 安装依赖

```bash
sudo apt update
sudo apt install -y python3-pygame
# 或使用 pip 安装：
pip3 install pygame
```

## 2. 放入工作空间并构建

将本目录放入工作空间 `src/`（`src/ground/ros-bridge` 的 `carla_twist_to_control`、
`carla_msgs`、`ros_compatibility` 需在同一工作空间），编译：

```bash
cd ~/catkin_ws
catkin_make
```

## 3. 加载工作空间环境

```bash
source ~/catkin_ws/devel/setup.bash
```

## 4. 启动 CARLA 与 ROS bridge

1. 在 GPU 宿主机启动 CARLA 0.9.16 服务端（步骤见 `docs/set_up_and_connect_to_carla.md`）；
2. 启动 ros-bridge 与示例自车（`host` 改为宿主机 IP）：

```bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch host:=192.168.8.1 timeout:=60000 town:='Town10HD_Opt'
```

3. 启动 Twist→车辆控制转换节点：

```bash
roslaunch carla_twist_to_control carla_twist_to_control.launch
```

## 5. 启动键盘遥操作

```bash
roslaunch vehicle_pygame_teleop main.launch
```

点击窗口获取键盘焦点即可驾驶。

> 自车生成前，转换节点告警 `Did not yet receive vehicle info.` 并忽略指令。

## 按键说明

| 按键 | 功能 |
| --- | --- |
| W 或 ↑ | 油门加速（速度指令上限 10 m/s） |
| S 或 ↓ | 前进时刹车，静止时倒车（倒车指令上限 5 m/s） |
| A 或 ← | 左转 |
| D 或 → | 右转 |
| 空格 | 急停（发布零速度指令） |
| ESC / 关闭窗口 / Ctrl+C | 发布零速度后退出 |

## 话题接口

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/carla/ego_vehicle/twist`（默认，可用参数 `~twist_topic` 修改） | `geometry_msgs/Twist` | `linear.x` 速度指令（m/s），`angular.z` 转向指令（rad），发布频率 30Hz |

零 Twist → `throttle=0, brake=1`（全力刹车）。

## 目录结构

```
vehicle_pygame_teleop/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── main.launch
└── scripts/
    └── vehicle_pygame_teleop.py
```
