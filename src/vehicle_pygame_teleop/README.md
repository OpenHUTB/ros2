# vehicle_pygame_teleop

基于键盘的 ROS 1 车辆遥操作功能包：把键盘输入转换为平滑的速度指令（`geometry_msgs/Twist`，
30Hz 发布），经本仓库的 `carla_twist_to_control` 转换为车辆控制指令，控制
**CARLA 0.9.16** 模拟器中的自车运动。

> 物理仿真（车辆动力学、场景、传感器）全部由 CARLA 提供；本功能包不含任何仿真器，
> 只负责「键盘 → 控制指令」这一段，对应课程任务（1）"对车辆进行物理仿真，
> 通过键盘对车辆进行运动控制"。

## 控制链路

```text
键盘 → vehicle_pygame_teleop（指令平滑，30Hz）
     → /carla/ego_vehicle/twist                (geometry_msgs/Twist)
     → carla_twist_to_control                  (本仓库 src/ground/ros-bridge)
     → /carla/ego_vehicle/vehicle_control_cmd  (carla_msgs/CarlaEgoVehicleControl)
     → CARLA 0.9.16 中的自车（物理仿真）
```

## 功能

- W/S/A/D（或方向键）控制油门、刹车/倒车与左右转向，指令经过平滑，车辆不会瞬间跳变
- 空格键急停：立即回落并发布零速度指令（CARLA 侧表现为全力刹车）
- ESC、关闭窗口或 Ctrl+C 退出时自动发布零速度，车辆可靠停车
- 30Hz 稳定输出 Twist；话题名可配置（`~twist_topic`，默认 `/carla/ego_vehicle/twist`）
- 只依赖标准消息 `geometry_msgs/Twist`，与 `carla_twist_to_control`、
  `carla_ackermann_control` 等既有控制链路即插即用

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

将整个 `vehicle_pygame_teleop` 文件夹放入 ROS 工作空间的 `src` 目录
（例如 `~/catkin_ws/src`），并确保 `src/ground/ros-bridge` 中的
`carla_twist_to_control`、`carla_msgs`、`ros_compatibility` 也在工作空间中，
然后编译：

```bash
cd ~/catkin_ws
catkin_make
```

## 3. 加载工作空间环境

每个新终端使用本功能包前，先 source 工作空间环境（只需执行一次，建议写入 `~/.bashrc`）：

```bash
source ~/catkin_ws/devel/setup.bash
```

## 4. 启动 CARLA 与 ROS bridge

1. 在装有 GPU 的宿主机上启动 CARLA 0.9.16 服务端（Windows 双击 `CarlaUE4.exe`，
   Linux 运行 `./CarlaUE4.sh`）；详细步骤参考课程文档
   《设置并连接到 Carla 模拟器》（`docs/set_up_and_connect_to_carla.md`）。
2. 在 ROS 终端启动 ros-bridge 并生成示例自车（`host` 改为宿主机 IP）：

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

启动后弹出键盘控制窗口，**点击窗口使其获得键盘焦点**，即可开始驾驶；
CARLA 窗口中的车辆随按键实时运动。

> 提示：自车详情话题 `/carla/ego_vehicle/vehicle_info` 就绪前，转换节点会忽略
> 收到的指令并给出 "Did not yet receive vehicle info." 警告，属正常现象，
> 待 ros-bridge 生成自车后自动开始工作。

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

其中零 Twist 指令会被 `carla_twist_to_control` 解释为全力刹车（`throttle=0, brake=1`），
因此 ESC/Ctrl+C 退出与空格急停都会让车辆可靠停车。

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
