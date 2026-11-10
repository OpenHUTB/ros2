# uav_physical_simulation — 无人机物理仿真与键盘运动控制

对应课程任务（1）：*对无人机进行物理仿真，通过键盘对机器人进行运动控制*。

详细文档：[docs/air/uav_physical_simulation.md](../../../docs/air/uav_physical_simulation.md)

## 功能

把终端按键转换为**机体系速度指令**，发布到 `/uav/cmd_vel`，
由 `carlair_ros_bridge` 连仿真器执行。键盘、规划器、端到端策略**共用同一条指令通道**。

## 控制链路

```text
按键 ──► uav_physical_simulation ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir/AirSim
        (raw 模式读键)            (geometry_msgs/Twist)  (坐标换算+执行)
```

## 按键映射

机体系（x 前 / y 左 / z 上）：

| 按键 | 动作 | Twist 分量 |
|---|---|---|
| `W` / `S` | 前进 / 后退 | `linear.x = ±horiz_speed` |
| `A` / `D` | 左移 / 右移 | `linear.y = ±horiz_speed` |
| `R` / `F` | 上升 / 下降 | `linear.z = ±vert_speed` |
| `Q` / `E` | 右转 / 左转 | `angular.z = ±yaw_rate` |
| 松开 | 悬停 | 全零 |
| `ESC` | 退出 | — |

## 运行环境

| 组件 | 版本 |
|---|---|
| 仿真器 | CarlaAir / AirSim（OpenHUTB v2.10.0） |
| 操作系统 | Ubuntu 20.04（虚拟机） |
| ROS | ROS 1 Noetic |
| Python | 3.8+（仅独立模式需要 `airsim` 客户端） |

## 依赖安装

```bash
# ROS 模式不需要额外依赖（只用 rospy / geometry_msgs）
# 独立模式需要 airsim 客户端：
pip3 install airsim
```

## 编译

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/OpenHUTB/ros2.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

## 运行

> **启动顺序**：桥接必须先起（它负责连仿真器并自动起飞悬停），键盘节点后起。
> 反序的话，键盘节点在桥接连上之前发布的指令会被丢弃。

```bash
# 终端 1：桥接（连仿真器 + 执行 + 自动起飞悬停）
source ~/catkin_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch publish_image:=false

# 终端 2：键盘控制
source ~/catkin_ws/devel/setup.bash
roslaunch uav_physical_simulation main.launch
```

可调参数：

```bash
roslaunch uav_physical_simulation main.launch horiz:=1.5 vert:=0.8 yaw:=0.8 dt:=0.1
```

把焦点放在键盘控制终端，按 `W/A/S/D/R/F/Q/E` 操控，松开即悬停，`ESC` 退出。

### 独立模式（不经 ROS，快速验证）

```bash
python3 scripts/main.py --host 192.168.8.1
```

独立模式会自行完成使能 API 控制、解锁、起飞悬停这些通常由桥接完成的步骤。

## 实现要点

* **`key_to_cmd` 是纯函数**：不依赖 ROS/终端，可在无仿真环境的机器上单元测试；
  独立模式与 ROS 模式共同调用它，保证两条路径行为一致。
* **终端 raw 模式**：Linux 终端默认行缓冲，用 `termios`+`tty` 进入 raw 模式实现单键即时响应；
  `with` 语句保证异常退出也能恢复终端设置。
* **固定周期发布**：以 10 Hz 持续发布（含零速度）。桥接层有「超时无指令自动悬停」保护，
  稀疏发布会触发该保护导致间断性掉高度。
* **控制周期须与桥接一致**：`dt` 必须等于 `carlair_ros_bridge` 的 `control/cmd_duration`（0.1 s）。

## 文件结构

```text
uav_physical_simulation/
├── CMakeLists.txt
├── package.xml
├── config/keyboard.yaml          # 速度上限、控制周期
├── launch/main.launch            # 课程约定主入口
└── scripts/
    ├── main.py                   # 主入口（ROS / 独立双模式）
    └── keyboard_control.py       # 按键映射纯函数 + raw 模式读取
```
