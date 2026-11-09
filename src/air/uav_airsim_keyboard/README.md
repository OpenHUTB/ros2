# uav_airsim_keyboard —— AirSim 无人机键盘遥控

用键盘遥控 AirSim 里的四旋翼，并把 AirSim 包装成标准 ROS 2 节点：位姿走话题、速度走话题、起降走服务。

- 课程任务：任务（1）物理仿真 + 键盘运动控制
- 环境：Windows 宿主机跑 AirSim Blocks，Ubuntu 22.04 + ROS 2 Humble 跑在 VirtualBox 虚拟机
- 入口：`ros2 run uav_airsim_keyboard main`，或 `ros2 launch uav_airsim_keyboard main.launch.py`

## 1. 依赖

```bash
sudo apt install python3-pip
pip3 install airsim
# ROS 2 Humble 已自带 rclpy / geometry_msgs / std_srvs
```

## 2. 编译

```bash
cd ~/ros2
colcon build --packages-select uav_airsim_keyboard --symlink-install
source install/setup.bash
```

## 3. 运行

先启动宿主机上的 `Blocks.exe`（Windows），等场景加载完、点一下窗口让它获得焦点。

```bash
# 方式一：一条命令跑整个模块（桥接 + 键盘）
ros2 run uav_airsim_keyboard main

# 方式二：launch 启动
ros2 launch uav_airsim_keyboard main.launch.py airsim_ip:=10.0.2.2

# 方式三：桥接在 launch 里，键盘另开一个终端
ros2 run uav_airsim_keyboard keyboard
```

按键：

| 键 | 动作 | 键 | 动作 |
| --- | --- | --- | --- |
| w / s | 前进 / 后退 | r / f | 上升 / 下降 |
| a / d | 左移 / 右移 | q / e | 左偏航 / 右偏航 |
| t | 起飞 | g | 降落 |

> `ros2 launch` 不会把 stdin 转发给子进程，因此 launch 起来时键盘节点会自动跳过并打印提示；需要键盘请用方式一或方式三。这是 ROS 2 的已知行为。

## 4. 话题与服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `uav/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 机体系速度，m/s，deg/s |
| `uav/pose` | `geometry_msgs/PoseStamped` | 发布 | 20 Hz，`frame_id=airsim_ned` |
| `uav/takeoff` | `std_srvs/Trigger` | 服务 | 起飞并悬停 |
| `uav/land` | `std_srvs/Trigger` | 服务 | 降落 |

## 5. 常见问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `无法连接 AirSim` | Blocks 没启动，或 IP 不对 | 先开 Blocks；VirtualBox NAT 用 `10.0.2.2`，桥接网卡用宿主机实际 IP |
| 按 t 没反应 | 服务没起来 / 还没连上 | `ros2 service list \| grep uav` 检查 |
| 一直往上飞 | NED/ENU 弄反了 | 本包已做 `vz_ned = -linear.z`，不要在参数里再取反 |
| Ctrl+C 后无人机悬停 | 进程被强杀，来不及降落 | 设了 `kill` 超时的话会少一次降落；用 `g` 或 `ros2 service call uav/land` |

## 6. 文档

完整原理、公式、算法流程与源码解析见 `docs/air/uav_airsim_keyboard.md`。
