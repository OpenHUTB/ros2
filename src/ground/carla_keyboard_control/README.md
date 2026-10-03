# CARLA 0.9.16 地面载具物理仿真与键盘运动控制 — ROS 2 功能包

## 概述

本功能包在 **CARLA 0.9.16** 中对地面载具（`vehicle.tesla.model3`）进行物理仿真，
通过键盘实时控制车辆前进、转向、刹车与倒车，并把前视 RGB 相机画面显示在窗口中。
对应课程任务（1）：*对车辆进行物理仿真，通过键盘对车辆进行运动控制*。

## 与已有「手动控制」示例的区别

本仓库已有 [`set_up_and_connect_to_carla`](https://openhutb.github.io/ros2/set_up_and_connect_to_carla/)
示例（基于 **carla-ros-bridge** 的手动驾驶）。本模块与它的区别如下：

| 对比项 | 已有「手动控制」示例 | 本模块 `carla_keyboard_control` |
|---|---|---|
| 技术路线 | `carla_ros_bridge` + 内置手动驾驶（按 `B` 进入） | CARLA Python API 直连自车 + 自研键盘控制节点 |
| 控制对象 | ros-bridge 的 example ego vehicle | 自生成的 ego vehicle（可配蓝图/出生点） |
| 控制方式 | 车辆内置手动驾驶逻辑 | 键盘 → `VehicleControl(throttle, steer, brake, reverse)`，含**倒挡判定** |
| 传感器展示 | RViz 中订阅 ros-bridge 话题 | pygame 前视画面 + HUD（速度/控制量/键位）实时叠加 |
| 运行依赖 | 必须编译并运行 ros-bridge（catkin） | 仅需 `carla` Python 客户端，**无需 ros-bridge** |
| 适用场景 | 学习 ros-bridge 话题体系 | 作为感知/规划/端到端作业的自车控制基座 |

> 二者**配置步骤不重复**：CARLA 服务端启动、宿主机 IP 与端口 2000 的查看、虚拟机网络设置等
> 通用步骤请直接参考
> [设置并连接到 Carla 模拟器](https://openhutb.github.io/ros2/set_up_and_connect_to_carla/)，
> 本模块文档不再重复，仅补充本模块特有的运行方式。

## 环境要求

- CARLA 0.9.16（服务端运行于有 GPU 的宿主机）
- 操作系统：Windows 10/11 原生；Ubuntu 20.04 / 22.04
- Python **3.10+**（独立模式；与 CARLA 0.9.16 客户端 wheel 的 cp310/cp311/cp312 对应）
- ROS（可选）：ROS 2 Humble（Ubuntu 22.04，Python 3.10）或 ROS 1 Noetic（Ubuntu 20.04，Python 3.8）
- Python 依赖：`numpy`、`pygame`、`carla` 客户端（见 `requirements.txt`）

> **Python 版本提示**：CARLA 0.9.16 的客户端 wheel 只提供 cp310/cp311/cp312。
> ROS 2 Humble（Ubuntu 22.04）自带的 Python 3.10 可直接匹配；ROS 1 Noetic
> （Ubuntu 20.04）自带 Python 3.8，需要另外指定 3.10+ 解释器，并保证 `pip` 与
> 运行 `main.py` 用的是同一个解释器。

## 安装

```bash
# 1. 安装 Python 依赖与 CARLA 客户端
pip3 install -r src/ground/carla_keyboard_control/requirements.txt
pip3 install carla==0.9.16          # 自动匹配 Python 版本（也可用 CARLA 包内 wheel）

# 2. 编译本功能包（ROS 2）
cd ~/ros2_ws
colcon build --packages-select carla_keyboard_control --symlink-install
source install/setup.bash
```

## 运行

```bash
# 独立模式（单进程直连 CARLA，pygame 窗口控制；虚拟机填宿主机 IP）
python3 src/ground/carla_keyboard_control/main.py --host 192.168.8.1 --follow

# ROS 2 Humble 节点模式
ros2 launch carla_keyboard_control main.launch.py host:=192.168.8.1

# ROS 1 Noetic
roslaunch carla_keyboard_control main.launch host:=192.168.8.1
```

## 操作键位

| 按键 | 作用 |
|---|---|
| `W` / `↑` | 油门加速 |
| `S` / `↓` | 刹车；车速低于阈值时自动挂**倒挡**后退 |
| `A` `D` / `←` `→` | 左 / 右转向 |
| `Q` `E` | 转向微调 |
| `ESC` | 退出 |

## ROS 2 话题接口

| 话题 | 类型 | 方向 |
|---|---|---|
| `/carla/ego_vehicle/vehicle_control_cmd` | `std_msgs/Float32MultiArray` | 订阅（[throttle, steer, brake, reverse]） |
| `/carla/ego_vehicle/rgb_front/image` | `sensor_msgs/Image` | 发布（前视相机 `rgb8`） |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | 发布（位姿 + 速度） |
| `/carla/ego_vehicle/speed` | `std_msgs/Float32` | 发布（速率 m/s） |

## 目录结构

```
carla_keyboard_control/
├── main.py / main.sh / main.bat      # 课程约定主入口（standalone / --launch）
├── carla_keyboard_control/
│   ├── carla_common.py               # CARLA API 公共封装
│   ├── carla_control_node.py         # ROS 2 仿真节点（自车/相机/同步步进）
│   └── keyboard_teleop_node.py       # ROS 2 键盘遥控节点（pygame/终端双后端）
├── launch/ main.launch.py | main.launch   # ROS 2 Humble / ROS 1 Noetic
├── config/sim_params.yaml            # 仿真参数
├── resource/  setup.py  setup.cfg  package.xml   # ament 打包
└── test/test_control_logic.py        # 单元测试（无需 CARLA 服务端）
```
