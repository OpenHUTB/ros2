# Carla 键盘控制车辆（Pygame + HUD 增强版）

> **与 `set_up_and_connect_to_carla` 教程的差异**：
>
> - 本模块**不依赖** `carla_ros_bridge`，直接通过 Carla Python API 控制，适合 Windows 端无 ROS 环境的快速测试。
> - 提供**丰富 HUD**（速度、油门、转向、刹车实时显示），教程中无。
> - 支持**多传感器视角切换**（RGB、深度、语义分割等）。
> - 支持**一键切换天气、更换车辆模型**。
>
> 基础流程请参考 `set_up_and_connect_to_carla` 教程；本模块定位为增强客户端。

通过键盘手动控制 CARLA 仿真环境中的车辆运行，对应课程任务（1）物理仿真 + 键盘运动控制。

## 运行环境

- Windows 主机（运行 Carla 仿真器）
- Ubuntu 20.04 + Python 3.8（Conda 环境）
- Carla/OpenHUTB 2.9.16

## 依赖安装

    conda create -n carla38 python=3.8 -y
    conda activate carla38
    pip install /path/hutb-2.9.16-cp38-cp38-linux_x86_64.whl
    pip install numpy pygame

如果 `import carla` 报 `undefined symbol` 错误，需要设置：

    export LD_PRELOAD=/home/user/miniconda3/envs/carla38/lib/libc++.so.1:/home/user/miniconda3/envs/carla38/lib/libc++abi.so.1

## 运行步骤

1. Windows 主机启动 Carla 仿真器：

       cd D:\WWW\hutb_car_vr_air_mujoco
       .\CarlaUE4.exe

2. Ubuntu 虚拟机运行控制脚本：

       cd src/carla_manual_control
       python main.py --host=192.168.56.1 --port=2000

或通过 launch 启动：

       roslaunch carla_manual_control_pygame main.launch host:=192.168.56.1

## 操作说明

| 按键 | 功能 |
| W | 油门加速 |
| S | 刹车 |
| A/D | 左/右转向 |
| Q | 倒车 |
| 空格 | 手刹 |
| P | 自动驾驶 |
| ESC | 退出 |

## 运行效果

![运行效果](../img/carla_manual_control_pygame_demo.gif)

左侧 HUD 实时显示速度、油门、转向、刹车等状态。
