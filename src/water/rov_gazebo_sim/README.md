# 水下ROV Gazebo仿真

水下机器人ROV在Gazebo中的物理仿真，支持键盘控制和摄像头目标追踪。

## 运行环境

- 操作系统：Ubuntu 20.04
- ROS版本：ROS2 Foxy
- Python版本：3.8
- 仿真器：Gazebo 11

## 依赖安装

```bash
sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-cv-bridge python3-opencv
```

## 编译运行

```bash
cd ~/ros2_ws
colcon build --packages-select rov_gazebo_sim
source install/setup.bash
ros2 launch rov_gazebo_sim rov_sim.launch.py
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| w/s | 前进/后退 |
| a/d | 左移/右移 |
| q/e | 下潜/上浮 |
| 1/2 | 左旋/右旋 |
| 空格 | 紧急悬停 |

## 节点说明

- `rov_teleop`：键盘遥控节点
- `rov_perception`：HSV颜色分割感知节点
- `rov_controller`：PID目标跟踪控制器
- `rov_bridge`：Gazebo物理桥接节点
