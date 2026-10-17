# 无人机Gazebo仿真

四旋翼无人机在Gazebo中的物理仿真，支持键盘控制和摄像头目标追踪。

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
colcon build --packages-select drone_gazebo_sim
source install/setup.bash
ros2 launch drone_gazebo_sim drone_sim.launch.py
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| w/s | 前进/后退 |
| a/d | 左移/右移 |
| q/e | 上升/下降 |
| 1/2 | 左旋/右旋 |
| 空格 | 悬停 |

## 节点说明

- `drone_teleop`：键盘遥控节点
- `drone_perception`：HSV颜色分割感知节点
- `drone_controller`：PID目标跟踪控制器
- `drone_bridge`：Gazebo物理桥接节点
