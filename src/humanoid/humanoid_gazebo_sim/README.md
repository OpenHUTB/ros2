# 人形机器人Gazebo仿真

人形机器人在Gazebo中的物理仿真，支持键盘控制和摄像头目标追踪。

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
colcon build --packages-select humanoid_gazebo_sim
source install/setup.bash
ros2 launch humanoid_gazebo_sim humanoid_sim.launch.py
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| w/s | 前进/后退 |
| a/d | 左移/右移 |
| q/e | 左转/右转 |
| 空格 | 停止 |

## 节点说明

- `humanoid_teleop`：键盘遥控节点
- `humanoid_perception`：HSV颜色分割感知节点
- `humanoid_controller`：PID目标跟踪控制器
- `humanoid_bridge`：Gazebo物理桥接节点
