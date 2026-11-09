# hutb_uav_keyboard_teleop

无人机（rotors_simulator firefly 六旋翼）**物理仿真 + 键盘运动控制**：键盘按键经
自定义节点映射为 `/joy` 摇杆消息，由 `rotors_joy_interface` 转发为姿态速率指令，
驱动 6 个电机插件产生刚体力/力矩，完成爬升、俯仰、滚转、偏航运动。

## 运行环境

- Ubuntu 20.04 + ROS Noetic + Gazebo 11（rotors_simulator 源码安装）
- Python 3.8，依赖 `rospy` / `sensor_msgs` / `rotors_gazebo` / `rotors_control` / `rotors_joy_interface`

## 运行步骤

```bash
# 1) 将本包放入 catkin 工作空间并编译
cd ~/catkin_ws/src && git clone <本模块> hutb_uav_keyboard_teleop
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 一键启动（Gazebo + firefly + 键盘节点 + 姿态控制器）
roslaunch hutb_uav_keyboard_teleop main.launch

# 3) 在启动该节点的终端内按键控制
```

## 键盘按键

| 按键 | 作用 |
|---|---|
| W / S | 油门升 / 降 |
| A / D | 滚转左 / 右 |
| ↑ / ↓ | 俯仰前 / 后 |
| Q / E | 偏航左 / 右 |
| C | 急停（复零） |

## 目录结构

```
launch/main.launch          一键启动
scripts/main.py             键盘遥操作节点（发布 /joy）
config/keyboard.yaml        键位映射与杆量步进参数
```
