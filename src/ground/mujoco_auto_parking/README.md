# MuJoCo 四轮差速车自动泊车 / MuJoCo Auto Parking

本 ROS 2 功能包在 MuJoCo 3.12 中搭建一个停车场场景，使用四轮差速小车完成任务①：物理仿真 + 键盘运动控制。后续 PR 会加入传感器、神经网络感知/规划/控制、建图导航和端到端泊车。

## 环境 / Environment

| 组件 / Component | 版本 / Version |
| --- | --- |
| Ubuntu | 22.04（本项目在 WSL2 中实测） |
| ROS 2 | Humble |
| Python | 3.10 |
| MuJoCo | 3.12.0 |

## 目录 / Layout

```text
mujoco_auto_parking/
├── main.py                 # 课程主入口
├── main.sh                 # 一键运行（设置 ROS/MuJoCo 环境）
├── launch/main.launch.py   # 仿真 + 键盘节点
├── config/simulation.yaml  # 仿真参数
├── models/parking_lot.xml  # 停车场 + 差速车 + 相机/激光雷达安装点
├── mujoco_auto_parking/
│   ├── kinematics.py
│   ├── keyboard_teleop_node.py
│   ├── resources.py
│   ├── simulation_node.py
│   └── simulator.py
└── test/
```

## 构建与运行 / Build and Run

```bash
# 把本包放进 ROS 2 工作空间的 src/ 下
cd ~/auto_parking_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_auto_parking --symlink-install
source install/setup.bash

cd src/ground/mujoco_auto_parking
./main.sh --launch
```

发送速度指令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}"
```

## 键盘 / Keyboard

| 按键 | 动作 |
| --- | --- |
| W / ↑ | 前进 |
| S / ↓ | 后退 |
| A / ← | 左转 |
| D / → | 右转 |
| 空格 / 0 | 急停 |
| Q / Esc | 退出 |

键盘节点需要真实终端；由 launch 启动时如果没有 TTY，会只发布零速度。可以在另一个终端运行：

```bash
ros2 run mujoco_auto_parking keyboard_teleop_node
```

## 话题 / Topics

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 车体速度 |
| 发布 | `/odom` | `nav_msgs/msg/Odometry` | MuJoCo 真值状态 |
| 发布 | `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟 |
| 发布 | `/tf` | `tf2_msgs/msg/TFMessage` | `odom` → `base_link` |

## 测试 / Test

```bash
cd src/ground/mujoco_auto_parking
./main.sh --test
```

测试内容：模型对象（差速车、前视相机、激光雷达安装点、目标车位）、前进/后退/转向位移、键盘映射、Watchdog 超时。

## 当前范围 / Scope

PR1 只包含任务①：物理仿真、停车场场景、键盘控制和基础 ROS 接口。相机/激光雷达数据发布、车位感知、神经网络规划与控制、建图导航和端到端泊车在后续 PR 中加入。
