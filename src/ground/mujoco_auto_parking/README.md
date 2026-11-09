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

## 任务②：相机感知 + 给定轨迹控制

启动仿真、视觉感知节点和神经网络轨迹控制节点：

```bash
./main.sh --task2
```

| 节点 | 输入 | 输出 |
| --- | --- | --- |
| `simulation_node` | `/cmd_vel` | `/odom`、`/clock`、`/tf`、`/camera/image_raw`、`/camera/depth`、`/scan` |
| `perception_node` | `/camera/image_raw` | `/parking/perception` = `[dx, dy, dtheta, confidence]` |
| `trajectory_control_node` | `/odom` | `/cmd_vel`，沿给定 S 曲线跟踪 |

训练与测试：

```bash
./main.sh --train            # 重新训练感知网络和轨迹控制网络
./main.sh --test-sensors     # 相机/深度/激光雷达/感知网络
./main.sh --test-nn          # 感知精度、控制网络精度、零权值消融
./main.sh --eval-trajectory  # 闭环 S 曲线跟踪评估
```

实测（2026-10-18，WSL Ubuntu 22.04 + ROS2 Humble + MuJoCo 3.12）：

- 视觉感知（80 个测试位姿）：MAE dx 0.34 m、dy 0.20 m、dtheta 0.05 rad；
- 视觉感知零权值消融：dtheta 误差 2.58 rad；
- 轨迹控制网络与几何专家相关系数：v 0.9989、wz 0.9995；
- 闭环 S 曲线：训练网络 RMSE 0.359 m 并到达终点；零权值 RMSE 0.704 m 且未到达。

权值文件：`models/perception.npz`、`models/trajectory.npz`；训练脚本：`train/train_perception.py`、`train/train_trajectory.py`。

## 任务③：建图 + Neural A* + 导航闭环

```bash
./main.sh --task3
# 或指定目标（上排/下排车位入口）：
ros2 launch mujoco_auto_parking task3.launch.py goal:=0.25,-1.2
```

| 节点 | 输入 | 输出 |
| --- | --- | --- |
| `simulation_node` | `/cmd_vel` | `/odom`、`/scan`、`/clock`、`/tf`、相机话题 |
| `navigation_node` | `/odom`、`/scan` | `/map`、`/global_path`、`/cmd_vel` |

- `navigation_node` 用 log-odds 占据栅格建图，Neural A* 规划到目标车位入口，再用任务②的神经网络跟踪路径；
- 训练引导网络：`./main.sh --train-planning`；
- 测试：`./main.sh --test3`。

实测（2026-10-18）：

- 上/下两个车位入口目标都在约 14.3 s 到达，终点误差约 0.34 m；
- 建图覆盖率约 55%，Neural A* 单次规划约 300 ms，展开节点约 420；
- 引导场训练验证相关系数 0.99993。

## 任务④：图像端到端控制

```bash
./main.sh --task4
```

- `end_to_end_node` 只订阅 `/camera/image_raw`，直接发布 `/cmd_vel`，不读取里程计或任何手工特征；
- 训练：`./main.sh --train-e2e`（用任务②的神经网络专家采集 5 条 S 曲线演示，训练小 CNN）；
- 测试：`./main.sh --test4`。

实测（2026-10-18，闭环 S 曲线）：

- 图像→控制 CNN：RMSE 0.355 m，最大误差 0.549 m，到达终点，推理约 1.9 ms；
- 零权值消融：RMSE 0.684 m，最大误差 1.001 m；
- 与专家动作相关系数：v 0.9978、wz 0.9994。

## 当前范围 / Scope

PR1：任务①物理仿真、停车场场景、键盘控制和基础 ROS 接口。
PR2：任务②相机/深度/激光雷达、视觉车位感知网络、给定 S 曲线的神经网络控制、训练和零权值消融。
PR3：任务③log-odds 占据栅格建图、Neural A* 规划、神经网络路径跟踪和导航闭环。
PR4：任务④前视图像→`[速度, 转向]` 端到端 CNN、训练/测试/ROS 封装和闭环消融。
后续 PR：MkDocs/GitHub Pages、四篇任务文档、GIF 和 Word 导出。
