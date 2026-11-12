# 水下机器人 MuJoCo 仿真 — ROS2 功能包

## 概述

本功能包将 MuJoCo 物理引擎中的水下机器人（ROV + UR5e 机械臂）仿真封装为 ROS2 节点，支持通过标准 ROS2 话题进行控制和数据获取。

## 环境要求与架构说明

> **重要提示**：本功能包为 **ROS 2 (ament_python)** 架构，ROS 节点与 Launch 流程依赖 `rclpy`，需要使用 `colcon build` 构建（推荐 **Ubuntu 22.04 + ROS 2 Humble**）。
> 若在 ROS 1 Noetic 或无 ROS 虚拟机环境下，可直接脱离 ROS，以纯 Python 方式运行离线控制算法与神经网络策略推理评测（仅需 Python 3.8+ 与 PyTorch）。

- **完整仿真与 ROS 2 节点环境**：
  - Ubuntu 22.04 + ROS 2 Humble (桌面版)
  - Python 3.10+
  - MuJoCo 3.12.0 (`pip install mujoco`)
  - WSL/无显示环境：运行前设置 `export MUJOCO_GL=osmesa`；需要窗口时使用 `LIBGL_ALWAYS_SOFTWARE=1 MUJOCO_GL=glfw`
- **离线算法与纯网络评测环境（无需 ROS / 无需 GPU）**：
  - Ubuntu 20.04/22.04 或任意 Linux/WSL 环境
  - Python 3.8+
  - PyTorch (CPU 版即可，`pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`)
  - NumPy, SciPy

## 安装

```bash
# 进入 ROS2 工作空间
cd ~/ros2_ws/src

# 复制本功能包
git clone https://github.com/OpenHUTB/ros2.git

# 编译
cd ~/ros2_ws
colcon build --packages-select rov_mujoco --symlink-install
source install/setup.bash
```

## 运行

### 6-DOF 键盘运动控制

```bash
ros2 launch rov_mujoco main.launch.py
```

### 键盘操作

| 按键 | 功能 |
|------|------|
| ↑ / ↓ | 前进 / 后退 |
| ← / → | 原地左转 / 右转 |
| W/S | 前进/后退 |
| A/D | 左移/右移 |
| Q/E | 上浮/下潜 |
| J/L | 左偏航/右偏航 |
| I/K | 前俯仰/后俯仰 |
| U/O | 左翻滚/右翻滚 |
| 数字键 1-9 | 与方向键/字母键等价（九宫格布局） |
| 空格 / 0 / 5 | 急停 |
| +/- | 调节推力比例 |

## ROS2 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令（订阅） |
| `/rov/odom` | `nav_msgs/Odometry` | 里程计（发布） |
| `/rov/depth` | `std_msgs/Float64` | 深度（发布） |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态（发布） |

## 离线回归评测（无需 ROS / MuJoCo）

轨迹跟踪子系统与端到端视觉策略网络均支持脱离 ROS 单独跑：

### 1. 轨迹跟踪离线回路评测
`TrajectoryGenerator` 的 ROS 消息按需导入，NN / LOS 控制器与一阶速度响应代理组合成闭环，只依赖 `numpy` 和 `matplotlib`。

```bash
python3 eval_offline.py --duration 60
python3 eval_offline.py --controllers los --trajectories lawnmower --duration 40
python3 -m unittest discover -s test -p "test_offline_eval.py" -v
```

结果写入 `results/offline_eval.json` 与 `results/offline_tracking.png`。代理模型不是 MuJoCo 水动力，指标只用于回归和相对比较，详见 `docs/water/rov_trajectory_tracking.md` 第 7.3 节。

### 2. 端到端视觉策略网络离线单元测试 (PyTorch CPU)
无需 ROS 2 / 无需仿真器，直接断言三大网络拓扑结构（Nature-CNN / ResNet-18 / Spatial-Softmax）、参数量（1,748,260 / 713,540 / 56,452）、前向输出动作范围及零权重回退机制：

```bash
python3 -m unittest discover -s test -p "test_task4_offline.py" -v
# 或使用 pytest:
pytest test/test_task4_offline.py -v
```

### 3. 端到端网络离线训练工具 (行为克隆 BC)
支持通过命令行灵活指定架构与超参数：

```bash
python3 rov_mujoco/train_end_to_end.py --help
# 训练指定架构 (如 Spatial-Softmax)
python3 rov_mujoco/train_end_to_end.py --arch spatial_softmax --epochs 15
```
