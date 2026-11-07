# 无信号环岛汇入仿真

本模块以 Python 单轨运动学模型模拟车辆驶入无信号环岛时与环道车流的交互，提供固定随机种子回放、键盘接管和基础安全指标。当前版本是独立二维仿真，不包含 CARLA 场景、真实传感器数据或 ROS 话题闭环。

![固定种子环岛仿真回放](assets/roundabout_demo.gif)

## 环境

- Python 3.10 或更高版本
- 图形模式需要 Tkinter
- ROS 2 Humble 工作空间可通过 `package.xml` 和 `launch/main.launch.py` 安装与启动

## 运行

在仓库根目录运行命令行仿真：

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360
```

启动键盘交互窗口：

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --gui --seed 7
```

按 W/上方向键加速，S/下方向键制动，A/D 或左右方向键转向。命令行输出运行时间、车辆状态、碰撞帧数和最小离障距离。相同随机种子可复现环道车辆的初始状态。

ROS 2 工作空间中构建并运行：

```bash
colcon build --packages-select carla_nn_roundabout_entry
source install/setup.bash
ros2 launch carla_nn_roundabout_entry main.launch.py
```

## English

This module uses a Python single-track kinematic model to simulate an unsignalized roundabout merge. It provides deterministic traffic replay, keyboard takeover, and basic safety metrics. The current implementation is a standalone 2D simulation; it does not include a CARLA scene, real sensor data, or a closed-loop ROS topic interface.

Run the headless simulation from the repository root:

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360
```

Start the keyboard-controlled window with `--gui`. Use W/Up to accelerate, S/Down to brake, and A/D or the arrow keys to steer. The command-line output reports elapsed time, vehicle state, collision frames, and minimum clearance. The same random seed reproduces the circulating traffic's initial state.

Build and launch the ROS 2 package in a workspace:

```bash
colcon build --packages-select carla_nn_roundabout_entry
source install/setup.bash
ros2 launch carla_nn_roundabout_entry main.launch.py
```
