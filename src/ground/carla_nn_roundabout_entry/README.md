# 无信号环岛汇入仿真

本模块以 Python 单轨运动学模型模拟车辆驶入无信号环岛时与环道车流的交互，提供固定随机种子回放、键盘接管、雷达间隙特征、神经网络策略控制、观测栅格与图像控制。策略、地图更新和图像控制均可在无界面仿真循环中运行；当前不包含 CARLA 场景、真实传感器数据或 ROS 话题闭环。

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

选择控制器：

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360 --controller gap-policy
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360 --controller baseline
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360 --controller image-model
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

This module uses a Python single-track kinematic model to simulate an unsignalized roundabout merge. It provides deterministic traffic replay, keyboard takeover, radar gap features, neural policy control, an observation grid, and image control. The policy, map update, and image control can run in the headless simulation loop; the current implementation does not include a CARLA scene, real sensor data, or a closed-loop ROS topic interface.

Run the headless simulation from the repository root:

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360
```

Select the controller explicitly with `--controller gap-policy`, `--controller image-model`, or `--controller baseline`.

Start the keyboard-controlled window with `--gui`. Use W/Up to accelerate, S/Down to brake, and A/D or the arrow keys to steer. The command-line output reports elapsed time, vehicle state, collision frames, and minimum clearance. The same random seed reproduces the circulating traffic's initial state.

Build and launch the ROS 2 package in a workspace:

```bash
colcon build --packages-select carla_nn_roundabout_entry
source install/setup.bash
ros2 launch carla_nn_roundabout_entry main.launch.py
```
