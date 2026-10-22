# 室内无人机测距制动与定高仿真 / Indoor UAV ranging, braking and altitude hold

## 功能 / Features

MuJoCo 纵向飞行模型保留重力、质量和碰撞。水平运动采用连续制动控制，竖直方向采用重力补偿和 PD 定高反馈。朝向固定，不模拟姿态和电机动力学。控制基线用于检验制动行为与建立学习控制的实验参照。

The longitudinal MuJoCo model retains gravity, mass and contact. Continuous horizontal braking and gravity-compensated PD altitude hold control translation. Heading is fixed; attitude and motor dynamics are outside this model. The reference controller provides a reproducible baseline for learning-based control experiments.

## 环境 / Environment

- Python 3.10–3.11；MuJoCo 3.12.0；依赖见 `requirements.txt`。 / See `requirements.txt` for dependencies.
- Windows 可运行独立物理仿真；Ubuntu 22.04 / ROS 2 Humble 用于 launch 启动。 / Standalone physics runs on Windows; launch uses Ubuntu 22.04 / ROS 2 Humble.

在模块目录执行 / Run from the module directory:

```bash
python -m pip install -r requirements.txt
python main.py --headless --output results
python main.py
```

也可从任意目录通过绝对路径运行 `main.py`，模型资源按源码位置解析。 / Absolute-path execution works from any directory; model assets resolve relative to the source.

ROS 环境中执行 / In a ROS environment:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ./launch/main.launch.py output:=/tmp/uav_braking
```

`trajectory.csv` 记录时间、间距、速度、高度和控制量，`summary.json` 汇总碰撞步数、最小间距和高度误差。 / `trajectory.csv` records time, clearance, speed, height and action. `summary.json` reports contact steps, minimum clearance and height error.

## 控制原理 / Control principle

目标速度：`v_ref = min(1.5, sqrt(2 × 0.8 × max(d − 0.8, 0)))`；加速度：`clip(3 × (v_ref − v), −2.5, 1.2)`；水平力：`mass × acceleration`。0.8 m 是减速参考间距，并非保证的最终停车间距。

Target speed follows the remaining braking distance. Acceleration is a clipped velocity-feedback command and horizontal force equals mass times acceleration. The 0.8 m parameter is a braking reference, not a guaranteed final clearance.

原路径 `src/ground/mujoco_drone_distance_stop/main.py` 继续转发至本模块。 / The original entry point forwards to this module.
