# 室内无人机测距制动与定高仿真 / Indoor UAV ranging, braking and altitude hold

本目录是独立 Python/MuJoCo 模块，不作为 ament/colcon 包构建，不支持 `ros2 run`。独立仿真可直接运行 `python main.py`；ROS 2 Humble 环境可在本目录通过 launch 文件路径启动节点。 / This is a standalone Python/MuJoCo module, not an ament/colcon package, and it does not support `ros2 run`. Run `python main.py` for standalone simulation; in ROS 2 Humble, launch the node by its file path from this directory.

## 功能 / Features

MuJoCo 纵向飞行模型保留重力、质量和碰撞。水平运动默认使用前向测距与距离差分速度进行连续制动，竖直方向采用重力补偿和 PD 定高反馈。朝向固定，不模拟姿态和电机动力学。控制基线用于检验制动行为与建立学习控制的实验参照。

The longitudinal MuJoCo model retains gravity, mass and contact. Forward ranging and finite-difference speed drive continuous horizontal braking by default; gravity-compensated PD holds altitude. Heading is fixed; attitude and motor dynamics are outside this model. The controller provides a reproducible baseline for learning-based control experiments.

## 环境 / Environment

- Python 3.10～3.11；MuJoCo 3.12.0；依赖见 `requirements.txt`。 / See `requirements.txt` for dependencies.
- Windows 可运行独立物理仿真；Ubuntu 22.04 / ROS 2 Humble 用于 launch 启动。 / Standalone physics runs on Windows; launch uses Ubuntu 22.04 / ROS 2 Humble.

在模块目录执行 / Run from the module directory:

```
python -m pip install -r requirements.txt
python main.py --headless --output results
python main.py
```

也可从任意目录通过绝对路径运行 `main.py`，模型资源按源码位置解析。 / Absolute-path execution works from any directory; model assets resolve relative to the source.

ROS 环境中执行 / In a ROS environment:

```
source /opt/ros/humble/setup.bash
ros2 launch ./launch/ros_node.launch.py controller:=mlp model:=models/mlp_controller.json
```

`ros_node.launch.py` 只推进仿真并发布 ROS 话题；`main.launch.py` 运行完整无窗口仿真并写出 `trajectory.csv`、`summary.json`。两个 launch 都支持 `python3:=`，当 MuJoCo 位于虚拟环境或 conda 时，应指定同时能导入 ROS 2 和 MuJoCo 的 Python 解释器。 / `ros_node.launch.py` steps the simulation and publishes ROS topics. `main.launch.py` runs the complete headless simulation and writes `trajectory.csv` and `summary.json`. Both launch files accept `python3:=`; when MuJoCo is installed in a virtual environment or conda, select an interpreter that can import both ROS 2 and MuJoCo.

`trajectory.csv` 记录时间、间距、速度、高度和控制量，`summary.json` 汇总碰撞步数、最小间距和高度误差。 / `trajectory.csv` records time, clearance, speed, height and action. `summary.json` reports contact steps, minimum clearance and height error.

## 控制原理 / Control principle

目标速度：`v_ref = min(1.5, sqrt(2 × 0.8 × max(d − 0.8, 0)))`；加速度：`clip(3 × (v_ref − v), −2.5, 1.2)`；水平力：`mass × acceleration`。0.8 m 是减速参考间距，并非保证的最终停车间距。

Target speed follows the remaining braking distance. Acceleration is a clipped velocity-feedback command and horizontal force equals mass times acceleration. The 0.8 m parameter is a braking reference, not a guaranteed final clearance.

## ROS 2 话题 / ROS 2 topics

节点发布以下话题：`/uav/front_range_m`（`Float32`，前向测距，米）、`/uav/braking_accel_m_s2`（`Float32`，水平控制加速度，米/秒²）和 `/uav/state`（`Float64MultiArray`，依次为仿真时间秒、传感器测距米、闭合速度米/秒和高度米）。节点运行至场景设定时长后停止发布并退出。

The node publishes `/uav/front_range_m` (`Float32`, forward range in metres), `/uav/braking_accel_m_s2` (`Float32`, horizontal control acceleration in m/s²), and `/uav/state` (`Float64MultiArray`, ordered as simulation time in seconds, sensor range in metres, closing speed in m/s, and altitude in metres). Publishing stops and the node exits when the configured scenario duration is reached.

查看状态 / Inspect the state:

```
ros2 topic echo /uav/state
```

在已安装依赖并加载 ROS 2 Humble 的环境中，可在模块目录运行真实启动回归测试：

```
python3 -m pytest -q test_ros_launch.py
```

测试分别启动 `reference`、`range`、`mlp` 三种控制器，从其他目录通过绝对 launch 路径运行节点，接收三个话题并检查状态字段、时间递增、加速度范围和 10 秒仿真结束后的进程正常退出。没有 ROS 2 的环境会跳过该项。节点在定时器回调返回后关闭 ROS，避免在回调内部执行关闭时阻塞退出。

原路径 `src/ground/mujoco_drone_distance_stop/main.py` 继续转发至本模块。 / The original entry point forwards to this module.

## 结果与文档 / Results and documentation

测距控制在初速度 0、1.5、3 m/s 的三组 10 s 场景均无碰撞，最小间距分别为 0.668362、0.668331、0.668360 m。神经控制在初速度 0 和 1.5 m/s 时均无碰撞，最小间距分别为 0.759476 m 和 0.470438 m，末速度分别为 0.015899 m/s 和 -0.056074 m/s；3 m/s 外推场景记录 4 个接触步。24 项自动化检查通过。/ Range control has no contacts in three 10 s episodes at initial speeds of 0, 1.5 and 3 m/s; respective minimum gaps are 0.668362, 0.668331 and 0.668360 m. Neural control has no contacts at 0 and 1.5 m/s, with minimum clearances of 0.759476 m and 0.470438 m and final speeds of 0.015899 m/s and -0.056074 m/s, respectively. The 3 m/s extrapolation case records four contact steps. Twenty-four automated checks pass.

完整方程、曲线和动图见[模块文档](../../../docs/air/mujoco_uav_range_neural_braking.md)。 / See the [module guide](../../../docs/air/mujoco_uav_range_neural_braking.md) for equations, curves and animation.

容器环境 / Container environment:

```
docker build -t uav-braking:humble .
docker run --rm --mount type=bind,source="$PWD",target=/work/module uav-braking:humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 launch ./launch/main.launch.py'
```

以上挂载命令在 Bash 中执行；Windows PowerShell 使用 `${PWD}` 表示当前路径。 / The mount example uses Bash; use `${PWD}` for the current path in Windows PowerShell.

## 数据与控制模式 / Data and controller modes

```
python main.py --headless --controller range --output results
python dataset.py --seed 2026 --episodes 30 --output datasets
python train.py --data datasets --output models
python evaluate.py --controller all --render --output results
python -m pytest -q
```

`range` 使用前向测距和差分闭合速度；`reference` 使用物理真值作为参照。`launch` 支持 `controller:=range`、`controller:=reference` 或 `controller:=mlp`，`model` 指定权重路径。测距无效时施加最大反向制动；此措施仅适用于当前前进接近场景，不保证任意方向运动的安全性。首帧闭合速度初始化为零，随后按 `0.8 × 原估计 + 0.2 × 距离差分` 更新。

`range` uses forward ranging and finite-difference closing speed; `reference` uses physics ground truth. Launch accepts all three controllers and a model path. Invalid ranging applies maximum reverse braking; this policy is limited to the forward-approach scene and is not a safety guarantee for arbitrary motion. Closing speed starts at zero and then updates as `0.8 × previous estimate + 0.2 × range difference / timestep`.

数据默认 30 回合、每回合 12 s，训练/验证/测试为 18/6/6 回合，21600/7200/7200 行。特征为距离和闭合速度，标签为加速度；各回合参数与完整 CSV 的 SHA-256 写入 `manifest.json`。完整数据保留在 `datasets/`，仓库的 `sample_data/` 提供字段样本及汇总。

The default dataset has 30 episodes of 12 s, split into 18/6/6 episodes and 21600/7200/7200 rows for training/validation/test. Features are range and closing speed; acceleration is the target. `manifest.json` records episode parameters and full-file SHA-256 values. Full data reside in `datasets/`; `sample_data/` contains schema samples and a summary.

训练标签来自解析测距控制基线。MLP 使用测距与闭合速度两个特征，高度由独立 PD 回路保持。验证集误差用于选择训练轮次，并在达到耐心阈值后提前停止。/ Training labels come from the analytical ranging controller. The MLP uses range and closing speed; a separate PD loop holds altitude. Validation error selects the retained training epoch and triggers early stopping.

`evaluate.py --controller all` 可重复运行几何参照、测距控制和 MLP 三种控制器在 0、1.5、3 m/s 下的闭环比较，并输出各控制器曲线与 JSON 指标。/ `evaluate.py --controller all` reproduces closed-loop comparisons for the geometric reference, ranging controller and MLP at 0, 1.5 and 3 m/s, and exports controller curves and JSON metrics.

## 大模型使用声明 / Large language model disclosure

本模块及其后续审查修订在 Codex（基于 GPT-5）辅助下完成。提交者已检查代码、测试和文档，并对最终提交内容负责。/ This module and its subsequent review revisions were completed with assistance from Codex (based on GPT-5). The submitter has reviewed the code, tests and documentation and is responsible for the final submission.
