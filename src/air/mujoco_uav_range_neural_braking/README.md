# 室内无人机测距制动与定高仿真 / Indoor UAV ranging, braking and altitude hold

## 功能 / Features

MuJoCo 纵向飞行模型保留重力、质量和碰撞。水平运动默认使用前向测距与距离差分速度进行连续制动，竖直方向采用重力补偿和 PD 定高反馈。朝向固定，不模拟姿态和电机动力学。控制基线用于检验制动行为与建立学习控制的实验参照。

The longitudinal MuJoCo model retains gravity, mass and contact. Forward ranging and finite-difference speed drive continuous horizontal braking by default; gravity-compensated PD holds altitude. Heading is fixed; attitude and motor dynamics are outside this model. The controller provides a reproducible baseline for learning-based control experiments.

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

## 结果与文档 / Results and documentation

测距控制在初速度 0、1.5、3 m/s 的三组 10 s 场景均无碰撞，最小间距分别为 0.668362、0.668331、0.668360 m。16 项物理、传感器、数据与 MLP 检查通过；ROS 2 Humble 容器中的测距节点成功初始化并完成仿真步进。 / Range control has no contacts in three 10 s episodes at initial speeds of 0, 1.5 and 3 m/s; respective minimum gaps are 0.668362, 0.668331 and 0.668360 m. Sixteen physics, sensor, dataset and MLP checks pass. The ranging node initializes and advances the simulation in a ROS 2 Humble container.

完整方程、曲线和动图见[模块文档](../../../docs/air/mujoco_uav_range_neural_braking.md)。 / See the [module guide](../../../docs/air/mujoco_uav_range_neural_braking.md) for equations, curves and animation.

容器环境 / Container environment:

```bash
docker build -t uav-braking:humble .
docker run --rm --mount type=bind,source="$PWD",target=/work/module uav-braking:humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 launch ./launch/main.launch.py'
```

以上挂载命令在 Bash 中执行；Windows PowerShell 使用 `${PWD}` 表示当前路径。 / The mount example uses Bash; use `${PWD}` for the current path in Windows PowerShell.

## 数据与控制模式 / Data and controller modes

```bash
python main.py --headless --controller range --output results
python main.py --headless --controller reference --output results/reference
python dataset.py --seed 2026 --episodes 30 --output datasets
python evaluate.py --controller range --render --output results/range
python -m pytest -q
```

`range` 使用前向测距和差分闭合速度；`reference` 使用物理真值作为参照；`mlp` 加载 `models/mlp_controller.json` 的 NumPy MLP。`launch` 支持三种控制器。测距无效时施加最大反向制动；此措施仅适用于当前前进接近场景，不保证任意方向运动的安全性。首帧闭合速度初始化为零，随后按 `0.8 × 原估计 + 0.2 × 距离差分` 更新。

`range` uses forward ranging and finite-difference closing speed; `reference` uses physics ground truth; `mlp` loads the NumPy model in `models/mlp_controller.json`. Launch accepts all three controller modes. Invalid ranging applies maximum reverse braking; this policy is limited to the forward-approach scene and is not a safety guarantee for arbitrary motion. Closing speed starts at zero and then updates as `0.8 × previous estimate + 0.2 × range difference / timestep`.

## 轻量学习控制 / Lightweight learning controller

训练使用回合数据中的距离和闭合速度两个有效特征，隐藏层为 16 个 ReLU 单元，权重以 JSON 保存，不依赖 PyTorch 或 GPU。/ Training uses range and closing speed from episode data, with 16 ReLU units; JSON weights keep the controller independent of PyTorch and GPUs.

```bash
python train.py --data datasets --output models
python main.py --headless --controller mlp --model models/mlp_controller.json --output results/mlp
```

当前模型测试集 MAE 为 0.1454 m/s²，测试集 RMSE 为 0.2604 m/s²；在默认 MuJoCo 场景运行 1000 步无碰撞，最小间距为 0.459629 m。/ The current model reaches 0.1454 m/s² test MAE and 0.2604 m/s² test RMSE. In the default MuJoCo scene it runs 1000 steps without contact, with a 0.459629 m minimum clearance. These numbers describe this fixed-heading scene and are not a flight-safety guarantee.

## ROS 2 接口 / ROS 2 interface

`launch/ros_node.launch.py` 启动 `uav_range_braking` 节点。节点以 100 Hz 推进仿真并发布 `/uav/front_range_m`、`/uav/braking_accel_m_s2` 和 `/uav/state`；状态数组依次为时间、测距、闭合速度和高度。/ `launch/ros_node.launch.py` starts `uav_range_braking`. The node advances the simulation at 100 Hz and publishes `/uav/front_range_m`, `/uav/braking_accel_m_s2`, and `/uav/state`; the state array is time, range, closing speed, and height.

```bash
source /opt/ros/humble/setup.bash
ros2 launch ./launch/ros_node.launch.py controller:=mlp
ros2 topic echo /uav/state
```

数据默认 30 回合、每回合 12 s，训练/验证/测试为 18/6/6 回合，21600/7200/7200 行。特征为距离、闭合速度、高度误差，标签为加速度；各回合参数与完整 CSV 的 SHA-256 写入 `manifest.json`。Windows 与 Ubuntu 的相同配置产出一致校验值。完整数据保留在 `datasets/`，仓库的 `sample_data/` 提供字段样本及汇总。

The default dataset has 30 episodes of 12 s, split into 18/6/6 episodes and 21600/7200/7200 rows for training/validation/test. Features are range, closing speed and altitude error; acceleration is the target. `manifest.json` records episode parameters and full-file SHA-256 values. Identical configurations produce matching hashes on Windows and Ubuntu. Full data reside in `datasets/`; `sample_data/` contains schema samples and a summary.

训练标签来自解析测距控制基线。由于回合初始高度等于参考值，数据中的高度误差恒为零；当前 MLP 因此只使用测距和闭合速度两个特征，高度仍由独立 PD 回路保持。 / Training labels come from the analytical ranging controller. Since each episode starts at the reference altitude, height error is always zero in the dataset. The MLP therefore uses range and closing speed as its two features, while a separate PD loop holds altitude.

## 完整报告 / Complete report

[课程报告](../../../docs/air/mujoco_uav_course_report.md)包含操作步骤、公式、算法流程、源码解析、三种策略在三档初速度下的指标以及 ROS 2 观测结果。 / The [course report](../../../docs/air/mujoco_uav_course_report.md) contains procedures, equations, algorithm flow, source analysis, three-controller comparisons at three initial speeds, and ROS 2 observations.

神经控制在默认 1.5 m/s 场景无碰撞；3 m/s 超出训练速度范围，记录到接触。 / Neural control has no contact in the default 1.5 m/s scene; contact is recorded at 3 m/s, outside training-speed coverage.
