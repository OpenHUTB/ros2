# 无信号环岛汇入仿真 / Unsignalized Roundabout Entry

本教程介绍固定随机种子的二维环岛车辆仿真，以及雷达间隙特征、神经网络汇入策略、占据栅格寻路和图像到控制模型。当前系统采用 Python 单轨运动学模型；不包含 CARLA 场景、真实传感器接入或 ROS 话题闭环。

This tutorial covers a deterministic 2D roundabout vehicle simulation, radar gap features, a neural entry policy, occupancy-grid path planning, and an image-to-control model. The current system uses a Python single-track kinematic model. It does not include a CARLA scene, real sensor integration, or a closed-loop ROS topic interface.

![固定种子环岛仿真回放 / Deterministic roundabout simulation](../img/roundabout_demo.gif)

## 1. 运动学仿真 / Kinematic Simulation

每个仿真步长为 \(\Delta t=0.05\,\mathrm{s}\)，车辆轴距 \(L=2.7\,\mathrm{m}\)。油门 \(u\in[0,1]\)、制动 \(b\in[0,1]\) 和转向角 \(\delta\in[-0.55,0.55]\) 更新速度、航向和位置：

\[
v_{k+1}=\operatorname{clip}\left(v_k+(3u_k-7b_k)\Delta t,\;0,\;12\right)
\]

\[
\psi_{k+1}=\psi_k+\frac{v_{k+1}}{L}\tan(\delta_k)\Delta t,\qquad
x_{k+1}=x_k+v_{k+1}\cos(\psi_{k+1})\Delta t,\qquad
y_{k+1}=y_k+v_{k+1}\sin(\psi_{k+1})\Delta t.
\]

The simulator advances at \(0.05\,\mathrm{s}\) per step with a wheelbase of \(2.7\,\mathrm{m}\). Throttle, brake, and steering are clipped to bounded ranges before updating speed, heading, and position. Circulating traffic is initialized from a fixed random seed.

## 2. 雷达间隙特征 / Radar Gap Features

前向半圆由 25 条射线组成，最大量程为 30 m；每条射线返回与圆形车辆相交的最近距离。间隙特征包括前向距离 \(d\)、横向偏移 \(\ell\)、闭合速度 \(v_c\) 和碰撞时间估计 \(TTC=d/v_c\)（仅在闭合速度为正时使用该估计）。

The forward 180-degree field uses 25 rays with a maximum range of 30 m. Each ray reports the nearest vehicle-circle intersection. The feature vector contains forward gap, lateral offset, closing speed, and a time-to-collision estimate when the closing speed is positive.

## 3. 神经网络汇入策略 / Neural Entry Policy

间隙策略将四维特征输入含 12 个隐藏单元的 MLP，再以 Softmax 输出 `WAIT` 或 `ENTER`。训练示例由规则生成的合成样本构成，不能替代实车或真实交通数据训练。决策阈值默认为 0.65。

The gap policy maps four features through a 12-unit hidden layer and a Softmax output to `WAIT` or `ENTER`. Its training example uses rule-generated synthetic samples; it is not trained on real traffic or vehicle data. The default entry threshold is 0.65.

## 4. 占据栅格与 A* / Occupancy Grid and A*

Bresenham 射线将观测路径标记为空闲、终点标记为占据；A* 使用八邻接搜索，只允许穿过已观测空闲单元，未知区域和障碍物均不可通行，并阻止对角线穿角。

Bresenham ray updates mark traversed cells free and endpoints occupied. Eight-connected A* searches only observed-free cells, treats unknown cells as blocked, and prevents diagonal corner cutting.

## 5. 图像到控制 / Image to Control

端到端模型接受 16×16 灰度数组，经 24 个隐藏单元映射为转向、油门和制动三个数值。当前提供模型类和训练接口；相机图像采集、数据集生成和闭环驾驶尚未接入。

The image-to-control model accepts a 16×16 grayscale array and predicts steering, throttle, and brake through a 24-unit hidden layer. The current implementation provides the model and training interface; camera capture, dataset generation, and closed-loop driving are not integrated.

## 6. 运行步骤 / Run the Simulation

在仓库根目录运行固定场景：

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360
```

启动键盘窗口：

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --gui --seed 7
```

使用多个随机种子汇总性能：

```bash
python3 src/ground/carla_nn_roundabout_entry/evaluate_scenarios.py --seeds 3 7 11 19 23 --steps 360
```

From the repository root, run the deterministic simulation with `--seed 7 --steps 360`. Add `--gui` to open the keyboard-controlled window. The evaluation command runs the same workload over five seeds and prints per-scenario and aggregate metrics.

ROS 2 Humble package launch entry:

```bash
colcon build --packages-select carla_nn_roundabout_entry
source install/setup.bash
export AMENT_PREFIX_PATH="$PWD/install:$AMENT_PREFIX_PATH"
ros2 launch carla_nn_roundabout_entry main.launch.py
```

## 7. 五种子结果 / Five-Seed Results

| 随机种子 / Seed | 碰撞帧 / Collision Frames | 最小离障距离 / Minimum Clearance (m) |
| ---: | ---: | ---: |
| 3 | 22 | -1.728 |
| 7 | 17 | -2.072 |
| 11 | 23 | -1.583 |
| 19 | 16 | -1.708 |
| 23 | 16 | -1.340 |

The five-seed run recorded collisions in all five scenarios, with 18.8 collision frames on average and a worst minimum clearance of -2.072 m. These measurements describe the current baseline and do not indicate safe entry.

文档首页：<https://openhutb.github.io/ros2/>。源码与运行说明见 [模块 README](../../src/ground/carla_nn_roundabout_entry/README.md)。
