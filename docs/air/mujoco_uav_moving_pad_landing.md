# MuJoCo 无人机移动平台着陆 / Moving-platform landing

该示例使用 MuJoCo 相机识别移动平台与无人机位置，通过多层感知机（MLP）根据图像误差、高度和平台速度生成着陆速度指令。模型训练与测试数据均来自仿真 rollout，并按完整 episode 划分，避免同一段轨迹同时进入训练集和测试集。

English: A MuJoCo camera observes the UAV and moving pad. An MLP maps image and motion features to bounded landing commands. Training and test sets are split by episode.

## 感知输入与网络 / Inputs and network

每个仿真步长为 20 ms。特征向量为：

```text
x = [e_x, e_y, h_n, v_pad_x, v_pad_y, q]
```

其中 `e_x, e_y` 是图像中的平台与无人机中心偏差，`h_n` 是归一化高度，`v_pad_x, v_pad_y` 是平台平面速度，`q` 表示目标检测是否有效。特征先经 `StandardScaler` 标准化，再输入含 24、16 个神经元的两个 `tanh` 隐藏层 MLP：

```text
z   = StandardScaler(x)
h1  = tanh(W1 z + b1)
h2  = tanh(W2 h1 + b2)
u_hat = W3 h2 + b3
```

模型以仿真 rollout 中的参考控制器指令作为标签。测试集按 episode 独立划分，并计算三轴速度预测的均方根误差：

```text
RMSE = sqrt( sum((û[i,j] - u[i,j])^2) / (N × 3) )
```

训练命令会输出 `test_rmse_m_s`、`iid_test_rmse_m_s` 和训练/测试 episode 数。无效检测或非有限输入会将指令置零；有效输出限幅为 `vx, vy ∈ [-0.55, 0.55] m/s`、`vz ∈ [-0.30, 0] m/s`。

English: The six features are normalized, processed by a 24/16-unit `tanh` MLP, and mapped to three velocity components. Episode-level test RMSE is reported with the training summary.

## 算法流程 / Algorithm flow

1. MuJoCo 渲染场景图像，并读取无人机高度与平台速度。 / Render the scene and read altitude and pad velocity.
2. 检测平台及无人机中心，组装六维特征。 / Detect both image centers and assemble six features.
3. MLP 推理得到三轴速度，检查检测标记并执行限幅。 / Run MLP inference, validate detection, and clamp the command.
4. ROS 2 发布 `/cmd_vel`；特征源执行仿真步进并发布下一帧 `/landing/features`。 / Publish `/cmd_vel`; the feature source advances MuJoCo and publishes the next `/landing/features` sample.

## 运行与测试 / Run and test

在仓库根目录运行训练、仿真和测试： / Run training, simulation, and tests from the repository root:

```bash
cd src/air/mujoco_uav_moving_pad_landing
python -m pip install -r requirements.txt
python main.py train --samples 5000 --steps-per-episode 20
python main.py demo --steps 180 --output output
python -m pytest -q tests
```

ROS 2 Humble 可通过 Docker Compose 启动特征源和控制器： / Start the feature source and controller with ROS 2 Humble:

```bash
cd src/air/mujoco_uav_moving_pad_landing
docker compose up --build
```

本地 ROS 2 工作区也可构建并通过 launch 启动： / A local ROS 2 workspace can build and launch the package:

```bash
colcon build --base-paths src/air/mujoco_uav_moving_pad_landing --packages-select mujoco_uav_moving_pad_landing
source install/setup.bash
ros2 launch mujoco_uav_moving_pad_landing landing.launch.py
ros2 topic echo /landing/features
ros2 topic echo /cmd_vel
```

## 源码结构 / Source map

| 文件 / File | 职责 / Responsibility |
| --- | --- |
| `landing.py` | MuJoCo 场景、视觉特征、MLP 训练与推理 / Simulation, visual features, MLP training and inference |
| `dataset.py` | rollout 样本生成与 episode 级数据划分 / Rollout generation and episode-level split |
| `feature_source.py` | 以 50 Hz 发布 `/landing/features` 并应用 `/cmd_vel` / Publish features at 50 Hz and apply velocity commands |
| `ros_node.py` | 订阅特征、运行 MLP 并发布限幅后的 `Twist` / Run MLP inference and publish bounded `Twist` commands |
| `launch/landing.launch.py` | 同时启动特征源和着陆控制器 / Launch the feature source and controller together |
| `main.py` | 提供训练、演示和指标输出入口 / Provide training, demo, and metric commands |

## 仿真结果 / Simulation result

默认场景的首次接触记录如下： / First-contact metrics from the default scene:

| 指标 / Metric | 结果 / Result |
| --- | ---: |
| 首次接触 / First contact | 第 149 步，2.98 s / Step 149, 2.98 s |
| 平面偏差 / Planar offset | 0.02186 m |
| 垂直速度 / Vertical speed | 0.02548 m/s |
| 接触点 / Contact points | 4 |
| 着陆成功 / Landing success | `true` |

成功判定在首次接触帧进行：无人机接触区域位于平台有效范围内、接触点数 `ncon > 0`、无人机高度与模型接触高度之差不超过 `0.03 m`，且垂直速度绝对值不超过 `0.6 m/s`。高度与速度条件分别为 `|z_uav - z_contact| <= 0.03 m`、`|v_z| <= 0.6 m/s`。

English: Landing success is evaluated at first contact. The UAV footprint must remain inside the pad, `ncon > 0`, altitude error must be at most `0.03 m`, and absolute vertical speed must be at most `0.6 m/s`.

180 步仿真时长为 3.6 s；着陆过程 GIF 为 91 帧、播放时长 6.0 s。 / The 180-step simulation spans 3.6 s; the landing GIF has 91 frames and a 6.0 s playback duration.

![MuJoCo 移动平台着陆场景 / MuJoCo moving-platform landing scene](../img/air/mujoco_uav_moving_pad_landing.png)

![MuJoCo 无人机着陆过程 / UAV landing sequence](../img/air/mujoco_uav_moving_pad_landing.gif)
