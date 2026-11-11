# MuJoCo 肌肉骨骼肘关节控制 / MuJoCo Musculoskeletal Elbow Control

本 ROS 2 Humble 功能包建立单自由度肘关节与屈肌、伸肌两个拮抗执行器，发布关节状态、肌腱力和肌肉激活量，并使用 PD 专家控制器采集神经网络训练所需的完整 episode。

This ROS 2 Humble package models a one-degree-of-freedom elbow with antagonistic flexor and extensor actuators. It publishes joint state, tendon forces, and muscle activations, and collects complete expert episodes for later neural-network training.

## 环境 / Environment

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MuJoCo 3.12.0
- PyTorch CPU（后续训练阶段 / later training stage）

## 构建与启动 / Build and launch

```bash
python3 -m pip install -r src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/requirements.txt
colcon build --packages-select mujoco_musculoskeletal_elbow_nn_control --symlink-install
source install/setup.bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control main.launch.py
```

仿真节点订阅 `/target_angle`，并发布 `/joint_states`、`/tendon_forces` 与 `/muscle_activations`。目标角度的安全范围为 0.05–2.20 rad，屈肌和伸肌激活量始终限制在 `[0, 1]`。

The simulation node subscribes to `/target_angle` and publishes `/joint_states`, `/tendon_forces`, and `/muscle_activations`. Target angles are accepted within 0.05–2.20 rad, and both muscle activations are always clamped to `[0, 1]`.

```bash
ros2 topic pub --once /target_angle std_msgs/msg/Float64 "{data: 1.2}"
ros2 topic echo /joint_states
ros2 topic echo /tendon_forces
```

## 专家数据采集 / Expert data collection

默认配置覆盖阶跃、正弦和 0.0/0.5 kg 前臂负载。CSV 每行包含目标角度、实际角度、角速度、两路肌腱力、两路专家激活量和负载；相邻帧不会跨 episode 划分。

The default configuration covers step and sine targets under 0.0 and 0.5 kg forearm loads. Each CSV row records target angle, measured angle, angular velocity, two tendon forces, two expert activations, and load. Dataset partitions operate on complete episodes rather than adjacent frames.

```bash
collect_episodes \
  --model install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/models/elbow.xml \
  --config install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/config/episodes.yaml
```

采集数据默认写入 `data/episodes`，该目录由仓库忽略。每个 CSV 配套一个 JSON 元数据文件，记录 schema 版本、随机种子、轨迹参数、负载和样本数。

Collected data defaults to `data/episodes`, which is ignored by Git. Every CSV has a JSON metadata file containing the schema version, random seed, trajectory parameters, load, and sample count.

## 测试 / Tests

```bash
python3 -m pytest src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/test -q
colcon test --packages-select mujoco_musculoskeletal_elbow_nn_control
colcon test-result --all
```

PD 控制器是专家数据基线，不是最终神经网络控制器。后续 MLP 使用 `[目标角度, 实际角度, 角速度, 屈肌力, 伸肌力]` 作为输入，输出屈肌和伸肌激活量，并与 PD 基线使用相同轨迹、负载和评价指标进行比较。

The PD controller is the expert-data baseline, not the final neural controller. The later MLP consumes `[target angle, measured angle, angular velocity, flexor force, extensor force]`, outputs flexor and extensor activations, and is evaluated against the PD baseline under identical trajectories, loads, and metrics.
