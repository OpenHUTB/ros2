# MuJoCo 肌肉骨骼肘关节控制

本 ROS 2 功能包提供单自由度肘关节仿真、PD 专家控制、多场景数据回合采集、MLP 训练与评价，以及安全神经控制节点。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MuJoCo 3.12.0
- PyYAML 6.0 或更高版本

## 安装、构建与启动

```bash
python3 -m pip install -r src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/requirements.txt
colcon build --packages-select mujoco_musculoskeletal_elbow_nn_control --symlink-install
source install/setup.bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control main.launch.py
```

仿真节点订阅 `/target_angle`，发布 `/joint_states`、`/tendon_forces` 和 `/muscle_activations`。目标角度限制在 0.05–2.20 rad，两路肌肉激活量限制在 `[0,1]`。

```bash
ros2 topic pub --once /target_angle std_msgs/msg/Float64 "{data: 1.2}"
ros2 topic echo /joint_states
ros2 topic echo /tendon_forces
```

## 专家数据采集

默认配置运行阶跃、正弦轨迹与 0.0 kg、0.5 kg 前臂负载四种组合。每个 CSV 包含目标角度、实际角度、角速度、两路肌腱力、两路专家激活量和负载；同名 JSON 文件记录数据结构版本、随机种子、轨迹参数和样本数。采集器还会写出 `episode_split.json`，固定训练、验证与测试回合的归属；后续训练直接复用该清单。

```bash
collect_episodes \
  --model install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/models/elbow.xml \
  --config install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/config/episodes.yaml
```

数据默认写入 `data/episodes`，该目录由 Git 忽略。自定义轨迹必须完全位于模型关节角范围内；采集过程带有按持续时间计算的最大步数保护。

## MLP 训练与评价

输入为目标角度、实际角度、角速度和两路肌腱力，输出为两路 `[0,1]` 肌肉激活量。数据划分按完整数据回合进行。评价报告平均绝对误差、均方根误差、末段稳态误差和 CPU 单样本推理耗时。

```bash
python3 -m mujoco_musculoskeletal_elbow_nn_control.train \
  --data data/episodes --model data/elbow_mlp.json
python3 -m mujoco_musculoskeletal_elbow_nn_control.evaluate \
  --data data/episodes --model data/elbow_mlp.json \
  --output data/evaluation.json
```

## ROS 2 神经控制器

```bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control neural_controller.launch.py
```

节点订阅 `/target_angle`、`/joint_states` 和 `/tendon_forces`，发布 `/muscle_activations`。输入缺失或超时时发布零激活。

## 测试

```bash
python3 -m pytest src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/test -q
colcon test --packages-select mujoco_musculoskeletal_elbow_nn_control
colcon test-result --all
```
