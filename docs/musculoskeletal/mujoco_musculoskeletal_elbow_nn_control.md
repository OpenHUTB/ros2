# 基于关节状态与肌腱力反馈的 MuJoCo 肌肉骨骼肘关节神经网络轨迹控制

## Neural Trajectory Control of a MuJoCo Musculoskeletal Elbow Using Joint-State and Tendon-Force Feedback

本模块面向单自由度肘关节轨迹跟踪。MuJoCo 模型包含上臂、前臂、可变负载以及屈肌和伸肌两条拮抗肌腱。第一阶段使用 PD 控制器生成可复现的专家 episode；后续轻量多层感知机以关节状态和肌腱力为反馈，直接预测两路安全激活量。

This module targets trajectory tracking for a one-degree-of-freedom elbow. The MuJoCo model contains an upper arm, a forearm, a variable payload, and antagonistic flexor and extensor tendons. A PD controller first generates reproducible expert episodes. A later lightweight multilayer perceptron uses joint-state and tendon-force feedback to predict two safe muscle activations.

## 系统接口 / System interface

| ROS 2 话题 / Topic | 类型 / Type | 方向 / Direction | 含义 / Meaning |
| --- | --- | --- | --- |
| `/target_angle` | `std_msgs/Float64` | 输入 / Input | 目标肘角 / target elbow angle |
| `/joint_states` | `sensor_msgs/JointState` | 输出 / Output | 角度与角速度 / angle and angular velocity |
| `/tendon_forces` | `std_msgs/Float64MultiArray` | 输出 / Output | 屈肌力、伸肌力 / flexor and extensor force |
| `/muscle_activations` | `std_msgs/Float64MultiArray` | 输出 / Output | 两路 `[0,1]` 激活量 / two bounded activations |

## 专家控制 / Expert control

角度误差为 \(e=q_d-q\)，PD 期望关节力矩为：

\[
\tau_d=K_p e-K_d\dot q.
\]

令 \(\tau_s\) 为归一化力矩尺度，\(a_c\) 为共同收缩量，则拮抗激活为：

\[
a_f=\operatorname{clip}(a_c+\max(0,\tau_d/\tau_s),0,1),
\quad
a_e=\operatorname{clip}(a_c+\max(0,-\tau_d/\tau_s),0,1).
\]

The angle error is \(e=q_d-q\), and the PD target torque is \(\tau_d=K_p e-K_d\dot q\). The torque sign selects the agonist while a small co-contraction keeps both tendons engaged. Both commands pass through a finite-value check and a `[0,1]` clamp before reaching MuJoCo.

## 数据 schema / Data schema

每个 CSV 文件只包含一个完整 episode。字段为 `episode_id`、`step`、`time_s`、`target_angle_rad`、`angle_rad`、`angular_velocity_rad_s`、`flexor_force_n`、`extensor_force_n`、`flexor_activation`、`extensor_activation` 和 `load_kg`。同名 JSON 文件记录 schema 版本、固定随机种子和轨迹参数。

Each CSV file contains exactly one complete episode. The paired JSON file records the schema version, fixed random seed, trajectory configuration, payload, column list, and sample count. Train, validation, and test partitions are formed by episode ID to prevent adjacent-frame leakage.

## 运行 / Run

```bash
python3 -m pip install -r src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/requirements.txt
colcon build --packages-select mujoco_musculoskeletal_elbow_nn_control --symlink-install
source install/setup.bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control main.launch.py
```

采集阶跃、正弦及不同负载 episode：

Collect step, sine, and variable-load episodes:

```bash
collect_episodes \
  --model install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/models/elbow.xml \
  --config install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/config/episodes.yaml
```

## 神经网络阶段 / Neural-network stage

轻量 MLP 输入为 \([q_d,q,\dot q,F_f,F_e]\)，输出为 \([a_f,a_e]\)。训练、验证和测试按完整 episode 划分并固定随机种子。在线推理继续复用激活限制器与异常安全停机逻辑。评价覆盖角度均方根误差、超调量、稳定时间和 CPU 推理延迟，并分别报告轨迹类型与前臂负载。

The lightweight MLP input is \([q_d,q,\dot q,F_f,F_e]\), and its output is \([a_f,a_e]\). Training, validation, and testing use complete-episode splits with a fixed seed. Online inference reuses the activation limiter and safe-stop path. Evaluation covers angular RMSE, overshoot, settling time, and CPU inference latency, grouped by trajectory type and forearm load.

## 验收 / Acceptance

```bash
python3 -m pytest src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/test -q
colcon test --packages-select mujoco_musculoskeletal_elbow_nn_control
colcon test-result --all
mkdocs build --strict
```

ROS 2 验收还应检查 `/joint_states`、`/tendon_forces` 和 `/muscle_activations` 的发布频率与数值范围，并在节点关闭时确认激活量归零。

ROS 2 acceptance also checks the publication rate and numeric ranges of `/joint_states`, `/tendon_forces`, and `/muscle_activations`, and confirms zero activation during node shutdown.
