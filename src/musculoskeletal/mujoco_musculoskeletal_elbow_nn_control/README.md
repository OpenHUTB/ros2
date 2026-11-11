# MuJoCo 肌肉骨骼肘关节控制

本 ROS 2 Humble 功能包构建单自由度肘关节模型，包含屈肌和伸肌两个拮抗执行器。模块发布关节状态、肌腱力和肌肉激活量，并提供 PD 专家控制器，用于采集神经网络训练所需的完整数据回合。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MuJoCo 3.12.0
- PyTorch CPU（神经网络训练阶段）

## 构建与启动

~~~bash
python3 -m pip install -r src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/requirements.txt
colcon build --packages-select mujoco_musculoskeletal_elbow_nn_control --symlink-install
source install/setup.bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control main.launch.py
~~~

仿真节点订阅 /target_angle，并发布 /joint_states、/tendon_forces 和 /muscle_activations。目标角度安全范围为 0.05–2.20 rad，屈肌和伸肌激活量限制在 [0,1]。

~~~bash
ros2 topic pub --once /target_angle std_msgs/msg/Float64 "{data: 1.2}"
ros2 topic echo /joint_states
ros2 topic echo /tendon_forces
~~~

## 专家数据采集

默认配置覆盖阶跃、正弦轨迹及 0.0 kg 和 0.5 kg 前臂负载。每行 CSV 记录目标角度、实际角度、角速度、两路肌腱力、两路专家激活量和负载。数据集按完整数据回合划分，不会拆分相邻帧。

~~~bash
collect_episodes \
  --model install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/models/elbow.xml \
  --config install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/config/episodes.yaml
~~~

采集数据默认写入 Git 忽略目录 data/episodes。每个 CSV 配套 JSON 元数据文件，记录数据结构版本、随机种子、轨迹参数、负载和样本数。

## 测试

~~~bash
python3 -m pytest src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/test -q
colcon test --packages-select mujoco_musculoskeletal_elbow_nn_control
colcon test-result --all
~~~

PD 控制器用于生成专家数据。后续 MLP 以目标角度、实际角度、角速度、屈肌力和伸肌力为输入，输出两路肌肉激活量，并在相同轨迹、负载和评价指标下与 PD 基线比较。
