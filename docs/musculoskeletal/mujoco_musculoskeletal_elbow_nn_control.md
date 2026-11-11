# 肌肉骨骼肘关节轨迹控制与激活模型训练

本模块使用 MuJoCo 模拟单自由度肘关节、可变负载及屈肌和伸肌两条拮抗肌腱。PD 控制器提供专家控制基线和训练数据；轻量多层感知机根据目标角度、关节状态和肌腱力预测两路肌肉激活量。

## 系统接口

| ROS 2 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| /target_angle | std_msgs/Float64 | 输入 | 目标肘角 |
| /joint_states | sensor_msgs/JointState | 输出 | 关节角度和角速度 |
| /tendon_forces | std_msgs/Float64MultiArray | 输出 | 屈肌力和伸肌力 |
| /muscle_activations | std_msgs/Float64MultiArray | 输出 | 两路 [0,1] 范围内的激活量 |

## PD 专家控制

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

激活命令传入 MuJoCo 前经过有限值检查，并限制在 [0,1] 范围内。

## 数据格式与划分

每个 CSV 文件只包含一个完整数据回合，记录目标角度、实际角度、角速度、两路肌腱力、两路激活量和负载。字段包括 episode_id、step、time_s、target_angle_rad、angle_rad、angular_velocity_rad_s、flexor_force_n、extensor_force_n、flexor_activation、extensor_activation 和 load_kg。同名 JSON 文件记录数据结构版本、固定随机种子、轨迹参数、字段列表和样本数。训练集、验证集和测试集按完整数据回合编号划分，避免相邻帧泄漏。

## 构建与启动

~~~bash
python3 -m pip install -r src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/requirements.txt
colcon build --packages-select mujoco_musculoskeletal_elbow_nn_control --symlink-install
source install/setup.bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control main.launch.py
~~~

发布目标角度并检查状态与肌腱力：

~~~bash
ros2 topic pub --once /target_angle std_msgs/msg/Float64 "{data: 1.2}"
ros2 topic echo /joint_states
ros2 topic echo /tendon_forces
~~~

## 专家数据采集

默认配置覆盖阶跃、正弦轨迹和不同负载下的数据回合：

~~~bash
collect_episodes \
  --model install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/models/elbow.xml \
  --config install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/config/episodes.yaml
~~~

每个数据回合单独保存 CSV 文件，并配套记录结构版本、随机种子、轨迹参数、负载和样本数的 JSON 元数据。

## 多层感知机训练与评价

模型输入为 \([q_d,q,\dot q,F_f,F_e]\)，输出为屈肌和伸肌激活量 \([a_f,a_e]\)。训练器仅使用训练集统计量进行输入标准化。训练、验证和测试按完整数据回合隔离，确保同一回合的相邻帧不会跨集合。

~~~bash
python3 -m mujoco_musculoskeletal_elbow_nn_control.train --data data/episodes --model data/elbow_mlp.json
python3 -m mujoco_musculoskeletal_elbow_nn_control.evaluate --data data/episodes --model data/elbow_mlp.json --output data/evaluation.json
~~~

评价按轨迹类型和负载分组，报告平均绝对误差、均方根误差、稳态误差和单样本 CPU 推理耗时。

## 测试

~~~bash
python3 -m pytest src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/test -q
colcon test --packages-select mujoco_musculoskeletal_elbow_nn_control
colcon test-result --all
~~~
