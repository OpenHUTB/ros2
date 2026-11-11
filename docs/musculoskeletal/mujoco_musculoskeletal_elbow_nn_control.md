# 基于关节状态与肌腱力反馈的 MuJoCo 肌肉骨骼肘关节神经网络轨迹控制

本模块面向单自由度肘关节轨迹跟踪。MuJoCo 模型包含上臂、前臂、可变负载以及屈肌和伸肌两条拮抗肌腱。PD 控制器生成专家数据回合，轻量多层感知机再以关节状态和肌腱力为反馈预测两路安全激活量。

## 系统接口

| ROS 2 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| /target_angle | std_msgs/Float64 | 输入 | 目标肘角 |
| /joint_states | sensor_msgs/JointState | 输出 | 角度与角速度 |
| /tendon_forces | std_msgs/Float64MultiArray | 输出 | 屈肌力与伸肌力 |
| /muscle_activations | std_msgs/Float64MultiArray | 输出 | 两路 [0,1] 范围内的激活量 |

## 专家控制

角度误差为 \(e=q_d-q\)，PD 期望关节力矩为：

$$
\tau_d=K_p e-K_d\dot q.
$$

令 \(\tau_s\) 为归一化力矩尺度，\(a_c\) 为共同收缩量，则拮抗激活为：

$$
a_f=\operatorname{clip}(a_c+\max(0,\tau_d/\tau_s),0,1),
\quad
a_e=\operatorname{clip}(a_c+\max(0,-\tau_d/\tau_s),0,1).
$$

力矩符号决定主要发力的肌腱；适量共同收缩使两条肌腱保持张紧。激活命令传入 MuJoCo 前经过有限值检查，并限制在 [0,1] 范围内。

## 数据格式与划分

每个 CSV 文件只包含一个完整数据回合。字段包括 episode_id、step、time_s、target_angle_rad、angle_rad、angular_velocity_rad_s、flexor_force_n、extensor_force_n、flexor_activation、extensor_activation 和 load_kg。同名 JSON 文件记录数据结构版本、固定随机种子、轨迹配置、负载、字段列表和样本数。训练集、验证集和测试集按完整数据回合编号划分，防止相邻帧泄漏。

## 运行

~~~bash
python3 -m pip install -r src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/requirements.txt
colcon build --packages-select mujoco_musculoskeletal_elbow_nn_control --symlink-install
source install/setup.bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control main.launch.py
~~~

## 数据采集

采集阶跃、正弦轨迹和不同负载下的数据回合：

~~~bash
collect_episodes \
  --model install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/models/elbow.xml \
  --config install/mujoco_musculoskeletal_elbow_nn_control/share/mujoco_musculoskeletal_elbow_nn_control/config/episodes.yaml
~~~

## 神经网络与评价

轻量多层感知机的输入为 \([q_d,q,\dot q,F_f,F_e]\)，输出为 \([a_f,a_e]\)。输入标准化参数只由训练集计算；网络使用 tanh 隐藏层和 sigmoid 输出层，损失函数为：

$$
L=\frac{1}{N}\sum_i\|\hat a_i-a_i^{PD}\|_2^2.
$$

在线推理对输出执行有限值检查和 [0,1] 限幅，并在输入缺失或超时时进入零激活安全状态。评价按完整数据回合、轨迹类型和负载分组，报告平均绝对误差、均方根误差、末段稳态误差和单样本 CPU 推理耗时。PD 与神经网络控制器使用相同的目标轨迹、初始状态和负载。

## ROS 2 神经控制器

~~~bash
ros2 launch mujoco_musculoskeletal_elbow_nn_control neural_controller.launch.py
~~~

节点订阅 /target_angle、/joint_states 和 /tendon_forces，发布 /muscle_activations。验证时检查输入超时、输出有限性、激活范围及节点退出后的零激活状态。

## 验收

~~~bash
python3 -m pytest src/musculoskeletal/mujoco_musculoskeletal_elbow_nn_control/test -q
colcon test --packages-select mujoco_musculoskeletal_elbow_nn_control
colcon test-result --all
mkdocs build --strict
~~~
