# 人形机器人深度图姿态估计与比例平衡控制

## 1. 功能概述

本模块使用深度相机（front_camera depth）渲染64×64深度图，训练纯NumPy实现的3层MLP神经网络，从深度图回归躯干的pitch/roll/yaw三个连续姿态角，再通过比例控制器输出15维关节修正目标。对应课程任务②：**深度相机感知与运动控制**。

与RGB灰度图分类（5类跌倒方向）不同，本模块输出连续姿态角，是回归任务。

## 2. 系统架构

```mermaid
graph LR
    A[深度相机 64×64] --> B[16×16池化→256维]
    B --> C[MLP 256→128→64→3]
    C --> D[pitch, roll, yaw]
    D --> E[比例控制器]
    E --> F[15维关节目标角]
    F --> G[站姿PD执行]
```

## 3. 计算原理

网络前向：
- h1 = ReLU(W1·x + b1), W1 ∈ R^{256×128}
- h2 = ReLU(W2·h1 + b2), W2 ∈ R^{128×64}
- y = W3·h2 + b3 ∈ R^3

损失函数：L = (1/N) Σ ||y_i - ŷ_i||²（MSE）

比例控制律：q_target = q_stand + Kp·θ_torso

## 4. 复现步骤

### 4.1 环境安装

```bash
pip install -r requirements.txt
```

### 4.2 自检

```bash
cd src/humanoid/humanoid_depth_pose_control
python3 main.py --check
```

### 4.3 采集数据

```bash
python3 main.py --collect --samples 2000
# 输出：data/depth_pose_dataset.npz
```

### 4.4 训练

```bash
python3 main.py --train
# 输出：models_trained/depth_pose_mlp.npz
```

### 4.5 演示

```bash
python3 main.py --demo --force 120 --direction 0
```

### 4.6 ROS2 launch（Ubuntu 20.04 Humble）

```bash
colcon build --packages-select humanoid_depth_pose_control
source install/setup.bash
ros2 launch humanoid_depth_pose_control main.launch.py
```

## 5. 性能指标

| 指标 | 数值 |
|---|---|
| 输入分辨率 | 64×64 深度图 |
| 输出维度 | 3（pitch/roll/yaw） |
| 网络参数量 | ~41K |
| 推理耗时 | <1ms/帧 |
