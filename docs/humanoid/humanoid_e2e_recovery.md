# 人形机器人端到端视觉平衡控制（行为克隆）

## 1. 功能概述

本模块采用行为克隆（Behavioral Cloning）方法，训练一个 3 层 MLP 神经网络直接从前视相机图像回归 15 维关节目标角，实现"图像→控制指令"的端到端映射。对应课程任务④：**端到端模型（输入图像、输出控制指令）**。

## 2. 系统架构

```mermaid
graph LR
    A[前视相机 64×64] --> B[16×16 池化→256维]
    B --> C[MLP 256→128→64→15]
    C --> D[15维关节目标角]
    D --> E[站姿PD执行]
```

## 3. 计算原理

网络结构：

$$
\mathbf{h}_1 = \mathrm{ReLU}(\mathbf{W}_1 \mathbf{x} + \mathbf{b}_1)
$$

$$
\mathbf{h}_2 = \mathrm{ReLU}(\mathbf{W}_2 \mathbf{h}_1 + \mathbf{b}_2)
$$

$$
\mathbf{y} = \mathbf{W}_3 \mathbf{h}_2 + \mathbf{b}_3 \in \mathbb{R}^{15}
$$

损失函数为均方误差：

$$
\mathcal{L} = \frac{1}{N} \sum_{i=1}^{N} \|\mathbf{y}_i - \hat{\mathbf{y}}_i\|^2
$$

专家控制器：基于躯干倾角的比例控制

$$
\Delta\mathbf{q} = K_p \cdot \boldsymbol{\theta}_{\text{torso}}
$$

## 4. 性能指标

| 指标 | 数值 |
|---|---|
| 验证 MSE | 2.8e-05 rad² |
| 关节角误差 | ~0.3° |
| 推理耗时 | < 1 ms/帧 |

## 5. 复现步骤

### 5.1 环境安装

```bash
pip install -r requirements.txt
# 依赖：mujoco >= 3.0, numpy >= 1.21
```

### 5.2 自检

```bash
cd src/humanoid/humanoid_e2e_recovery
python3 main.py --check
```

### 5.3 采集专家数据

```bash
python3 main.py --collect --episodes 30
# 输出：data/e2e_dataset.npz（约 8943 样本）
```

### 5.4 训练网络

```bash
python3 main.py --train
# 输出：models_trained/e2e_policy.npz
# 典型验证 MSE：2.8e-05 rad²
```

### 5.5 演示

```bash
python3 main.py --demo --force 100 --direction 0
# 前向推扰 100 N，端到端网络直接输出关节目标角
```

### 5.6 ROS2 launch（Ubuntu 20.04 Humble）

```bash
colcon build --packages-select humanoid_e2e_recovery
source install/setup.bash
ros2 launch humanoid_e2e_recovery main.launch.py
```

## 演示动图

![端到端视觉平衡控制](../img/humanoid/e2e_recovery_demo.gif)

