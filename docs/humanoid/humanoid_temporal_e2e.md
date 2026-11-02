# 人形机器人多帧时序端到端视觉平衡控制

## 1. 功能概述

本模块使用连续3帧前视图像作为输入，通过纯NumPy实现的MLP网络（768→256→128→15）直接回归15维关节目标角。对应课程任务④：**端到端模型（输入图像、输出控制指令）**。

与单帧端到端（PR2）相比，本模块引入时序窗口，能捕捉机器人运动的动态信息。

## 2. 系统架构

```mermaid
graph LR
    A[3帧前视图像] --> B[每帧16×16池化→256维]
    B --> C[拼接为768维]
    C --> D[MLP 768→256→128→15]
    D --> E[15维关节目标角]
    E --> F[站姿PD执行]
```

## 3. 计算原理

网络前向：
- h1 = ReLU(W1·x + b1), W1 ∈ R^{768×256}
- h2 = ReLU(W2·h1 + b2), W2 ∈ R^{256×128}
- y = W3·h2 + b3 ∈ R^{15}

损失函数：L = (1/N) Σ ||y_i - ŷ_i||²（MSE）

## 4. 复现步骤

### 4.1 环境安装

```bash
pip install -r requirements.txt
```

### 4.2 采集数据

```bash
cd src/humanoid/humanoid_temporal_e2e
python3 main.py --collect --episodes 30
```

### 4.3 训练

```bash
python3 main.py --train
```

### 4.4 演示

```bash
python3 main.py --demo
```

### 4.5 ROS2 launch

```bash
colcon build --packages-select humanoid_temporal_e2e
source install/setup.bash
ros2 launch humanoid_temporal_e2e main.launch.py
```

## 5. 与PR2对比

| 维度 | PR2（单帧） | 本模块（3帧时序） |
|---|---|---|
| 输入维度 | 256 | 768（3帧拼接） |
| 时序信息 | 无 | 有 |
| 网络结构 | 256→128→64→15 | 768→256→128→15 |
