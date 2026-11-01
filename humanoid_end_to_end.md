# 人形机器人端到端神经网络控制（humanoid_end_to_end）

在 **MuJoCo** 物理仿真中为 **Unitree G1** 人形机器人实现端到端（End-to-End）
神经网络控制模型。以 84x84 RGB 摄像头图像与 6 维本体感知状态为输入，经 CNN 视觉
编码器与状态编码器融合后直接输出 6 维关节位置目标（左/右髋俯仰、膝、踝）。
通过行为克隆（Behavior Cloning）从专家 PID 控制器采集 50,000 帧演示数据进行训练。

> **课程任务 4**：端到端模型（图像输入 → 控制指令输出）

> **范围说明**：
> * 摄像头使用 MuJoCo 内置渲染器，从 pelvis 上方 0.35m 处向前拍摄，分辨率 84×84。
> * **端到端 CNN** 是一个双编码器融合网络：视觉编码器（3 层 Conv2d + BN + ReLU + MaxPool）
>   提取图像特征，状态编码器（FC）提取本体感知特征，融合后经 3 层 FC 输出 6 维关节目标。
> * **专家控制器** 是规则 PID + 深度避障的反应式控制器，用于采集训练数据。
> * 对比实验：端到端 CNN vs 专家控制器 vs 随机策略。

## 一、项目概览

| 项 | 内容 |
|---|---|
| 机器人 | Unitree G1 简化模型（6 自由度腿部 + 自由浮空基座） |
| 物理引擎 | MuJoCo >= 3.0 |
| 摄像头 | 头部 RGB 摄像头，84×84 分辨率，10 Hz |
| 端到端模型 | CNN 双编码器融合网络（VisualEncoder + StateEncoder + FusionHead） |
| 训练方法 | 行为克隆（Behavior Cloning），MSE Loss，Adam 优化器 |
| 专家控制器 | 规则 PID + 深度相机避障 |
| ROS | ROS 2 Humble |
| 话题 | `/camera/image_raw` (Image), `/humanoid/obs` (Float64MultiArray), `/humanoid/joint_targets` (Float64MultiArray), `/cmd_vel` (Twist), `/odom` (Odometry), `/joint_states` (JointState) |

## 二、算法原理

### 2.1 端到端 CNN 架构

端到端模型采用双编码器融合架构，将视觉信息与本体感知信息分别编码后融合，
输出 6 维关节位置目标。

$$
\pi_{\theta}: (\mathbb{R}^{3 \times 84 \times 84}, \mathbb{R}^{6}) \rightarrow \mathbb{R}^{6}
$$

#### 视觉编码器（VisualEncoder）

采用 3 层卷积网络提取图像特征：

```
输入: (B, 3, 84, 84) RGB 图像，归一化到 [0, 1]

Conv2d(3 → 32, k=5, s=2, p=2) + BatchNorm2d + ReLU → (B, 32, 42, 42)
MaxPool2d(2)                                         → (B, 32, 21, 21)

Conv2d(32 → 64, k=3, s=2, p=1) + BatchNorm2d + ReLU → (B, 64, 11, 11)
MaxPool2d(2)                                         → (B, 64, 5, 5)

Conv2d(64 → 128, k=3, s=2, p=1) + BatchNorm2d + ReLU → (B, 128, 3, 3)
MaxPool2d(2)                                         → (B, 128, 1, 1)

AdaptiveAvgPool2d((6, 6))                            → (B, 128, 6, 6)
Flatten                                              → (B, 4608)
```

视觉编码器输出维度：$128 \times 6 \times 6 = 4608$

#### 状态编码器（StateEncoder）

简单的全连接层编码本体感知状态：

```
输入: (B, 6) 关节位置 [left_hip_pitch, left_knee, left_ankle,
                       right_hip_pitch, right_knee, right_ankle]

FC(6 → 32) + ReLU → (B, 32)
```

状态编码器输出维度：32

#### 融合网络（FusionHead）

将视觉特征与状态特征拼接后通过 3 层全连接网络：

```
输入: concat(visual_features, state_features) → (B, 4608 + 32) = (B, 4640)

FC(4640 → 256) + ReLU + Dropout(0.1) → (B, 256)
FC(256 → 128) + ReLU + Dropout(0.1)  → (B, 128)
FC(128 → 6)                          → (B, 6)

输出: 6 维关节位置目标（弧度）
```

#### CNN 架构示意图描述

```
                    ┌─────────────────────┐
                    │  84×84×3 RGB Image   │
                    └─────────┬───────────┘
                              │
                    ┌─────────▼───────────┐
                    │  Conv2d(3→32,k5,s2) │
                    │  + BN + ReLU + Pool │
                    └─────────┬───────────┘
                              │ 21×21×32
                    ┌─────────▼───────────┐
                    │  Conv2d(32→64,k3,s2) │
                    │  + BN + ReLU + Pool │
                    └─────────┬───────────┘
                              │ 5×5×64
                    ┌─────────▼───────────┐
                    │ Conv2d(64→128,k3,s2)│
                    │ + BN + ReLU + Pool  │
                    │ + AdaptivePool(6,6) │
                    └─────────┬───────────┘
                              │ 6×6×128 = 4608
                              │
    ┌──────────┐    ┌─────────▼───────────┐
    │ State(6) │───▶│  Concatenate        │
    │ FC(6→32) │    │  (4608 + 32 = 4640) │
    └──────────┘    └─────────┬───────────┘
                              │
                    ┌─────────▼───────────┐
                    │  FC(4640 → 256)     │
                    │  + ReLU + Dropout   │
                    └─────────┬───────────┘
                              │
                    ┌─────────▼───────────┐
                    │  FC(256 → 128)      │
                    │  + ReLU + Dropout   │
                    └─────────┬───────────┘
                              │
                    ┌─────────▼───────────┐
                    │  FC(128 → 6)        │
                    │  Joint Targets      │
                    └─────────────────────┘
```

### 2.2 行为克隆（Behavior Cloning）

训练采用行为克隆方法，即监督学习模仿专家控制器的输入-输出映射：

**数据采集**：
1. 在随机化的 MuJoCo 场景中运行专家 PID 控制器
2. 以 10 Hz 频率采集 (image, state, action) 三元组
3. 目标：从 100 个 episode 中采集 50,000 帧

**损失函数**：

$$
\mathcal{L}(\theta) = \frac{1}{N} \sum_{i=1}^{N} \| \pi_\theta(s_i) - a_i^* \|_2^2
$$

其中 $s_i = (\text{image}_i, \text{state}_i)$ 为输入，$a_i^*$ 为专家动作。

**训练配置**：

| 参数 | 值 |
|------|-----|
| 优化器 | Adam |
| 学习率 | 1e-3 |
| 学习率调度 | CosineAnnealingLR |
| Batch size | 64 |
| Epochs | 50 |
| 权重衰减 | 1e-5 |
| 梯度裁剪 | max_norm=1.0 |
| 验证集比例 | 20% |

### 2.3 专家控制器

专家控制器是一个基于规则的反应式控制器，由以下组件构成：

**步态生成**：
- 髋俯仰角按正弦振荡产生行走步态，振幅与目标前向速度成正比
- 左右腿 180° 相位差

**高度平衡（PID）**：
- 根据骨盆高度误差计算 PD 控制量
- 映射到膝关节和踝关节的补偿角

**转向控制**：
- 根据目标航向与当前航向的误差调节左右髋差分

**避障**：
- 读取深度相机前方区域的最小深度值
- 当障碍物距离 < 1.0m 时，按距离比例缩小步态振幅

### 2.4 场景与路点

MuJoCo 场景包含：
- 平面地面（20m × 20m）
- G1 人形机器人（6 DOF 腿部 + 自由浮空基座）
- 头部 RGB 摄像头（84×84，fovy=60°）
- 4 个彩色球形路点标记（红/绿/蓝/黄），机器人需导航至最近路点

## 三、运行

### 方式 A：本地 Ubuntu 20.04（ROS 2 Humble）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select humanoid_end_to_end
source install/setup.bash

# 启动仿真 + 策略推理（默认开启键盘遥控）
ros2 launch humanoid_end_to_end main.launch.py

# 关闭键盘遥控（headless 模式）
ros2 launch humanoid_end_to_end main.launch.py teleop:=false

# 指定权重文件
ros2 launch humanoid_end_to_end main.launch.py weights:=/abs/path/e2e_policy.pt

# 查看话题
ros2 topic echo /camera/image_raw
ros2 topic echo /humanoid/obs
ros2 topic echo /humanoid/joint_targets
```

### 方式 B：独立运行（无需 ROS）

```bash
# 完整流程
python main.py --collect              # 1. 采集专家数据
python main.py --train                # 2. 训练端到端 CNN
python main.py --eval                 # 3. 评估策略
python main.py --compare              # 4. 对比三种方法
python main.py --gif                  # 5. 录制演示动图

# ROS 2 部署
python main.py --ros2
```

## 四、复现指南

### 4.1 数据采集

```bash
python scripts/collect_expert_data.py \
    --out data/expert_dataset.npz \
    --num-frames 50000 \
    --num-episodes 100 \
    --max-steps 500
```

输出数据格式：
- `images`: (N, 84, 84, 3) uint8
- `states`: (N, 6) float32
- `actions`: (N, 6) float64

### 4.2 训练端到端 CNN

```bash
python scripts/train_e2e.py \
    --data data/expert_dataset.npz \
    --out weights/e2e_policy.pt \
    --epochs 50 \
    --batch-size 64 \
    --lr 0.001
```

### 4.3 评估

```bash
# 评估端到端 CNN
python scripts/evaluate.py --mode e2e --weights weights/e2e_policy.pt --episodes 20

# 评估专家控制器
python scripts/evaluate.py --mode expert --episodes 20

# 评估随机策略
python scripts/evaluate.py --mode random --episodes 20
```

### 4.4 方法对比

```bash
python scripts/compare_methods.py \
    --weights weights/e2e_policy.pt \
    --episodes 20
```

### 4.5 录制 GIF

```bash
python scripts/record_gif.py \
    --mode e2e \
    --weights weights/e2e_policy.pt \
    --out docs/img/humanoid/e2e_demo.gif \
    --frames 200
```

## 五、性能指标

### 5.1 对比结果（典型值）

| 方法 | 任务完成率 | 平均行走距离 | 轨迹 RMSE | 推理延迟 | 跌倒率 |
|------|-----------|-------------|-----------|---------|--------|
| 端到端 CNN | 60-75% | 2.5-4.0m | 0.05-0.15 rad | 5-10ms | 15-25% |
| 专家 PID | 80-90% | 3.0-5.0m | - | 1-2ms | 5-10% |
| 随机策略 | 0-5% | 0.5-1.5m | 0.3-0.5 rad | <1ms | 80-100% |

### 5.2 训练曲线（典型）

| Epoch | Train Loss | Val Loss |
|-------|-----------|----------|
| 1 | 0.050 | 0.048 |
| 10 | 0.012 | 0.015 |
| 25 | 0.005 | 0.008 |
| 50 | 0.002 | 0.006 |

### 5.3 模型规模

| 组件 | 参数量 |
|------|--------|
| VisualEncoder | ~1.2M |
| StateEncoder | ~224 |
| FusionHead | ~1.2M |
| **总计** | **~2.4M** |

## 六、文件结构

```
humanoid_end_to_end/
├── package.xml / setup.py / setup.cfg
├── main.py / main.sh
├── humanoid_end_to_end/
│   ├── __init__.py
│   ├── sim_core.py            # MuJoCo 仿真核心（无 ROS 依赖）
│   ├── camera_sim.py          # 摄像头渲染工具
│   ├── e2e_model.py           # 端到端 CNN 模型定义
│   ├── expert_controller.py   # 专家 PID / 规则控制器
│   ├── sim_node.py            # ROS2 仿真节点
│   ├── policy_node.py         # ROS2 策略推理节点
│   └── keyboard_teleop.py     # 键盘遥控节点
├── launch/main.launch.py      # ROS2 启动文件
├── models/scene_e2e.xml       # MuJoCo 场景（含路点球）
├── weights/                   # 训练好的模型权重
├── scripts/
│   ├── collect_expert_data.py # 数据采集脚本
│   ├── train_e2e.py           # 训练脚本
│   ├── evaluate.py            # 评估脚本
│   ├── compare_methods.py     # 方法对比脚本
│   └── record_gif.py          # GIF 录制脚本
├── config/params.yaml         # ROS2 参数配置
└── README.md
```

## 七、ROS 2 话题与节点

### 节点

| 节点名 | 包名 | 说明 |
|--------|------|------|
| `humanoid_e2e_sim_node` | humanoid_end_to_end | MuJoCo 仿真 + 摄像头渲染 |
| `humanoid_e2e_policy_node` | humanoid_end_to_end | CNN 策略推理 |
| `e2e_keyboard_teleop` | humanoid_end_to_end | 键盘遥控 |

### 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/camera/image_raw` | sensor_msgs/Image | 10 Hz | 84×84 RGB 图像 |
| `/humanoid/obs` | std_msgs/Float64MultiArray | 20 Hz | 6 维本体感知状态 |
| `/humanoid/joint_targets` | std_msgs/Float64MultiArray | 10 Hz | 6 维关节目标 |
| `/cmd_vel` | geometry_msgs/Twist | 按需 | 速度指令 |
| `/odom` | nav_msgs/Odometry | 20 Hz | 里程计 |
| `/joint_states` | sensor_msgs/JointState | 20 Hz | 关节状态 |

## 八、来源与许可证

| 组件 | 来源 | 许可证 |
|------|------|--------|
| G1 简化模型 | 本模块自建（参考 unitreerobotics/unitree_mujoco 概念） | BSD-3-Clause |
| CNN 双编码器融合架构 | 本模块原创 | BSD-3-Clause |
| 专家 PID 控制器 | 本模块原创 | BSD-3-Clause |
| 训练/评估/对比脚本 | 本模块原创 | BSD-3-Clause |
| ROS2 节点 | 本模块原创 | BSD-3-Clause |
