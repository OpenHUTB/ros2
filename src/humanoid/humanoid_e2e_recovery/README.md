# 人形机器人端到端视觉平衡控制（行为克隆）

> 课程任务④：端到端模型（输入图像、输出控制指令）—— 前视相机图像 → MLP → 15 维关节目标角

## 功能概述

基于行为克隆（Behavioral Cloning）方法，用一个简单的比例控制器作为"专家"，在随机外力扰动下采集前视图像与专家关节目标角的配对数据，训练一个纯 NumPy 实现的 3 层 MLP 直接从图像回归 15 维关节目标角。与 PR1（分类+规则控制）形成**模块化 vs 端到端**的方法论对比。

### 方法原理

```
前视相机 64×64 灰度图
      │
      ▼ 16×16 平均池化 → flatten (256 维)
┌─────────────────┐
│   3 层 MLP        │
│ 256→128→64→15    │
│  ReLU + 线性输出   │
└────────┬────────┘
         │ 15 维关节目标角
         ▼
  MuJoCo 站姿 PD 执行
```

- **专家控制器**：基于躯干前/后/左/右倾角的比例控制器（`expert.py`）
- **端到端网络**：图像 → 15 维关节目标角（回归任务，MSE 损失）
- **对比 PR1**：PR1 是"感知分类（5类）+ 规则控制"的模块化方法；PR2 是"图像直接→关节指令"的端到端方法

## 目录结构

```
humanoid_e2e_recovery/
├── main.py                              # 入口（--collect/--train/--demo/--check）
├── models/humanoid_camera.xml           # 人形模型（含前视相机）
├── launch/main.launch.py                # ROS2 launch
├── humanoid_e2e_recovery/
│   ├── sim.py                           # MuJoCo 仿真封装
│   ├── expert.py                        # 专家比例控制器
│   ├── e2e_nn.py                        # 纯 NumPy MLP（回归）
│   ├── dataset.py                       # 专家数据采集
│   ├── train.py                         # 训练
│   └── predictor.py                     # 端到端推理
├── data/e2e_dataset.npz               # 示例数据集
└── models_trained/e2e_policy.npz       # 训练权重
```

## 快速开始

```bash
# 安装依赖
pip install -r requirements.txt

# 自检
python3 main.py --check

# 采集专家数据（30 episodes）
python3 main.py --collect --episodes 30

# 训练
python3 main.py --train

# 演示
python3 main.py --demo --force 100 --direction 0
```

## 性能指标

| 指标 | 数值 |
|---|---|
| 数据集规模 | 8943 样本 |
| 输入维度 | 256（16×16 灰度图） |
| 输出维度 | 15（关节目标角） |
| 网络参数量 | ~41 K |
| 验证 MSE | **2.8e-05 rad²**（关节角误差 ~0.3°） |
| 单帧推理 | < 1 ms |

## 与 PR1 的对比

| 维度 | PR1（模块化） | PR2（端到端） |
|---|---|---|
| 感知输出 | 5 类跌倒方向 | 15 维关节角 |
| 控制方式 | 规则查表（按类别叠加修正） | 网络直接回归 |
| 损失函数 | 交叉熵 | MSE |
| 可解释性 | 高（知道预测了哪类） | 低（黑盒映射） |
