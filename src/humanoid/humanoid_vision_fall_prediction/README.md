# 人形机器人视觉跌倒预测与分级平衡控制

> 课程任务②：实现传感器感知及运动控制 —— 前视相机图像 → MLP 跌倒方向分类 → 踝/髋分级修正

## 功能概述

基于 MuJoCo 3.x 物理仿真，在人形机器人头部挂载前视相机，采集随机外力扰动下的灰度图像，训练一个纯 NumPy 实现的 3 层 MLP（256→128→64→5），实时预测机器人的跌倒方向（正常/前倒/后倒/左倒/右倒），并输出相应的踝/髋关节修正目标角。

### 方法原理

```
前视相机 64×64 灰度图
      │
      ▼ 16×16 平均池化 → flatten (256 维)
┌─────────────────┐
│   3 层 MLP       │
│ 256→128→64→5    │
│  ReLU + Softmax  │
└────────┬────────┘
         │ 5 类概率
         ▼
  normal / forward / backward / left / right
         │
         ▼
  BalanceController：按类别叠加踝/髋修正力矩
```

- **感知**：前视相机（`front_camera`，fovy=60）渲染 64×64 灰度图
- **神经网络**：纯 NumPy 实现的 MLP（无 PyTorch 依赖），交叉熵损失 + SGD 动量
- **控制**：基于预测类别，叠加 `_CORRECTION` 关节角偏移到站姿 PD 目标

## 目录结构

```
humanoid_vision_fall_prediction/
├── main.py                              # standalone 入口（--collect/--train/--demo/--check）
├── main.sh                              # Linux 启动脚本
├── package.xml / setup.py / setup.cfg    # ROS2 ament_python 包配置
├── requirements.txt                     # Python 依赖
├── config/params.yaml                    # 仿真/站姿/采集/网络参数
├── models/humanoid_camera.xml           # MuJoCo MJCF 人形模型（含前视相机）
├── launch/main.launch.py                # ROS2 launch（同时启动仿真+推理节点）
├── humanoid_vision_fall_prediction/
│   ├── sim.py                           # MuJoCo 仿真封装（站姿 PD + 推力）
│   ├── dataset.py                       # 随机推力数据采集
│   ├── nn.py                            # 纯 NumPy MLP
│   ├── train.py                         # 训练脚本
│   ├── predictor.py                     # 推理封装
│   ├── controller.py                    # 分级平衡控制器
│   ├── vision_sim_node.py                # ROS2 仿真节点
│   └── fall_predictor_node.py           # ROS2 推理节点
├── data/fall_dataset.npz                # 示例数据集（约 100 KB）
├── models_trained/fall_predictor.npz     # 示例训练权重
└── test/test_smoke.py                   # 冒烟测试
```

## 环境要求

- Python 3.8+
- MuJoCo ≥ 3.0
- NumPy ≥ 1.21
- PyYAML ≥ 5.4
- （ROS2 节点仅在 Ubuntu 20.04 ROS Humble 下需要 rclpy）

安装：
```bash
pip install -r requirements.txt
```

## 快速开始

### 1. 自检（验证模型/渲染/训练流程）
```bash
python3 main.py --check
```

### 2. 采集数据集
```bash
python3 main.py --collect --samples 300
# 输出：data/fall_dataset.npz
```

### 3. 训练神经网络
```bash
python3 main.py --train
# 输出：models_trained/fall_predictor.npz
# 典型验证准确率：> 85%
```

### 4. 演示（推扰 + 视觉预测 + 平衡控制）
```bash
# 前向推扰 120 N
python3 main.py --demo --force 120 --direction 0

# 侧向推扰
python3 main.py --demo --force 120 --direction 90
```

### 5. ROS2 launch（Ubuntu 20.04 Humble）
```bash
# 编译
colcon build --packages-select humanoid_vision_fall_prediction
source install/setup.bash
# 启动
ros2 launch humanoid_vision_fall_prediction main.launch.py
```

## 性能评价（本地 Windows 实测）

| 指标 | 数值 |
|---|---|
| 数据集规模 | 97 样本（5 类） |
| 训练轮数 | 80 epochs |
| 最终验证准确率 | **89.5%** |
| 推理单帧耗时 | < 1 ms（CPU） |
| 预测类别 | forward / backward / left / right / normal |

## 与已有模块的区别

- 与 `humanoid_push_recovery`（规则式 ZMP/捕获点平衡）不同：本模块**从前视图像学习**跌倒方向，不依赖解析的 ZMP/质心计算。
- 与 PR2（端到端图像→关节力矩）形成对比：本模块是"感知分类 + 规则控制"的模块化方法。

## 参考文献

1. Macenski S, et al. *Robot Operating System 2: Design, architecture, and uses in the wild.* Science Robotics, 2022.
2. MuJoCo Documentation: https://mujoco.readthedocs.io/
