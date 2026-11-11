# uav_airsim_depth_obstacle_avoidance

AirSim 仿真里的无人机深度图避障：**前视深度图 64×64 → 小型 CNN → (偏航角速率, 目标高度)**。

专家策略只读深度图（不看真值位姿），网络从零手写 NumPy 实现，不依赖 PyTorch。

## 目录

```
uav_airsim_depth_obstacle_avoidance/
├── main.py                       # 四合一入口：采集 / 训练 / 演示 / 评估
├── launch/main.launch.py         # ros2 launch 入口
├── models_trained/               # 训练权重与样本
└── uav_airsim_depth_obstacle_avoidance/
    ├── env.py                    # AirSim 环境封装（深度图、位姿、控制）
    ├── cnn.py                    # 手写 CNN（im2col 卷积 + 池化 + 全连接 + Adam）
    ├── policy.py                 # 专家策略（带滞回）与网络策略
    ├── dataset.py                # 样本采集
    ├── train.py                  # 训练（翻转增强 + 动作加权）
    ├── demo.py                   # 实飞演示并录 gif
    └── node.py                   # ROS 2 节点
```

## 快速开始

```bash
# 1. 启动 AirSim Blocks 场景（Windows 下双击 Blocks.exe，等 40 s 端口 41451 就绪）
# 2. 采集专家样本
python3 main.py --collect --episodes 6 --steps 120
# 3. 训练
python3 main.py --train --epochs 80
# 4. 实飞并录 gif
python3 main.py --demo --steps 180 --gif demo.gif
# 5. 网络与专家的动作偏差
python3 main.py --eval
```

ROS 2 启动：

```bash
ros2 launch uav_airsim_depth_obstacle_avoidance main.launch.py
```

## 结果

| 指标 | 数值 |
|---|---|
| 训练样本 | 720（6 回合 × 120 步） |
| 验证 MSE | 0.0640 |
| 验证 R² | 0.298 |
| 演示 gif | 4.2 MB，180 帧 |
| 演示中最小深度 | 0.10（约 3 m） |
| 演示平均每帧最小深度 | 0.25（约 7.5 m） |

完整说明见 [docs/air/uav_airsim_depth_obstacle_avoidance.md](../../../docs/air/uav_airsim_depth_obstacle_avoidance.md)。
