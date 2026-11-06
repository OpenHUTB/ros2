# carla_rl_driving

地面载具（CARLA）端到端强化学习自动驾驶：单目前视图像 → 神经网络策略 → 阿克曼控制指令。

- 任务覆盖：任务二（传感器感知 + 运动控制）、任务四（输入图像输出控制指令的端到端模型）。
- 对比实验：同模型不同 RL 算法（PPO vs SAC）；同 RL 算法不同网络结构（CNN vs CNN+GRU）。
- 启动：`ros2 launch carla_rl_driving carla_rl_train.launch.py` / `carla_rl_infer.launch.py`。

完整教程见文档站点：`docs/ground/rl_driving.md`。

## 目录

```
carla_rl_driving/
├── carla_rl_driving/
│   ├── env.py          # gym.Env，对接 CARLA 同步模式
│   ├── reward.py       # 奖励塑形公式
│   ├── models.py       # CNN / CNN+GRU 策略网络
│   ├── train.py        # SB3 PPO/SAC 训练入口
│   ├── infer_node.py   # rclpy 部署推理节点
│   └── utils.py        # 图像预处理
├── launch/*.launch.py # 一键启动
└── config/rl_driving.yaml
```
