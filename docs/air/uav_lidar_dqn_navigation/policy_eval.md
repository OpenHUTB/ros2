# DQN 策略离线评估

`scripts/eval_policy.py` 用来在没有 ROS、没有 AirSim 的机器上检查已训练的策略：
它读取 `models/train_nav_log.csv` 汇总训练结果，再构造“正前方若干米有墙”的
状态，打印网络输出的 Q 值与动作；如果部署侧安全层 `safety_shield.py` 存在，
同时打印屏蔽后的动作。

## 用法

```bash
cd src/air/uav_lidar_dqn_navigation

# 训练统计 + 贴墙探测
python3 scripts/eval_policy.py

# 指定权重与日志
python3 scripts/eval_policy.py --model models/dqn_nav.pt \
                              --log models/train_nav_log.csv

# 只跑几何与安全层断言（不需要权重）
python3 scripts/eval_policy.py --self-test
```

## 输出示例

```
训练日志: models/train_nav_log.csv
  回合 200 | 整体 62.5% | 末 50 94.0% | 末 20 100.0% | 碰撞 28 | 近失 0

模型: models/dqn_nav.pt
障碍     网络动作        安全层后        Q(6 动作)
无遮挡   forward        -             [32.19, 33.65, 32.97, 32.79, 33.09, 32.56]
5.0 m    forward_left   -             [26.39, 26.93, 27.42, ...]
2.0 m    forward_left   -             [23.76, 18.96, 25.07, ...]
1.0 m    forward_left   -             [19.58, 13.34, 20.53, ...]
0.8 m    forward_left   -             [17.62, 10.94, 18.53, ...]
0.4 m    forward_left   -             [9.11, 2.72, 10.14, ...]
```

## 怎么读

- 无遮挡时选择 `forward`；障碍进入 5 m 内后改为 `forward_left`，
  且 `forward` 的 Q 从 33.65 降到 2.72，说明网络确实在使用雷达特征
- 网络在 0.4 m 时倾向斜向而不是原地转开，这是训练分布外的窗口；
  部署侧由 `safety_shield.py` 在前方 1.0 m 内屏蔽前向动作，输出列会显示
  屏蔽后的 `turn_left` / `hover`
- 控制用的 24 扇区索引与扫描的 ROS 角度约定是两套顺序：前者按 AirSim 机体系
  （索引 0 在左侧），后者按 ROS（`ranges[0]` 在右侧 -90°）

## self-test

`--self-test` 不加载权重，只断言两件事：0.4 m 墙会让正前扇区接近 0，
以及安全层会屏蔽直行。适合在 CI 或换机器后快速确认几何与屏蔽逻辑没被改坏。
