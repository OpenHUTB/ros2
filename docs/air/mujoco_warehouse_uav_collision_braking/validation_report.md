# 验证记录 / Validation Record

## 输入与命令 / Inputs and Commands

数据由 `seed=42`、30条轨迹、三个仓储场景产生；训练分区18条轨迹，验证分区6条，测试分区6条。模型使用 NumPy MLP，隐藏层为24和16个单元。
The dataset uses `seed=42`, 30 episodes and three warehouse scenes; 18 episodes are used for training, 6 for validation and 6 for testing. The model is a NumPy MLP with hidden layers of 24 and 16 units.

```bash
python3 main.py --collect --episodes 30 --seed 42 --duration 8 --output dataset
python3 main.py --train --dataset dataset --model model --seed 42
python3 main.py --evaluate --dataset dataset --model model
python3 main.py --benchmark --model model --output results
```

## 结果 / Results

| 指标 / Metric | 值 / Value |
| --- | ---: |
| 测试样本 / Test samples | 2008 |
| 风险准确率 / Risk accuracy | 0.768 |
| 速度MAE / Speed MAE | 0.522 m/s |
| 静态接触 / Static contact | No |
| 横穿接触 / Crossing contact | No |
| 窄道接触 / Narrow contact | No |

学习速度经过参考制动速度上限，三类场景8秒运行均无接触。窄道风险一致率为0.000，表明横向几何样本仍需扩充；该场景的速度输出仍由安全上限约束。
Learned speed is clipped by the reference braking limit, and all three 8-second scenes complete without contact. Narrow-scene risk agreement is 0.000, indicating that lateral-geometry coverage should be expanded; speed output remains bounded by the safety limit.

## 复查范围 / Review Scope

检查覆盖数据分区隔离、模型保存与加载、输入输出形状、速度范围、ROS 话题名称和三场景基准输出。
Checks cover split isolation, model save/load, input/output shapes, speed bounds, ROS topic names and three-scenario benchmark output.
学习 launch 在 `duration:=2.0` 时由传感器和学习控制节点共同完成并正常退出。
The learned launch completes with both sensor and learned-control nodes and exits cleanly with `duration:=2.0`.
