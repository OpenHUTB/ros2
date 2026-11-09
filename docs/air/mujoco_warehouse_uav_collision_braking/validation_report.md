# 验证记录

## 输入与命令

数据由 `seed=42`、30条轨迹、三个仓储场景产生；训练分区18条轨迹，验证分区6条，测试分区6条。模型使用 NumPy MLP，隐藏层为24和16个单元。

```bash
python3 main.py --collect --episodes 30 --seed 42 --duration 8 --output dataset
python3 main.py --train --dataset dataset --model model --seed 42
python3 main.py --evaluate --dataset dataset --model model
python3 main.py --benchmark --model model --output results
```

## 结果

| 指标| 值|
| 测试样本| 2008 |
| 风险准确率| 0.768 |
| 速度MAE| 0.522 m/s |
| 静态接触| 否 |
| 横穿接触| 否 |
| 窄道接触| 否 |

学习速度经过参考制动速度上限，三类场景8秒运行均无接触。窄道风险一致率为0.000，表明横向几何样本仍需扩充；该场景的速度输出仍由安全上限约束。

## 复查范围

检查覆盖数据分区隔离、模型保存与加载、输入输出形状、速度范围、ROS 话题名称和三场景基准输出。
学习 launch 在 `duration:=2.0` 时由传感器和学习控制节点共同完成并正常退出。
