# 验证记录

## 输入与命令

数据由 `seed=42`、30条轨迹、三个仓储场景产生；训练分区18条轨迹，验证分区6条，测试分区6条。模型使用 NumPy MLP，隐藏层为24和16个单元。

```bash
python3 main.py --collect --episodes 30 --seed 42 --duration 8
python3 main.py --train --dataset dataset --model model --seed 42
python3 main.py --evaluate --dataset dataset --model model
python3 main.py --benchmark --model model --output results
```

## 结果

| 指标 | 值 |
| --- | --- |
| 测试样本 | 2008 |
| 风险准确率 | 0.770 |
| 速度 MAE | 0.820 m/s |
| 参考控制静态接触 | 否 |
| 参考控制横穿接触 | 否 |
| 参考控制窄道接触 | 否 |

纯学习控制在三个场景均发生接触；加参考制动速度上限后，三类场景8秒运行均无接触。风险一致率分别为静态0.818、横穿0.753、窄道0.893。该对照表明约束控制结果不能等同于模型自身性能。

离线基准使用 MuJoCo 纵向速度真值构造模型输入和参考标签；ROS 学习节点根据 `/warehouse/pose` 相邻位姿差分估速。基准结果描述固定仿真条件下的控制组对照，不代表 ROS 在线节点的实测结果。

## 复查范围

检查覆盖数据分区隔离、模型保存与加载、输入输出形状、速度范围、ROS 话题名称和三场景基准输出。
学习 launch 在 `duration:=2.0` 时由传感器和学习控制节点共同完成并正常退出。
