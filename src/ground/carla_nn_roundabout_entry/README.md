# 无信号环岛汇入仿真

本模块模拟车辆驶入无信号环岛时与环道车流的交互，提供可复现的车辆运动学、键盘接管和基础安全指标。车辆采用单轨运动学模型，环道车辆按固定随机种子运行。

## 环境

- Python 3.10 或更高版本
- Tkinter（图形交互模式）

## 运行

在仓库根目录执行：

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --gui
```

按 W/上方向键加速，S/下方向键制动，A/D 或左右方向键转向。无图形界面时运行可重复的 headless 仿真：

```bash
python3 src/ground/carla_nn_roundabout_entry/main.py --seed 7 --steps 360
```

命令行会输出运行时间、车辆状态、碰撞帧数和最小离障距离。`--seed` 固定环道车辆初始位置与速度，便于比较后续感知与控制策略。
