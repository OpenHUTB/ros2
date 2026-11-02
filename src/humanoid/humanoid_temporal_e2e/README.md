# 人形机器人多帧时序端到端视觉平衡控制

> 课程任务④：端到端模型 —— 连续3帧前视图像 → MLP → 15维关节目标角

## 功能概述

与PR2（单帧图像→关节角）不同，本模块使用**连续3帧前视图像**作为输入，捕捉时序动态信息，输出15维关节目标角。网络结构为768(3×256)→256→128→15。

## 快速开始

```bash
pip install -r requirements.txt
python3 main.py --collect --episodes 30
python3 main.py --train
python3 main.py --demo
# ROS2:
colcon build --packages-select humanoid_temporal_e2e
source install/setup.bash
ros2 launch humanoid_temporal_e2e main.launch.py
```

## 网络结构

```
3帧前视图像 → 每帧16×16池化→256维 → 拼接为768维
  → MLP 768→256→128→15
  → 15维关节目标角
```

## 与PR2对比

| 维度 | PR2（单帧） | 本模块（3帧时序） |
|---|---|---|
| 输入 | 256维（1帧） | 768维（3帧拼接） |
| 时序信息 | 无 | 有（3帧滑动窗口） |
| 网络 | 256→128→64→15 | 768→256→128→15 |
