# 人形机器人多模态跌倒预警

> 课程任务②：多模态融合 —— 图像特征(256) + IMU(2) → MLP → 二分类（正常/即将跌倒）

## 功能概述

融合前视图像特征（256维）与躯干IMU姿态（pitch/roll，2维），通过MLP（258→128→64→2）判断机器人是否即将跌倒。当跌倒概率>0.6时触发紧急下蹲姿态。

与PR1（纯RGB 5分类）的区别：本模块融合IMU传感器，且是二分类预警。

## 快速开始

```bash
pip install -r requirements.txt
python3 main.py --collect --samples 2000
python3 main.py --train
python3 main.py --demo
# ROS2:
colcon build --packages-select humanoid_fall_warning
source install/setup.bash
ros2 launch humanoid_fall_warning main.launch.py
```

## 网络结构

```
前视图像 16×16池化→256维  +  IMU(pitch,roll) 2维
  → 拼接为258维
  → MLP 258→128→64→2
  → Softmax → (正常概率, 跌倒概率)
  → 跌倒概率>0.6 → 紧急下蹲
```

## 多模态对比

| 模块 | 输入模态 | 输出 | 任务 |
|---|---|---|---|
| PR#423 | 纯RGB | 5类跌倒方向 | 分类 |
| 本模块 | RGB+IMU | 2类（正常/跌倒） | 二分类预警 |
