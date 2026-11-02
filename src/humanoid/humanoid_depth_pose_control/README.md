# 人形机器人深度图姿态估计与比例平衡控制

> 课程任务②：深度相机感知 + 运动控制 —— 深度图 → MLP回归3D躯干姿态 → 比例平衡控制

## 功能概述

基于深度相机（front_camera depth）渲染64×64深度图，训练纯NumPy实现的3层MLP（256→128→64→3），从深度图回归躯干的pitch/roll/yaw三个姿态角，再通过比例控制器输出15维关节修正目标。

与PR1（RGB灰度图→5类跌倒方向分类）的区别：本模块用**深度图**做**连续姿态回归**，输出是连续角度而非离散类别。

## 快速开始

```bash
# 安装依赖
pip install -r requirements.txt

# 自检
python3 main.py --check

# 采集深度图+姿态标签（需要MuJoCo）
python3 main.py --collect --samples 2000

# 训练
python3 main.py --train

# 演示
python3 main.py --demo --force 120 --direction 0

# ROS2 launch（Ubuntu 20.04 Humble）
colcon build --packages-select humanoid_depth_pose_control
source install/setup.bash
ros2 launch humanoid_depth_pose_control main.launch.py
```

## 网络结构

```
深度图 64×64 → 16×16平均池化 → 256维
  → MLP 256→128→64→3
  → (pitch, roll, yaw) 弧度
  → 比例控制器 → 15维关节目标角
```

## 性能指标

| 指标 | 数值 |
|---|---|
| 输入 | 64×64 深度图 |
| 输出 | 3维躯干姿态角 |
| 损失函数 | MSE |
| 网络参数量 | ~41K |
| 推理耗时 | <1ms/帧 |
