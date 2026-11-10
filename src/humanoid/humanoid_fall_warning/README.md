# 人形机器人多模态跌倒预警

> 课程任务②：多模态融合 —— 图像特征(256) + IMU(2) → MLP → 二分类（正常/即将跌倒）

![demo](https://raw.githubusercontent.com/WZJ6666wudu/ros2/feat/humanoid-fall-warning/docs/img/humanoid/humanoid_fall_warning/demo.gif)

上图为 `main.py --demo --force 150 --gif ...` 的产物：MuJoCo `overview` 机位逐帧真实渲染
（154 帧，2.5 MB），画面左上角叠加的是模型实时输出的 p_normal / p_fall 与当前控制模式。

## 功能概述

融合前视图像特征（256 维）与躯干 IMU 姿态（pitch/roll，2 维），用纯 NumPy 实现的
MLP（258→128→64→2）判断"未来 1.2 s 内是否会跌倒"。p_fall > 0.6 时输出紧急下蹲姿态
（髋屈 + 膝屈 + 踝背屈），用降低重心换时间。

与 PR#423（纯 RGB 5 分类）的区别：本模块融合 IMU，且从"分类已成事实的跌倒方向"改为
"预测尚未发生的跌倒"，所以标签口径、采集时刻、评估指标都不同。

## 快速开始

环境：Ubuntu 22.04.5 + ROS 2 Humble + Python 3.12 + MuJoCo 4.0.0（无 GPU 也能跑）。

```bash
pip install -r requirements.txt
cd src/humanoid/humanoid_fall_warning

python3 main.py --check                  # 7 项自检：模型/站姿/图像/数据集/权重/推理/控制器
python3 main.py --collect --samples 1500 # 采集（约 4 分钟，CPU）
python3 main.py --train --epochs 80      # 训练并打印混淆矩阵、基率对照与消融
python3 main.py --demo --force 150 --gif demo.gif
python3 -m pytest test/ -q
```

ROS 2 方式：

```bash
colcon build --packages-select humanoid_fall_warning
source install/setup.bash
ros2 launch humanoid_fall_warning main.launch.py
```

## 网络结构

```
前视图像 64×64 → 16×16 均值池化 → 256 维
躯干 IMU → (pitch, roll) → 2 维
  → 拼接 258 维 → 逐维标准化（均值/方差随权重一起存）
  → MLP 258→128→64→2 → softmax → (p_normal, p_fall)
  → p_fall > 0.6 → 紧急下蹲（SQUAT_TARGET）
```

## 实测结果（验证集 300 条，阈值 0.6）

| 指标 | 本模块 | 基率（永远预测多数类） |
|---|---|---|
| 混淆矩阵 | TN=113 FP=0 FN=0 TP=187 | TN=0 FP=114 FN=0 TP=187 |
| accuracy | 1.000 | 0.623 |
| precision | 1.000 | 0.623 |
| recall | 1.000 | 1.000 |
| F1 | 1.000 | 0.768 |
| specificity | 1.000 | 0.000 |

模态消融（把一路模态替换为训练集均值）：

| 输入 | accuracy | recall | F1 |
|---|---|---|---|
| 图像 + IMU | 1.000 | 1.000 | 1.000 |
| 仅 IMU（图像替换为均值） | 0.780 | 0.706 | 0.800 |
| 仅图像（IMU 替换为均值） | 0.987 | 0.984 | 0.989 |

图像是主信息源，IMU 提供补充，融合后最优 —— 这是"多模态"在本任务里的实际收益。

## 目录

```
humanoid_fall_warning/
├── main.py                    # --check / --collect / --train / --demo --gif
├── sim.py                     # MuJoCo 封装：站姿配平 + 踝策略平衡层 + 真实 RGB 渲染
├── dataset.py                 # 扰动后采集，标签 = 未来 1.2 s 内是否跌倒
├── nn.py                      # 纯 NumPy MLP（Adam + 输入标准化）
├── train.py                   # 训练 + 混淆矩阵 + 基率对照 + 模态消融
├── predictor.py / predictor_node.py / sim_node.py
├── models/humanoid_warning.xml
├── models_trained/fall_warning.npz
└── test/test_fall_warning.py  # 8 条回归测试
```

## 与 PR#423 的对比

| 模块 | 输入模态 | 输出 | 任务 |
|---|---|---|---|
| PR#423 | 纯 RGB | 5 类跌倒方向 | 分类（事实发生后） |
| 本模块 | RGB + IMU | 2 类（正常/将跌倒） | 预警（事实发生前 1.2 s） |
