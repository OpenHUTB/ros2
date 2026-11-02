# pix2cmd_end2end_driving —— 相机图像到控制指令的端到端驾驶（pix2cmd）

> 课程任务（4）：**每种机器人端到端模型（输入图像、输出控制指令）**。
> 输入 48×48 单通道前视相机图像，直接输出 (转向, 油门)，中间没有人工的
> "感知→规划→控制"分层。网络、反向传播、Adam 优化全部用 numpy 手写，
> **不需要 PyTorch / TensorFlow / GPU**，也**不需要 CARLA / AirSim / Gazebo**。

完整原理推导、算法流程、源码解析与实测对比见文档：
[`docs/ground/pix2cmd_end2end_driving.md`](../../../docs/ground/pix2cmd_end2end_driving.md)

## 运行环境

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Windows 10/11、Ubuntu 20.04 / 22.04 |
| Python | 3.8 及以上（已在 3.8.10 / 3.10.6 / 3.11.15 验证） |
| 依赖 | `numpy`、`matplotlib`、`pillow`（`requirements.txt`） |
| ROS（可选） | ROS 2 Humble / ROS Noetic，仅 `--ros` 模式需要 |
| 硬件 | 普通 CPU 即可，四种结构全部训练完约 12 s |

## 运行步骤

### 1. 安装依赖

```bash
cd src/ground/pix2cmd_end2end_driving
pip3 install -r requirements.txt
```

### 2. 训练 + 闭环演示（默认模式）

```bash
python3 main.py --demo --output-dir results
```

Windows 下可直接双击 / 执行 `main.bat`，Linux 下执行 `./main.sh`，两者等价于上面的命令。

终端会打印每轮训练损失与验证 MAE，随后打印闭环指标，并在 `results/` 下生成：

| 文件 | 内容 |
| --- | --- |
| `training_curve.png` | 训练 MSE 与验证 MAE 曲线 |
| `predictions.png` | 验证集预测值 vs 真值散点 |
| `samples.png` | 6 帧相机输入及预测转向 |
| `drive_demo.gif` | 闭环驾驶动图（120 帧，约 144 KB） |
| `metrics.json` | 全部指标 |

### 3. 结构对比（同一任务、四种网络结构）

```bash
python3 main.py --sweep --output-dir results
```

输出 `results/arch_comparison.csv`、`results/arch_comparison.png`，并在终端打印
`mlp / cnn_tiny / cnn_base / cnn_wide` 四种结构在同一数据集、同一闭环评价下的对比表。

### 4. 单元测试

```bash
python3 main.py --test
# 或
python3 -m pytest -q test
```

预期 `17 passed`（不需要 ROS，也不需要任何仿真器）。

### 5. ROS 2 运行

```bash
# 在 colcon 工作空间根目录
colcon build --packages-select pix2cmd_end2end_driving --symlink-install
source install/setup.bash
ros2 launch pix2cmd_end2end_driving main.launch.py
```

另一个终端观察：

```bash
ros2 topic echo /pix2cmd/cmd      # [steering, throttle]
ros2 topic echo /pix2cmd/state    # 带横向误差的可读状态
ros2 topic hz   /pix2cmd/camera   # 相机图像发布频率
```

节点启动时先训练一次网络（默认 800 样本 8 轮）并打印验证 MAE，之后以 10 Hz 发布图像与控制指令。

## 目录结构

```
pix2cmd_end2end_driving/
├── main.py / main.sh / main.bat        # 课程约定主入口（main.*）
├── launch/main.launch.py               # ROS 2 launch 入口
├── config/params.yaml                  # 话题名与训练超参
├── pix2cmd/
│   ├── dataset.py                      # 针孔相机投影渲染 + 纯追踪标签
│   ├── cnn.py                          # im2col 卷积、手写反向传播、Adam
│   ├── train.py                        # 训练循环、闭环驾驶、出图与 GIF
│   ├── arch_sweep.py                   # 四种结构对比 + 直线回正实验
│   └── node.py                         # ROS 2 节点
├── test/test_pix2cmd.py                # 17 项离线单元测试
└── requirements.txt
```

## 典型输出

```
参数数量            : 63298
最终 train MSE      : 0.000258
验证 MAE (steering) : 0.01505
验证 MAE (throttle) : 0.01303

--- 闭环驾驶 ---
steps                       : 120
mean_abs_lateral_error_m    : 0.3113
max_abs_lateral_error_m     : 0.8356
within_lane_ratio           : 0.9

动图: results/drive_demo.gif  (144 KB, 120 帧)
```
