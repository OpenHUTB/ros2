# uav_rotor_fault_detector_nn

用机载 IMU 和控制器下发的推力指令，实时估计四旋翼**每个桨还剩多少推力权限**（健康度 `g ∈ (0, 1]`）。

这个包对应课程任务②里"传感器感知"的部分：把"飞机现在坏没坏、坏得有多重"变成一个可以喂给
上层控制器的连续量。它不负责容错控制，只负责**在坠机之前把故障报出来**。

## 为什么需要这一层

`uav_actuator_fault_sim` 已经能量化"单桨完全失效有多致命"：悬停时 0.64 s 就坠地。
要在那 0.64 s 里做出反应，控制器必须先用传感器数据判断出"是哪一桨、剩多少"。
靠阈值判断姿态误差是做不到的——推力衰减 40 % 时姿态几乎看不出来，但剩余权限已经只有 0.6。

实测（240 次飞行、11600 个滑窗样本训练后，测试集 1039 个样本）：

| 指标 | 数值 |
| --- | --- |
| 四桨增益 MAE | 0.0087 |
| 故障帧召回率 | 0.910 |
| 精确率 / 误报率 | 1.000 / 0.000 |
| 单桨完全失效检出延时 | 0.10 s |
| 渐进退化检出延时 | 0.80 s |

悬停 1.5 m 下单桨失效到坠地约 0.64 s，0.10 s 的检出延时给上层留出约 0.5 s 的处置窗口。

## 目录结构

```
uav_rotor_fault_detector_nn/
├── uav_rotor_fault_detector_nn/
│   ├── features.py      # 13 维特征与滑窗
│   ├── data.py          # 故障脚本、飞行数据生成、数据集划分
│   ├── model.py         # 1D-CNN + GRU 网络与检查点读写
│   ├── train.py         # 训练循环
│   ├── infer.py         # 在线估计器（滑窗缓冲）
│   ├── metrics.py       # MAE / 召回 / 误报 / 检测延时
│   ├── evaluate.py      # 测试集报告
│   ├── render.py        # 对比动图
│   ├── ros_node.py      # ROS2 节点
│   ├── selftest.py      # 51 项自检（无需 ROS）
│   └── cli.py           # 命令行入口
├── config/detector.yaml
├── launch/detector.launch.py
├── tools/gen_figures.py     # 生成文档配图
├── test/                    # pytest（51 个用例）
├── results/                 # 权重、指标、动图（数据集不入库）
├── main.py  main.sh  main.bat
└── README.md
```

## 运行环境

| 项 | 要求 |
| --- | --- |
| Python | ≥ 3.10 |
| 依赖 | `torch ≥ 2.0`、`numpy ≥ 1.21`、`mujoco ≥ 3.0`；画图另需 `matplotlib`、`pillow` |
| ROS | ROS2 Humble（仅 `--node` / `ros2 launch` 需要） |
| 硬件 | 普通 CPU 即可；不需要独显、不需要 Gazebo、不需要 Unreal |

仿真环境由已合并的 `src/air/uav_mujoco_sim` 提供，本包**不修改它**。

## 快速开始

```bash
pip install -r requirements.txt

python main.py --selftest                       # 51 项自检，不需要 ROS
python main.py --gen-data results/detector_dataset.npz --episodes 240   # 约 26 s
python main.py --train --threads 8              # 约 52 s（CPU）
python main.py --eval test                      # 测试集报告
python main.py --demo results/detection_demo.gif # 真值 vs 估计对比动图
python tools/gen_figures.py                     # 训练曲线、分故障指标图
```

`--gen-data` 写出的数据集约 7 MB，不入库；`results/` 里只提交权重和动图。

## ROS2 启动

```bash
colcon build --packages-select uav_mujoco_sim uav_rotor_fault_detector_nn
ros2 launch uav_rotor_fault_detector_nn detector.launch.py
ros2 launch uav_rotor_fault_detector_nn detector.launch.py fault:=degradation rotor:=2 severity:=0.4
```

节点内部闭环仿真并在线估计，把真值和估计同时发出来，方便在 rviz2 / rqt 里直接对照。

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav/odom` | `nav_msgs/Odometry` | 发布 | 位姿与速度（ENU） |
| `/uav/status` | `std_msgs/String` | 发布 | `WARMUP` / `MONITOR` / `FAULT` / `CRASHED` / `DONE` |
| `/uav/health_true` | `std_msgs/Float32MultiArray` | 发布 | 四桨真实增益（仿真真值） |
| `/uav/health_est` | `std_msgs/Float32MultiArray` | 发布 | 四桨健康度估计 |
| `/uav/fault` | `std_msgs/String` | 发布 | 检测到故障时的文字告警 |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 速度指令（换算成悬停目标点） |
| `/uav/goal` | `geometry_msgs/Point` | 订阅 | 直接给悬停目标 |

参数（`config/detector.yaml`）：`scene`、`fault`、`rotor`、`onset`、`severity`、`imu_noise`、
`checkpoint`、`window`、`duration`、`publish_rate`。

## 输入特征与网络

每步取 13 维：

```
[ax, ay, az, gx, gy, gz, vx, vy, vz, u0, u1, u2, u3]
 └─ 机身比力 ─┘└ 角速度 ┘└ 速度 ┘ └ 指令推力 ┘
```

机身比力由速度差分反推：`f_body = R(q)ᵀ (dv/dt + g·ẑ)`。四个指令推力是控制器的输出
（故障**之前**的值），除以 `max_thrust` 归一化——它提供了"控制器想要多少推力"这个参考，
故障时实际加速度与期望不符，这个差值就是网络要学的东西。

网络 `(B, 100, 13) → (B, 4)`：

```
Conv1d(13→32, k5) → BN → ReLU → Conv1d(32→32, k5) → BN → ReLU → MaxPool(2)
→ Conv1d(32→48, k3) → BN → ReLU → MaxPool(2)
→ GRU(48 → 64) → 取末隐状态 → Linear(64→64) → ReLU → Dropout → Linear(64→4) → Sigmoid
```

参数量 **38452**，CPU 上单次前向 < 1 ms（窗口 100 步）。

损失用加权 MSE：含故障的样本权重 ×4（故障样本在数据集中只占少数），优化器 AdamW +
`ReduceLROnPlateau`，按验证集 MAE 早停。

## 结果

训练 30 epoch，最佳 epoch 28，验证 MAE **0.0075**，参数量 38452。测试集 1039 个样本：

| 故障类型 | 样本 | MAE | 召回 | 误报 | 检测延时/s |
| --- | --- | --- | --- | --- | --- |
| 单桨推力衰减 | 169 | 0.010 | 0.980 | 0.000 | 0.000 |
| 双桨失效 | 138 | 0.019 | 0.974 | 0.000 | 0.300 |
| 健康 | 142 | 0.004 | – | 0.000 | – |
| IMU 零偏 | 213 | 0.003 | – | 0.000 | – |
| 单桨完全失效 | 152 | 0.009 | 0.929 | 0.000 | 0.100 |
| 渐进退化 | 121 | 0.008 | 0.806 | 0.000 | 0.800 |
| 桨卡死 | 104 | 0.011 | 0.806 | 0.000 | 0.200 |

混淆矩阵 TP=253 FP=0 TN=761 FN=25，准确率 0.976。

补两张图（生成于 `docs/img/air/uav_rotor_fault_detector_nn/`）：

- `training_curve.png`：训练损失与验证 MAE
- `detection_by_fault.png`：分故障类型的召回率与检测延时

## 局限

1. **数据来自仿真**。IMU 噪声按白噪声建模，真实机架的振动与桨叶损伤机理不同，
   迁移到实机需要重新标定噪声模型。
2. **只估计推力权限**，不估计故障类型（桨卡死和完全失效在增益上都表现为 0）。
   区分二者需要看电调反馈，本仿真没有这一路信息。
3. **依赖指令推力**。真实机载系统里这个量在飞控内部，需要额外透传；
   若只能拿到 IMU，需改成本包 `infer.py` 的接口换成隐状态观测。
4. 检测延时在渐进退化上偏长（0.80 s），因为早期增益变化小于噪声幅度，
   属于信息量本身的限制，加大窗口可以换一点延时（代价是响应更慢）。

## 参考

1. Mueller, M. W., D'Andrea, R. *Stability and control of a quadrocopter despite the complete
   loss of one, two, or three propellers.* ICRA 2014.
2. 李航. *统计学习方法*. 清华大学出版社.（CNN / GRU / 加权损失的基本推导）
3. [uav_actuator_fault_sim](../uav_actuator_fault_sim/README.md)：故障注入与评测台
