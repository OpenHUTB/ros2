# 基于 GRU 的无人机多时域飞行状态预测与 ROS2 在线误差分析

本章记录一个可独立复现的无人机飞行状态预测实验：用一层的 GRU（门控循环单元）网络，从过去 1 秒的飞行状态历史预测未来 0.25 / 0.5 / 1.0 秒的位置与速度；预测结果通过 ROS2 话题实时发布，并在目标时刻的真实状态到达后自动对齐、计算误差。

网络并不直接回归绝对位置，而是回归**相对恒速模型的残差**。这个设计让网络只需学习"运动偏离匀速直线运动的那一部分"，在小样本、弱机动的飞行数据上更容易训练。

全部结果都在 Ubuntu 20.04 + ROS2 Humble + AirSim Blocks 场景的仿真环境中实测得到；表格里的每个数字都可以用第 10 节的命令重新跑出来。

![ROS2 实时回放与在线误差曲线](../img/air/state_prediction/demo.gif)


---

## 1. 任务定义与设计取舍

**输入**：最近 20 帧飞行状态（20 Hz 采样，约 1 秒），每帧 16 维。

**输出**：未来 3 个时域（0.25 s / 0.5 s / 1.0 s），每个时域给出位置（3 维）与速度（3 维），共 18 维。

**约束**：不使用未来的控制指令。这意味着模型只能依赖状态自身的时间演化——这对无人机是现实的：飞控指令在预测时不可知。

三个关键取舍：

| 取舍 | 选择 | 理由 |
|---|---|---|
| 预测目标 | 预测**残差**而非绝对位置 | 恒速基线已能解释大部分运动；残差的量级小一到两个数量级，网络只需拟合偏差 |
| 序列模型 | GRU 而非 LSTM / Transformer | 单层 48 隐单元已足够（10,386 参数），CPU 单样本推理 p50 0.79 ms、p95 1.32 ms；更深的模型在小数据上过拟合 |
| 评估方式 | 目标时刻**对齐**而非固定 lead time | 状态以不规则间隔到达，只有按真实目标时刻插值才能得到无偏误差 |

### 1.1 功能与入口

`main.py` 是统一入口，`main.sh` 负责加载 ROS 环境并激活虚拟环境后转交执行：

| 阶段 | 入口 | 输出 |
|---|---|---|
| 仿真采集 | `bash main.sh collect --host <IP>` | `data/raw/episode_*.csv` / `.json` |
| 数据集 | `bash main.sh prepare` | `data/processed/*.npz`、`manifest.json` |
| 神经网络 | `bash main.sh train --output models/retrained` | `models/<variant>_<seed>/model.pt`、`loss.csv` |
| 独立测试 | `bash main.sh evaluate --output results/xxx` | `metrics.csv`、`episode_metrics.csv`、4 张图 |
| 构建 | `bash main.sh build` | `install/`、`build/`、`log/` |
| ROS2 回放 | `bash main.sh demo` | `/uav/state`、预测路径、误差与可视化 |
| ROS2 实时 | `bash main.sh live --host <IP>` | 同上，数据来自 AirSim RPC |
| 自检 | `bash main.sh doctor` | `artifacts/environment.json` |

`demo` 用仓库内的真实回放数据 `data/sample/episode_026.csv`，能完整演示 **状态发布 → ROS2 → GRU → 预测发布 → 未来真实值到达 → 误差发布 → 绘图** 这条链路；`live` 直接从 AirSim RPC 取状态。**只有 `collect` 会控制仿真无人机**，回放与只读桥接都不下发飞控指令。

---
