# 无人机飞行轨迹类型识别与 ROS2 自动任务日志

本页记录一个可独立复现的飞行轨迹分类实验：用过去 **6 秒**的飞行遥测，识别无人机当前在执行 **五类轨迹**中的哪一类，并在 ROS2 中把稳定的识别结果转成**任务事件**。

模型不是序列到序列的黑箱：先从遥测里抽出 8 个有物理含义的几何特征，再在时间维上取均值与标准差，最后过一个 MLP。GRU 作为序列模型对照，放在同一份数据上比较。

全部数字都在本机重跑过，包括**重训结果与仓库随附权重逐位一致**的验证（第 7.5 节）。

![真实 ROS2 看板：回放两段飞行，识别由 line 切换到 figure_eight](../img/air/flight_pattern_recognition/demo.gif)

上图为 `ros2` 实跑窗口：左侧是收到的轨迹，右上是五类的 softmax 输出，左下是识别时间线，右下是自动任务日志。回放 `episode_300`（直线）与 `episode_302`（8 字），两段之间触发一次任务事件。

---

## 1. 任务定义与设计取舍

**输入**：最近 6 秒遥测（10 Hz 采样，61 帧），每帧取位置、速度、加速度共 9 个分量。

**输出**：五类之一 `line` / `circle` / `figure_eight` / `climb` / `stop_go`，或 `uncertain`（置信度不足时）。

**约束**：采集时的标签与命令**不进入模型输入**。识别节点只读取观测流，这一点由代码结构保证，见第 3.4 节。

三个关键取舍：

| 取舍 | 选择 | 理由 |
|---|---|---|
| 特征 vs 端到端 | 先抽 8 个几何特征，再分类 | 训练窗口只有 280 个；直接喂 61×9 原始序列，参数一多就过拟合。几何特征把"转弯有多急""走了多远"显式化，小样本下更稳 |
| 分类器 | MLP（16→32→5） | 特征已在时间维聚合，序列信息被压成统计量，不需要再堆循环层 |
| 序列对照 | 独立训练的 GRU(8→32) | 用来回答"时序建模是否必要"，而不是拿来当主模型 |

### 1.1 与相邻项目的边界

| 方向 | 输入 / 输出 | 本项目区别 |
|---|---|---|
| 状态预测（PR #121） | 历史状态 → 未来位置、速度 | 本项目输出**离散**轨迹类型 |
| 遥测异常检测（PR #138） | 状态 → 异常分数、告警 | 本项目训练正常任务类别，不注入故障 |
| 避障、PPO 导航、PID 跟踪 | 感知/目标 → 控制动作 | 分类与日志节点**不发送飞行控制指令** |
| YOLO 图像检测 | 图像 → 目标框 | 本项目只使用飞行遥测 |

2026-10-02 曾核查 OpenHUTB 主分支目录及最近 100 个 PR 的标题与范围，未发现相同的轨迹分类与任务日志模块；这不代表其他仓库不存在相关研究。上游基线为 `c10e1331a5b24dd2d9a0d37bafe91cecdb5ae92b`。

### 1.2 功能与入口

`main.py` 是统一入口，`main.sh` 负责加载 ROS 环境与虚拟环境：

| 阶段 | 入口 | 输出 |
|---|---|---|
| 自检 | `bash main.sh doctor` | `artifacts/environment.json` |
| 数据准备 | `python main.py prepare` | `data/prepared/*.npz`、`manifest.json` |
| 训练 | `python main.py train --variant mlp --seed 42 --output models/x` | `model.pt`、`config.json`、`loss.json` |
| 复现 | `python reproduce.py --output models/retrained_run` | 5 个模型 + 2 份评估 |
| 评估 | `python main.py evaluate --split final` | `results/*.json`、`*_predictions.csv` |
| 构建 | `bash main.sh build` | `install/`、`build/`、`log/` |
| ROS2 回放 | `bash main.sh demo "<csv;csv>"` | 三个话题 + 看板 |
| 看板 | `python main.py dashboard` | Tk 窗口 |

---
