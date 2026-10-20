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

## 2. 仿真环境与运行步骤

### 2.1 环境

| 项 | 配置 |
|---|---|
| 宿主 | Windows 11 + VMware Workstation 16 |
| 虚拟机 | Ubuntu 20.04.6 LTS，6 核 / 8.5 GB，kernel 5.15 |
| ROS | ROS2 **Humble**（`/opt/ros/humble`，为 Python 3.8 构建） |
| 仿真器 | AirSim 1.8.1，Blocks 场景，SimpleFlight 控制器 |
| Python | 3.8.10，venv `--system-site-packages` |
| 深度学习 | PyTorch **2.4.1+cpu**，numpy 1.24.4，matplotlib 3.1.2 |
| 推理设备 | 纯 CPU（无 CUDA） |

### 2.2 安装

```bash
cd ~/ros2/src/air/state_prediction
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages ~/uav_prediction_env
source ~/uav_prediction_env/bin/activate
python -m pip install -r requirements.txt      # CPU 版 torch，约 195 MB
bash main.sh doctor                            # 环境自检，输出 artifacts/environment.json
bash main.sh build                             # colcon 构建
```

`doctor` 会逐一导入 12 个必需模块（numpy、torch、matplotlib、rclpy、std_msgs、lark、launch_ros 及 6 个 colcon 插件），任何一个缺失都会直接报错退出，不会带着残缺环境继续构建。

安装时的四个常见问题：

- **`ensurepip` 缺失**：说明系统没装 `python3-venv`，先 `sudo apt install python3.8-venv`。
- **没有图形窗口**：`topic_plot` 依赖 Tk，需要 `python3-tk`（`import tkinter` 可自检）。
- **`doctor` 报 `launch_ros: No module named 'lark'`**：在模块目录、已激活的 venv 里重跑 `pip install -r requirements.txt` 即可，`lark==1.1.9` 已在依赖表中；不要用跳过 `launch_ros` 检查的方式绕过。
- **Conda 环境冲突**：Conda 的 `python3` 无法导入 ROS2。保留原环境，另建一个基于 `/usr/bin/python3` 的 venv，再跑同一个 requirements 命令。

虚拟环境的优先级由 `main.sh` 决定：`UAV_ENV` > 当前已激活的 `VIRTUAL_ENV` > `~/uav_prediction_env`；`ROS_SETUP` 可覆盖 ROS 发行版的 setup 文件。换目录重建时，旧的 `build/`、`install/`、`log/` **不要**复制过来，否则会指向旧路径。

### 2.3 回放演示

回放不需要启动仿真器，也不需要下载任何场景：

```bash
bash main.sh demo
```

这条命令启动三个节点：`state_source`（回放 CSV 状态）、`state_predictor`（加载 GRU、发布预测）、`topic_plot`（Tk 窗口实时绘图）。窗口内容自上而下为：

- 左上：轨迹平面图，绿色为实测路径，橙色为三个时域的未来位置预测；
- 右上：高度曲线，含未来高度预测；
- 下方：三个时域随目标时间累积的位置 RMSE 曲线。

回放结束后窗口保持显示，按 `Ctrl+C` 退出。

![回放运行时的真实窗口](../img/air/state_prediction/demo_window.png)

### 2.4 为什么是 Tk 绘图而不是 RViz2

这台 Ubuntu 20.04 上的 ROS2 Humble 是既有环境，`rviz2` 启动即失败：

```
rviz2: error while loading shared libraries: libOgreMain.so.1.12.1: cannot open shared object file
```

缺的是 Ogre 1.12 运行库。因此默认用包内的 `topic_plot`（TkAgg）直接从 ROS2 话题绘图：

- 只依赖 Python + matplotlib，不需要 Ogre；
- 话题消费方式与 RViz 完全一致，不影响验证结论；
- `rviz/prediction.rviz` 仍然保留，供图形依赖完整的 ROS2 环境使用。

需要 RViz2 时用 `bash main.sh demo --rviz`。**没有为了这个故障去换机或重装整个虚拟机**——环境是既有的，能验证什么就验证什么，不在文档里假装 RViz 可用。

### 2.5 采集与实时接入

只有在需要重新采集或实时接入时才需要 AirSim 客户端：

```bash
python -m pip install -r requirements_airsim.txt
bash main.sh collect --host <AirSim 主机 IP>
bash main.sh live    --host <AirSim 主机 IP>
```

`collect` 会真正控制仿真无人机飞行五类轨迹；`live` 与 `demo` 只读取状态，**不下发任何飞控指令**。

两点必须注意：

1. `config/airsim_settings.json` 里的 `192.168.239.1` 与 `PredictionDrone` 是示例值，换机器必须按实际的主机地址、车辆名和端口调整，不要照抄。
2. 重新采集的数据**不保证逐值重复**原始实验——采集受实时性、控制器状态和网络延迟影响。要复现本文表格中的数字，请用仓库自带的 `assets/flight_corpus.zip`，不要重新采集。

### 2.6 仿真器选择与验证范围的边界

新部署推荐使用维护者提供的 [OpenHUTB 模拟器发布页](https://github.com/OpenHUTB/hutb/releases)，选择支持无人机的场景与 AirSim 兼容接口。

**必须写清楚的一点**：本文所有训练数据、曲线和截图来自 **Windows Blocks / AirSim 1.8.1** 的实际运行，**尚未在 OpenHUTB 模拟器上重新采集与验证**。AirSim RPC 客户端可以直接复用，但场景、机型与接口版本需要按实际配置核对。把本文的误差数字套用到其他场景时需要重新评估。

### 2.7 回放与源码的关系

`data/sample/episode_026.csv` 同时被 `demo` 回放与第 7.1 节的对比图使用，因此文档中的图表与可运行的演示是同一条轨迹，便于对照检查。


---
