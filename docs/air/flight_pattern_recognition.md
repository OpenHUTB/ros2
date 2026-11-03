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

## 2. 运行环境与快速开始

### 2.1 环境

| 项 | 配置 |
|---|---|
| 系统 | Ubuntu 20.04.6，6 核 / 8.5 GB |
| ROS | ROS2 **Humble**（`/opt/ros/humble`，为 Python 3.8 构建） |
| Python | 3.8.10，venv `--system-site-packages` |
| 深度学习 | PyTorch **2.4.1+cpu**，numpy 1.24.4，matplotlib 3.1.2 |
| 数据来源 | AirSim 1.8.1（Blocks 场景）录制，随模块附 30 段 + 15 段语料 |

### 2.2 安装与自检

```bash
cd ~/ros2/src/air/flight_pattern_recognition
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages ~/uav_prediction_env
source ~/uav_prediction_env/bin/activate
python -m pip install -r requirements.txt
bash main.sh doctor      # 12 个必需模块逐一导入，缺一个就退出
bash main.sh build
```

`doctor` 会打印实际解释器并写入 `artifacts/environment.json`；不通过时直接报错，不会带着残缺环境继续。

安装时的四个常见问题：

- **`ensurepip` 缺失**：先 `sudo apt install python3.8-venv`；
- **没有图形窗口**：看板依赖 Tk，需要 `python3-tk`；
- **`doctor` 报 `launch_ros: No module named 'lark'`**：在模块目录、已激活的 venv 里重跑 `pip install -r requirements.txt`，依赖表已含 `lark==1.1.9`；
- **Conda 环境冲突**：Conda 的 `python3` 无法导入 ROS2，另建基于 `/usr/bin/python3` 的 venv。

虚拟环境优先级：`UAV_ENV` > 当前 `VIRTUAL_ENV` > `~/uav_prediction_env`；`ROS_SETUP` 可覆盖 ROS 发行版的 setup 文件。换目录重建时旧的 `build/`、`install/`、`log/` 不要复制过来。

### 2.3 跑通回放

```bash
python main.py prepare                     # 首次运行：解压语料、校验哈希、生成窗口
bash main.sh demo "$PWD/data/final_raw/episode_300.csv;$PWD/data/final_raw/episode_302.csv"
```

`main.sh demo` 走 `ros2 launch`，起两个节点：`pattern_replay`（按时间戳回放）与 `pattern_recognizer`（识别）。看板是独立进程，另开一个终端：

```bash
python main.py dashboard
```

看板不在 launch 里，是为了让识别链路可以脱离图形界面运行——没有显示器的机器只订阅话题即可。**先开看板再起 demo**：五段回放（`episode_300`～`304`）约 95 秒后自然结束，回放进程退出会连带关停识别节点。

### 2.4 采集与实时接入（可选）

重新采集才需要 AirSim 客户端：

```bash
python -m pip install -r requirements_airsim.txt
python collect_final.py --host <AirSim 主机 IP> --start 400 --episodes 5 --output data/new_collection
```

`collect_final.py` 会真正控制仿真无人机，含碰撞、区域边界、起点稳定与降落确认；采集器拒绝覆盖已有文件。新增数据**不属于**随附最终测试集，若用于训练需要重新设计完整划分并保留新的未见测试集。

`config/airsim_settings.json` 里的主机地址与载具名是示例值，换机器必须按实际配置修改。回放、推理与只读桥接都不下发飞控指令。

只读实时桥接（读正在运行的 AirSim，不控制）：

```bash
python main.py live --ros-args -p host:=192.168.239.1 -p vehicle:=PredictionDrone
ros2 run flight_pattern_recognition pattern_recognizer --ros-args \
  -p model:="$PWD/models/mlp_42" -p log:="$PWD/results/live_predictions.jsonl"
```

飞行本身需要独立控制器。五类训练数据**不覆盖起飞、降落、悬停、故障**等阶段，这些阶段的输出没有准确性保证。

### 2.5 验证边界

本页所有数据来自 **AirSim Blocks 场景**、规则小范围轨迹、SimpleFlight 控制器，**尚未在 OpenHUTB 模拟器上重新采集**。把这里的准确率套到别的场景或真实无人机上不成立。

---

## 3. 数据、标签与防泄漏

### 3.1 原始字段

每段飞行一个 CSV，逐行记录 `timestamp_ns`、位置 `px/py/pz`、速度 `vx/vy/vz`、加速度 `ax/ay/az`（世界 **NED**，m 与 m/s、m/s²）。标签来自采集时执行的轨迹家族，记录在对应的 JSON 里。

数据来自实际运行的 AirSim `getMultirotorState`，随模块保存 CSV、采集参数、时间戳与 SHA256——**不是**按轨迹方程直接生成的理想状态。轨迹方程只用于生成控制目标，数据里包含仿真飞行控制器的实际响应。

五个标签对应采集任务家族，含义如下：

| 标签 | 采集任务含义 |
|---|---|
| `line` | 水平直线往返，正弦速度变化 |
| `circle` | 水平圆形轨迹 |
| `figure_eight` | 水平八字轨迹 |
| `climb` | 水平位移伴随周期性**升降**，不是单向爬升 |
| `stop_go` | 平滑停留/换向，不是硬切换停车 |

后两条容易被字面意思误导：`climb` 并非一直向上，`stop_go` 也不是急停急走。

### 3.2 窗口构造

对每段飞行，从第 7 秒开始，每 0.5 秒取一个窗口：

- 取窗口末端往前 **6.0 s** 的观测；
- 在 `[-6, 0]` 秒上等间隔取 **61** 个点（10 Hz），逐通道线性插值；
- 计算 8 个几何特征（第 4.1 节）。

跳过前 7 秒是为了避开采集开始时的悬停与加速瞬态——这一点写在代码注释里，也是 `config.json` 里 `warmup_s=7` 的来源。

采样间隔超过 150 ms、时间戳非单调、含非有限值的窗口直接丢弃。

这 8 个特征对**整体平移**与**绕竖直轴旋转**不变（只用到相对位移与水平分量），但**不**对所有飞行速度或姿态变化不变——速度大小、转弯急缓、升降幅度都会改变特征值，这正是分类所依赖的。

### 3.3 数据划分

| 集合 | 回合 | 每类 | 窗口 | 用途 |
|---|---|---:|---:|---|
| 训练（`000`–`019`） | 20 | 4 | 280 | 拟合权重与标准化参数 |
| 验证（`020`–`024`） | 5 | 1 | 70 | 选权重、定置信阈值 |
| 保留测试（`025`–`029`） | 5 | 1 | 70 | 只报告，不参与任何选择 |
| 最终测试（`300`–`314`） | 15 | 3 | 315 | 冻结模型后另采，每类 3 次 |

旧语料 30 段共 **8,430** 条状态，最终测试 15 段共 **5,416** 条，合计 **13,846** 条有效状态。

final 集是**另外 15 段**飞行，与前三集在回合 ID 与文件 SHA256 上都无交集，且每类恰好 3 段——`prepare_final.py` 会对这两条做断言，不满足就报错。

采集过程中有一段被质量规则拒绝：第 `308` 段首次采集出现**重复时间戳与 0.576 秒停顿**，按规则丢弃；用相同种子与任务参数重采后通过。失败的 CSV 留在数据包的 `rejected_raw/`，原始与替代哈希记在 `results/collection_quality.json`。**替代发生在读取任何最终分类结果之前**，不影响指标。

### 3.4 防泄漏

1. **标签不进输入**：`pattern_data.geometry()` 只接受位置/速度/加速度，签名里没有任何标签或命令字段。
2. **窗口不跨回合**：窗口在单段飞行内生成，`prepare_final.py` 还校验 final 与训练/测试的哈希不重叠。
3. **标准化只拟合训练集**：均值与标准差存在 `config.json`，推理时复用同一份。
4. **测试集不参与选择**：阈值与权重只看验证集（`train.py` 的阈值策略显式写在验证集上）。
5. **冻结协议**：`results/frozen_protocol.json` 记录关键文件的 SHA256，final 集准备前先校验，改动过就直接失败。

---

## 4. 计算原理

### 4.1 八个几何特征

记窗口内第 \(t\) 帧的水平速度 \(\mathbf{v}_{xy}(t)=(v_x,v_y)\)、水平加速度 \(\mathbf{a}_{xy}(t)=(a_x,a_y)\)、水平位置 \(\mathbf{p}_{xy}(t)=(p_x,p_y)\)。八个特征逐帧计算：

$$
\begin{aligned}
f_1(t) &= \big\|\mathbf{v}_{xy}(t)\big\|_2 &&\text{水平速度}\\
f_2(t) &= v_z(t) &&\text{垂直速度}\\
f_3(t) &= \frac{\mathbf{v}_{xy}(t)\cdot\mathbf{a}_{xy}(t)}{\max(\|\mathbf{v}_{xy}(t)\|_2,\ 0.05)} &&\text{切向加速度}\\
f_4(t) &= \frac{v_x a_y - v_y a_x}{\max(\|\mathbf{v}_{xy}(t)\|_2,\ 0.05)} &&\text{带符号法向加速度}\\
f_5(t) &= a_z(t) &&\text{垂直加速度}\\
f_6(t) &= \big\|\mathbf{p}_{xy}(t)-\mathbf{p}_{xy}(0)\big\|_2 &&\text{水平位移}\\
f_7(t) &= p_z(t)-p_z(0) &&\text{相对高度}\\
f_8(t) &= \frac{f_6(t)}{L(t)},\quad L(t)=\sum_{k=1}^{t}\big\|\mathbf{p}_{xy}(k)-\mathbf{p}_{xy}(k-1)\big\|_2 &&\text{路径效率}
\end{aligned}
$$

三处设计意图：

- **分母下限 0.05 m/s**：悬停时速度接近 0，切向与法向加速度的定义会炸开；钳住分母让"停住"成为一个稳定状态而不是噪声放大器。
- **法向加速度带符号**：左转与右转在 8 字轨迹里方向相反，取绝对值会把两者抹平。
- **路径效率 \(f_8\)**：直线时位移≈路程，效率接近 1；绕圈时位移远小于路程，效率明显小于 1。它是区分"直线"与"闭合曲线"最直接的一个量。

### 4.2 时间维聚合与标准化

特征在时间维取均值与标准差（\(\sigma\) 用总体标准差，`unbiased=False`）：

$$
\mu_j=\frac{1}{61}\sum_{t=1}^{61}f_j(t),\qquad
\sigma_j=\sqrt{\frac{1}{61}\sum_{t=1}^{61}\big(f_j(t)-\mu_j\big)^2}
$$

拼成 16 维向量 \(\mathbf{z}=[\mu_1..\mu_8,\sigma_1..\sigma_8]^{\top}\)，再用训练集的统计量标准化：

$$
\tilde{\mathbf{z}}=\frac{\mathbf{z}-\mathbf{m}}{\max(\mathbf{s},\,0.02)}
$$

下限 0.02 防止某个特征在训练集上几乎不变时被除爆。

### 4.3 主模型：两层 MLP

$$
\mathbf{h}=\mathrm{ReLU}\big(\mathbf{W}_1\tilde{\mathbf{z}}+\mathbf{b}_1\big)\in\mathbb{R}^{32},\qquad
\mathbf{o}=\mathbf{W}_2\mathbf{h}+\mathbf{b}_2\in\mathbb{R}^{5}
$$

参数量 \(16\times32+32 + 32\times5+5 = 709\)。

### 4.4 序列对照：GRU

$$
\mathbf{h}_t=\mathrm{GRU}\big(\mathbf{f}(t),\mathbf{h}_{t-1}\big),\quad
\mathbf{u}=\mathrm{ReLU}\big(\mathbf{W}_g\mathbf{h}_{61}+\mathbf{b}_g\big)\in\mathbb{R}^{24},\quad
\mathbf{o}=\mathbf{W}_o\mathbf{u}+\mathbf{b}_o
$$

其中 GRU 使用 PyTorch `nn.GRU` 的约定（重置门作用于隐藏态线性变换的结果，更新门加权上一时刻隐状态）。它吃的是未经时间聚合的 61×8 特征序列。

### 4.5 softmax 与损失

$$
p_k=\frac{\exp(o_k)}{\sum_{j=1}^{5}\exp(o_j)},\qquad
\mathcal{L}=-\frac{1}{N}\sum_{n=1}^{N}\log p_{n,\,y_n}
$$

### 4.6 置信阈值与拒识

模型总会给出一个 argmax，但低置信度的预测不该变成任务事件。阈值 \(\tau\) 在**验证集**上按固定规则选：

$$
\tau^{*}=\min\Big\{\tau\in[0.20,0.95]\ \big(\text{步长 }0.01\big)\ :\ \mathrm{cov}(\tau)\ge 0.20\ \wedge\ \mathrm{acc}_{\text{acc}}(\tau)\ge 0.90\Big\}
$$

其中 \(\mathrm{cov}(\tau)=\frac{1}{N}\big|\{n:p_{\max}^{(n)}\ge\tau\}\big|\) 是被接受的窗口比例，\(\mathrm{acc}_{\text{acc}}\) 是被接受窗口的准确率。取满足条件的最小阈值，等价于**在保证 90% 接受准确率的前提下最大化覆盖率**。没有候选时回落到 0.80。

低于阈值时输出 `uncertain`，而不是硬选一个类。

### 4.7 评价指标

$$
\mathrm{acc}=\frac{1}{N}\sum_{n}\mathbb{1}\big[\hat{y}_n=y_n\big],\qquad
P_k=\frac{\mathrm{TP}_k}{\mathrm{TP}_k+\mathrm{FP}_k},\qquad
R_k=\frac{\mathrm{TP}_k}{\mathrm{TP}_k+\mathrm{FN}_k}
$$

$$
F1_k=\frac{2P_kR_k}{P_k+R_k},\qquad
\text{macro-F1}=\frac{1}{5}\sum_{k=1}^{5}F1_k
$$

宏平均让五个类权重相同——本数据每类窗口数相等，宏平均与微平均接近，但在类别均衡被打破时宏平均更能暴露短板。

另外报告**回合级准确率**：对一个回合内所有窗口的预测取众数，与回合标签比较。它衡量"整段飞行是否被认对"，比逐窗口准确率更接近实际使用。

---
