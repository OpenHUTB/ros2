# 基于 GRU 的无人机多时域飞行状态预测与 ROS2 在线误差分析

在 OpenHUTB/ros2 现有 [AirSim 无人机 ROS 桥接示例](https://github.com/OpenHUTB/ros2/tree/master/src/air/air_teleop) 的连接方式和话题解耦思想上扩展。原示例使用 ROS1 `rospy`、相机和速度指令；本模块使用 ROS2 `rclpy`、飞行状态、神经网络预测及未来时刻真实值对齐评估。任务是 **预测**，不执行避障或 PPO 导航。

已在 Windows 11 宿主机 + VMware Ubuntu 20.04.6 + ROS2 Humble + Windows Blocks/AirSim 1.8.1 实际运行。该 Humble/20.04 虚拟机是现有环境，ROS2 节点可运行，但其 RViz2 缺少 Ogre 1.12 图形库；本机使用包内 `topic_plot` 从 ROS2 话题实时绘图。保留 `rviz/prediction.rviz` 供图形依赖完整的 ROS2 环境使用。未因 RViz 故障更换或重装整个虚拟机。

![ROS2 真实回放与在线误差](state_prediction/ros2_window.png)

![架构](state_prediction/docs/architecture.png)

## 功能及文件

| 阶段 | 入口 | 输出 |
|---|---|---|
| 仿真采集 | `python main.py collect` | `data/raw/episode_*.csv/.json` |
| 数据集 | `python main.py prepare` | `data/processed/*.npz`、`manifest.json` |
| 神经网络 | `python main.py train` | `models/<variant>_<seed>/model.pt`、loss CSV |
| 独立测试 | `python main.py evaluate` | `results/metrics.csv`、曲线、消融 |
| ROS2 | `python main.py build`，`./main.sh demo` 或 `./main.sh live` | `/uav/state`、预测路径、误差和可视化 |

`main.py` 为统一入口。`demo` 使用仓库内少量真实仿真数据 `data/sample/episode_026.csv` 回放，能展示完整的 **状态发布 → ROS2 → GRU → 预测发布 → 未来真实值到达 → 误差发布 → 绘图** 流程；`live` 直接从 Windows AirSim RPC 取状态。`collect` 才会控制仿真无人机；演示和实时状态桥接本身不下发飞控指令。

## 环境安装、检查与构建

以下假设仓库位于 `~/ros2`；若克隆位置不同，请修改第一行路径。已有环境可跳过创建；首次安装需使用与系统 ROS2 匹配的 `/usr/bin/python3`，避免 Conda 的 `python3` 被误用：

```bash
cd ~/ros2/src/air/state_prediction
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages ~/uav_prediction_env
source ~/uav_prediction_env/bin/activate
python -m pip install -r requirements.txt
bash main.sh doctor
bash main.sh build
```

`requirements.txt` 一次安装 CPU 版 PyTorch、NumPy、Matplotlib、colcon 构建插件、ROS launch 所需的 Lark 和可选仿真客户端的前置依赖；不需要另装 CUDA。ROS2 的 `rclpy`/消息包和 Tk 来自系统 ROS/Python 安装，不能用 pip 安装 ROS2 代替。若创建 venv 报缺少 ensurepip，先安装系统 `python3-venv`；没有图形窗口时检查 `python3-tk`。

如果旧环境在 `doctor` 中报 `launch_ros: No module named 'lark'`，请在本模块目录、已激活的虚拟环境中重新运行 `python -m pip install -r requirements.txt`，再运行 `bash main.sh doctor`。依赖表已显式包含 `lark==1.1.9`，无需重建环境；不要用隐藏缺包错误或跳过 `launch_ros` 检查的方式继续构建。

已经由 Conda 创建且无法导入 ROS2 的环境，建议保留原环境，另建一个系统 Python venv（例如 `~/ros2_course_env`），激活后运行同一个 requirements 命令。`main.sh` 优先使用 `UAV_ENV` 指定的目录，其次使用当前已激活的 `VIRTUAL_ENV`，最后才使用 `~/uav_prediction_env`；`ROS_SETUP` 可指定 ROS setup 文件。每个新终端都要激活环境，或使用会加载环境的 `main.sh`。

```bash
source ~/uav_prediction_env/bin/activate
```

构建通过当前 Python 调用 `colcon_core.command.main`，不依赖非标准的 `python -m colcon` 兼容模块。`doctor` 检查 ROS2、神经网络库和构建插件，打印实际解释器并保存 `artifacts/environment.json`；该环境记录已随本 PR 提交。

目录和 ROS 包已统一更名为 `state_prediction`。请在新目录重新执行 build；旧路径生成的 `build/`、`install/`、`log/` 不要复制到新目录。回放使用随附模型和数据，**无需先启动或下载仿真器**：

```bash
bash main.sh demo
```

回放结束后窗口保持显示，按 Ctrl+C 退出。需要完整 Ogre 依赖的 RViz2 时使用 `bash main.sh demo --rviz`。

## 仿真器选择与实际验证范围

新部署推荐使用维护者提供的 [OpenHUTB 模拟器发布页](https://github.com/OpenHUTB/hutb/releases)，选择支持无人机的场景及 AirSim 兼容接口。先使用现有场景和回放验证项目，再按需要下载对应平台版本。

本提交的既有训练数据、曲线和截图来自 Windows Blocks / AirSim 的实际运行，**尚未将这些结果重新标为 OpenHUTB 模拟器实测**。可复用 AirSim RPC 客户端，但具体场景、车辆名、主机地址及端口需按实际配置检查。配置示例随代码保存在 `config/airsim_settings.json`；示例中的 `192.168.239.1` 和 `PredictionDrone` 不应在其他机器盲目照抄。

只有重新采集或实时接入才需要额外安装仿真客户端。先完成核心 requirements，再运行：

```bash
python -m pip install -r requirements_airsim.txt
```

核心 requirements 已安装 `numpy` 与 `msgpack-rpc-python`，满足旧版 AirSim 安装脚本的前置导入要求；回放不导入 AirSim。重新采集程序会控制仿真无人机；回放、推理和只读桥接不下发飞控指令。

## 重现训练与测试

仓库附带原始 30 段真实仿真数据压缩包 `assets/flight_corpus.zip`（约 1.6 MiB），首次 prepare 自动解压并校验 CSV 哈希。先用随附的部署模型直接重算指标（此时只评估两个物理基线和 `gru_44`，不伪造缺失的消融模型）：

```bash
bash main.sh prepare
bash main.sh evaluate --output results/checkpoint_check
```

完整三种子训练和消融使用新目录，保留发布模型：

```bash
bash main.sh prepare
bash main.sh train --output models/retrained
bash main.sh evaluate --models models/retrained --output results/retrained
```

随附部署模型仍为原验证集选择的 `gru_44`，现有结果来自原始实验。也可用 `bash main.sh collect --host <仿真主机IP>` 重新采集，但重新采集产生的新数据不保证逐值重复原始实验。

## 数据定义与方法

每回合包含 `timestamp_ns`、位置 `px/py/pz`、线速度 `vx/vy/vz`、加速度 `ax/ay/az`、四元数 `qw/qx/qy/qz`、角速度 `wx/wy/wz`、本次命令速度、碰撞标志和采集时钟。位置、线速度和线加速度使用 AirSim 世界 **NED** 坐标系（米、米/秒、米/秒²）；姿态是机体 **FRD** 到世界 NED 的四元数；角速度为机体 FRD。ROS2 绘图时转换为 **ENU**，并在 `Odometry` 中给出匹配的姿态与机体系速度。训练输入没有未来控制指令。

飞行轨迹有直线往返、圆形、8 字、升降、平滑停走五类；振幅、频率和相位按记录的种子变化。每类 6 次，共 30 个独立回合、8,430 条真实 AirSim 状态。采集时检查时间戳单调、碰撞和安全范围；所有回合碰撞数为 0。采样中位间隔约 0.051 秒；数据处理用真实仿真时间戳插值到 20 Hz，最大间隔超过 0.15 秒则拒绝回合。四元数先做符号连续化，再插值和归一化。

先按**完整回合**划分训练/验证/测试：各类分别为 4/1/1 回合，即 20/5/5 回合。再从每个回合生成过去 20 帧（约 1 秒）输入、未来 0.25/0.5/1.0 秒位置与速度标签。最终窗口数为 4,820 / 1,205 / 1,205。窗口不跨回合。标准化参数仅从训练窗口拟合，验证集用于提前停止及从 3 个随机种子中选部署模型；测试集不用于模型选择。随机种子为 42/43/44。

模型是 1 层、48 隐单元 GRU，预测相对于**恒速模型**的残差；损失是归一化残差 MSE，Adam，学习率 0.002，批量 128，最多 45 轮，验证损失连续 8 轮无改善即停止。两个物理基线分别是假定恒速、恒加速度。消融：`no_acc_att` 去掉加速度、姿态和角速度，只用位置与速度历史；`mlp` 只使用当前帧完整状态。部署模型 `gru_44` 由验证损失最小原则选出。

## 真实实验结果

独立测试集共 5 回合、1,205 个**重叠**预测窗口。下表 RMSE/MAE 是 XYZ 三轴、所有窗口的平均，位置单位米，速度单位米/秒；逐回合结果和向量范数 RMSE 在 `results/episode_metrics.csv`、`results/metrics.csv`。重叠窗口不是独立样本，不能把 1,205 当作独立飞行次数。

| 模型 | 时域 | 位置 MAE | 位置 RMSE | 速度 MAE | 速度 RMSE |
|---|---:|---:|---:|---:|---:|
| 恒速 | 0.25 s | 0.004302 | 0.006983 | 0.034224 | 0.055405 |
| 恒速 | 0.5 s | 0.016950 | 0.027381 | 0.066956 | 0.107859 |
| 恒速 | 1.0 s | 0.065469 | 0.105059 | 0.127754 | 0.204840 |
| 恒加速度 | 0.25 s | 0.000416 | 0.000781 | 0.004648 | 0.008948 |
| 恒加速度 | 0.5 s | 0.002950 | 0.005787 | 0.016972 | 0.033891 |
| 恒加速度 | 1.0 s | 0.021354 | 0.042784 | 0.061036 | 0.122541 |
| GRU `gru_44` | 0.25 s | 0.000287 | 0.000444 | 0.002145 | 0.003305 |
| GRU `gru_44` | 0.5 s | 0.001026 | 0.001624 | 0.004304 | 0.007145 |
| GRU `gru_44` | 1.0 s | 0.005363 | 0.009265 | 0.016877 | 0.030767 |

1 秒位置 RMSE 的三种子平均±标准差：完整 GRU **0.00899±0.00030 m**；去加速度/姿态/角速度 **0.04709±0.00453 m**；单帧 MLP **0.01203±0.00035 m**。这说明本任务中惯性与姿态信息、历史序列都提供了增益；在当前简单轨迹中，单帧 MLP 仍然表现较好，不能推广到任意动态任务。

![训练与验证损失](state_prediction/results/loss.png)
![预测时域误差](state_prediction/results/horizon_errors.png)
![真实与预测对比](state_prediction/results/prediction_comparison.png)
![消融实验](state_prediction/results/ablation.png)

模型在同一 Blocks 场景、规则小范围轨迹与同一 SimpleFlight 控制器上训练和测试。未评估强风、碰撞、随机遥控、不同地图和真实无人机；数毫米误差只在本实验条件下成立。若未来控制动作不可知，复杂机动的 1 秒预测通常会更难。原始数据、种子、结果和部署模型随本模块保存；提交上游时遵循仓库对二进制与大数据的限制，附带原始数据压缩包和选定部署模型，其他训练权重可用上述入口重建。

## ROS2 话题与验证

`/uav/state`：含 `schema_version`、整数纳秒时间戳、16 维 NED/FRD 状态的 JSON `std_msgs/String`；`/uav/odometry`：已转换为 ENU/FLU 的标准 `nav_msgs/Odometry`；`/uav/truth_path` 和 `/uav/predicted_path`：ENU `nav_msgs/Path`；`/uav/prediction_markers`：未来位置与指标；`/uav/forecast`：3 个时域预测值和推理耗时；`/uav/errors`：真实值到达时发布的累计 MAE/RMSE。保留 JSON 原始时间戳是为了不丢纳秒精度；演示用的回放文件与该话题遵循同一数据结构。

在线节点保存待评估预测，每个目标时间 `t+0.25/0.5/1.0 s` 到达后，在相邻真实状态间插值并计算误差。回放实测 281 条状态、83 次预测、236 条对齐误差，三个时域都有有效匹配；CPU 推理耗时中位数约 1.05 ms。`results/replay_flow_check.json` 是机器检查记录，`ros2_window.png` 是真实 Ubuntu 窗口截图。`results/online_errors.jsonl` 可保存逐次误差，运行生成，不作为固定离线测试结果。

## 仓库提交说明

将本目录放入 `src/air/state_prediction/`，将配套 `docs/air/state_prediction.md` 加到 `mkdocs.yml` 空域载具导航和 `docs/index.md` 首页。按上游约定验证 `mkdocs serve --livereload`，提交前检查大文件与截图尺寸；上游要求另一台机器由其他人测试并同意才可合并。`data/raw`、`data/processed` 和所有实验检查点默认由 `.gitignore` 排除；选定小模型及样例数据可提交。

本项目部分设计、代码与文档使用 OpenAI Codex 辅助生成与检查；测试数据、曲线和截图均来自本机实际运行。提交者需审核代码并对提交内容负责。
