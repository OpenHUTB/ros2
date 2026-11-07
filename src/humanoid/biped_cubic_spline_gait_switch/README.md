# 基于三次样条插值的双足机器人多步态平滑切换仿真

本项目在 ROS 2 Humble 中实现一个平面双足机器人（躯干 + 双腿，每腿髋/膝/踝 3 个关节）的多步态仿真：
站立、行走、快步、跑步四种步态按时间表切换，切换瞬间用**三次 Hermite 样条**对 6 个关节角做位置与速度连续（C1）的过渡；
摆动脚轨迹由三次多项式（smoothstep 与 `16u²(1-u)²` 抬脚曲线）生成；关节由带速度前馈的 PD 伺服驱动。
控制只使用自身关节反馈，ZMP/质心用于事后评估。

## 与已有双足教程的区别

仓库已有《双足步态生成与 ZMP 稳定性》侧重 LIPM 与 ZMP 基础。本项目聚焦**多步态平滑切换**：

| 项目 | 已有教程 | 本项目 |
| --- | --- | --- |
| 运行环境 | ROS 1 / 纯 Python | ROS 2 Humble |
| 步态数量 | 单一行走步态 | 站立 / 行走 / 快步 / 跑步 |
| 切换方式 | 无 | 三次 Hermite 样条 + 直接切换对照 |
| 平滑性证据 | 无 | 切换窗口 jerk / 角加速度 / 速度跳变对比 |
| 可视化 | 曲线 | 关节轨迹、ZMP、踏地轨迹、GIF |

## 环境

- Ubuntu 22.04，ROS 2 Humble，Python 3.8
- 依赖：`numpy`、`pyyaml`、`matplotlib`、`Pillow`（`python3-pil`）
- 无 Gazebo/MuJoCo 依赖，纯 Python 运动学 + 关节伺服仿真，可在无显示器环境运行

## 目录

```
biped_cubic_spline_gait_switch/
├── biped_cubic_spline_gait_switch/
│   ├── spline.py           # 自然三次样条 + 三次 Hermite
│   ├── kinematics.py       # 双足 IK/FK、质心、支撑多边形
│   ├── gait.py             # 步态定义、调度、平滑切换
│   ├── controller_node.py  # 200 Hz 轨迹生成
│   ├── sim_node.py         # 500 Hz 关节伺服 + 质心/ZMP
│   └── logger_node.py      # 100 Hz CSV 记录
├── config/params.yaml      # 步态参数与调度
├── launch/main.launch.py
├── tools/analyze.py        # 指标、图表、GIF
├── test/                   # pytest
├── results/run_spline/     # 真实运行数据（三次样条）
├── results/run_direct/     # 对照数据（直接切换）
├── main.py                 # 一键实验
└── README.md
```

## 编译

```bash
mkdir -p ~/biped_ws/src
ln -s /path/to/biped_cubic_spline_gait_switch ~/biped_ws/src/
cd ~/biped_ws
colcon build --packages-select biped_cubic_spline_gait_switch --symlink-install
source install/setup.bash
```

## 运行

```bash
cd ~/biped_ws/src/biped_cubic_spline_gait_switch
python3 main.py            # 依次运行 spline 与 direct 两组实验，并生成图表
```

也可单跑一组：

```bash
ros2 launch biped_cubic_spline_gait_switch main.launch.py \
  mode:=spline output_dir:=$PWD/results/run_spline duration:=34.0
```

`mode:=spline` 使用三次样条过渡，`mode:=direct` 在切换时刻直接跳变，作为平滑性对照。

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/joint_cmd` | `sensor_msgs/JointState` | 期望关节角与角速度（200 Hz） |
| `/joint_states` | `sensor_msgs/JointState` | 伺服后的实际关节角 |
| `/base_cmd` | `std_msgs/Float32MultiArray` | `[base_x, v, hip_z, lean]` |
| `/base_state` | `nav_msgs/Odometry` | 浮动基座状态 |
| `/com`、`/zmp` | `geometry_msgs/PointStamped` | 质心与 ZMP |
| `/biped_metrics` | `std_msgs/Float32MultiArray` | 质心、ZMP、裕度、双脚位置与接触 |
| `/gait_info` | `std_msgs/String` | 步态名、相位、切换标志 |

## 算法

### 步态参数

| 步态 | 周期 (s) | 支撑相占比 | 步长 (m) | 抬脚 (m) | 髋高 (m) |
| --- | --- | --- | --- | --- | --- |
| stand | 1.00 | 0.50 | 0.00 | 0.00 | 0.84 |
| walk | 1.20 | 0.62 | 0.22 | 0.10 | 0.82 |
| fast_walk | 0.90 | 0.58 | 0.30 | 0.13 | 0.80 |
| run | 0.60 | 0.38 | 0.42 | 0.18 | 0.78 |

支撑相占比 > 0.5 时存在双支撑（walk/fast_walk），< 0.5 时存在腾空（run）。

### 三次样条切换

切换时刻 `t0` 记录当前关节角 `q0` 与角速度 `dq0`，并预测过渡结束 `t1 = t0 + 0.6 s` 时目标步态的角度 `q1` 与角速度 `dq1`。过渡期内每个关节按下式插值：

```
q(t) = h00(u)·q0 + h10(u)·T·dq0 + h01(u)·q1 + h11(u)·T·dq1
u = (t - t0) / T
```

其中 `h00/h10/h01/h11` 为三次 Hermite 基函数，因而位置与速度连续。摆动脚在 `u∈[0,1]` 内用 `3u²-2u³` 平滑前进，抬脚高度用 `16u²(1-u)²`，两端高度与竖直速度均为 0。

### 评价指标

- 关节跟踪 RMS：`q_des - q_actual`
- 切换窗口最大关节 jerk / 角加速度 / 速度跳变（`t0 ~ t0+0.6 s`）
- ZMP 裕度：ZMP 到支撑多边形边界的最小距离
- 腾空比例、步数、前进距离与平均速度

## 实验结果

以下为本机实际运行数据（34 s 调度，100 Hz 记录，共 3244 条采样；对照组同一参数下运行直接切换）：

| 指标 | 三次样条切换 | 直接切换 |
| --- | --- | --- |
| 前进距离 / 平均速度 | 16.781 m / 0.688 m/s | 同调度 |
| 关节跟踪 RMS | 0.0710 rad | — |
| 切换窗口最大 jerk | **8410.2 rad/s³** | 22112.3 rad/s³ |
| 切换窗口最大角加速度 | **96.91 rad/s²** | 415.14 rad/s² |
| 切换窗口最大速度跳变 | **1.287 rad/s** | 4.951 rad/s |
| jerk 降低 | **62.0%** | — |
| 角加速度降低 | **76.7%** | — |
| ZMP 最小裕度（支撑期） | -0.2502 m | — |
| 腾空比例 / 步数 | 0.044 / 65 | — |

分步态统计：

| 步态 | 采样数 | ZMP 最小裕度 (m) | 跟踪 RMS (rad) |
| --- | --- | --- | --- |
| stand | 917 | 0.0706 | 0.0013 |
| walk | 1154 | -0.1308 | 0.0709 |
| fast_walk | 585 | -0.0455 | 0.0800 |
| run | 588 | -0.2502 | 0.1054 |

ZMP 裕度为负是因为本仿真采用开环轨迹跟踪，没有加入平衡控制器；该指标只用于衡量规划轨迹本身的平滑性，不代表真实接触力。

图表见 `results/run_spline/`：

- `joints.png`：期望/实际关节角
- `velocity.png`：关节角速度
- `jerk.png`：加加速度
- `zmp.png`：质心、ZMP 与裕度
- `feet.png`：双脚轨迹
- `gait_timeline.png`：步态时间线
- `comparison.png`：三次样条 vs 直接切换
- `walking.gif`：行走动画

## 测试

```bash
cd src/humanoid/biped_cubic_spline_gait_switch
python3 -m pytest -q test
```

覆盖自然三次样条插值与 C2 连续性、Hermite 端点速度、IK/FK 往返、脚掌水平、支撑多边形、双支撑/腾空比例、切换平滑性。

## 复现说明

`results/run_spline` 与 `results/run_direct` 均为本机实际运行产生：`main.py` 启动三个 ROS 2 节点，记录 34 s、约 3200 条 100 Hz 采样，再由 `tools/analyze.py` 生成指标与图表。数据未手工修改。

## 大模型使用声明

代码与文档在 AI 编程助手辅助下完成，作者已逐行核对、运行测试并实际采集仿真数据。
